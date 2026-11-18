"""Real-ring preparation actions; the caller owns the robot and its lifecycle.

The individual methods allow the behavior tree to compose these actions later.
This module does not create ROS nodes or send commands at import time.
"""


class RingRealPrepareSkill:
    def __init__(self, robot, config, initial, hold_left, keep_claws, waypoint_with_left_hold, move_head):
        self.robot = robot
        self.config = config
        self.keep_claws = keep_claws
        self.waypoint_with_left_hold = waypoint_with_left_hold
        self.move_head = move_head
        self.left_hold = initial[4:11].tolist() if hold_left else None
        if self.left_hold is not None:
            robot.left_hold_anchor = robot.np.asarray(self.left_hold, dtype=float)
            print('PREPARE: 保持阶段开始时实测的左臂关节姿态', flush=True)

    def torso(self):
        options = self.config['integrated']
        print('PREPARE: 躯干先回到配置的初始位置', flush=True)
        self.robot.move_torso(self.robot.np.radians(options.get('prepare_torso_deg', options['home_torso_deg'])))

    def claws(self):
        if self.keep_claws:
            print('PREPARE: 保持夹爪当前状态，不下发夹爪命令', flush=True)
        else:
            self.robot.move_claws(self.config['prepare']['claw_percent'], purpose='prepare')

    def head(self):
        options = self.config['integrated']
        self.move_head(self.robot, *options['head_yaw_pitch_deg'], speed_deg_s=options['head_speed_deg_s'],
                       endpoint_tolerance_deg=2.5)

    def arms(self):
        points = self.config['prepare']['arm_waypoints_deg']
        for index, point in enumerate(points):
            target = self.waypoint_with_left_hold(point, self.left_hold) if self.left_hold is not None else point
            self.robot.move_arms(target, hold_at_end=index == len(points) - 1,
                                 **({'coordinated_prepare': True} if index == 0 else {}))

"""Composable real-ring carry actions; robot lifecycle belongs to the caller."""


class RingRealCarrySkill:
    def __init__(self, robot, config, hold_left, waypoint_with_left_hold):
        self.robot = robot
        self.config = config
        self.waypoint_with_left_hold = waypoint_with_left_hold
        self.left_hold = None
        self.observed = self.base_anchor = None
        self.move_left_to_initial = bool(config['carry'].get('move_left_to_initial', False))
        if hold_left and not self.move_left_to_initial:
            observed, _, base = robot.fresh()
            observed, _, _ = robot.checked_feedback(observed, base)
            self.observed, self.base_anchor = observed, base
            self.left_hold = observed[4:11].tolist()
            robot.left_hold_anchor = robot.np.asarray(self.left_hold, dtype=float)
            print('CARRY: 保持当前实测左臂关节；右臂使用搬运路点', flush=True)

    def arms(self, coordinated_final=False):
        points = self.config['carry']['arm_waypoints_deg']
        for index, point in enumerate(points):
            target = (self.waypoint_with_left_hold(point, self.left_hold)
                      if self.left_hold is not None else point)
            last = index == len(points) - 1
            if last and coordinated_final:
                self.robot.move_arms(target, hold_at_end=True, coordinated_carry=True)
            else:
                self.robot.move_arms(target, hold_at_end=last)
        if self.move_left_to_initial:
            observed, _, base = self.robot.fresh()
            observed, _, _ = self.robot.checked_feedback(observed, base)
            self.left_hold = observed[4:11].tolist()
            self.robot.left_hold_anchor = self.robot.np.asarray(self.left_hold, dtype=float)
            self.observed, self.base_anchor = observed, base
            print('CARRY: 双臂到 initial 后记录左臂实测姿态', flush=True)

    def torso(self):
        self.robot.move_torso(self.robot.np.radians(self.config['carry']['torso_deg']))

    def observe(self, observer):
        self.robot.wait_native_still()
        observed, _, base = self.robot.fresh()
        observer(self.robot, observed, base, observed[4:18].copy(),
                 self.config['carry'].get('posture_observation_s', 5.0))
        self.observed, self.base_anchor = observed, base

    def base(self, run_base, config_path, skip):
        if not skip:
            self.robot.mutated = True
        run_base(self.config, config_path, skip)
        if not skip and self.left_hold is not None:
            self.robot.checked_feedback(self.observed, self.base_anchor, allow_base=True)

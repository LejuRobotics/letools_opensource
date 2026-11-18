"""Composable real-ring placement actions using a caller-owned robot session."""


class RingRealPlaceSkill:
    def __init__(self, robot, config, left_hold, waypoint_with_left_hold):
        self.robot = robot
        self.config = config
        self.left_hold = left_hold
        self.waypoint_with_left_hold = waypoint_with_left_hold

    def forward(self, initial, base, height_mode):
        from scipy.spatial.transform import Rotation
        robot, config = self.robot, self.config
        options = config['integrated']
        position, rotation = robot.right_ee_fk(base)
        if options['place_position_mode'] == 'relative_to_current':
            target_pos = position.copy()
            target_pos[0] += options['place_forward_m']
            target_pos[1] += options['place_lateral_offset_m']
            if height_mode == 'fixed':
                target_pos[2] = options['place_height_m']
            print('PLACE height mode:', height_mode, 'z(m):', float(target_pos[2]), flush=True)
        else:
            target_pos = robot.np.asarray(options['place_local_xyz_m'], dtype=float)
        if not robot.np.isfinite(target_pos).all():
            raise RuntimeError('Computed placement contains non-finite coordinates')
        desired_rotation = (rotation if options['place_orientation_mode'] == 'current_fk'
                            else Rotation.from_euler('ZYX', options['place_yaw_pitch_roll_deg'], degrees=True))
        offset = float(options.get('place_tool_rotation_deg', 0.0))
        if not robot.np.isfinite(offset):
            raise ValueError('Invalid local tool rotation')
        desired_rotation = desired_rotation * Rotation.from_euler('z', offset, degrees=True)
        pitch = float(options.get('place_base_pitch_deg', 0.0))
        if not robot.np.isfinite(pitch):
            raise ValueError('Invalid base-frame pitch')
        desired_rotation = Rotation.from_euler('y', pitch, degrees=True) * desired_rotation
        orientation = desired_rotation.as_euler('ZYX', degrees=True).tolist()
        target = target_pos.tolist() + orientation
        print('PLACE local FK target (m,deg):', target, flush=True)
        robot.start_arm(initial, base)
        robot.ee_local(target, config['place']['move_duration_s'])

    def release(self):
        robot, config = self.robot, self.config
        wait = config['integrated'].get('release_wait_for_still', True)
        if wait:
            print('PLACE: wait for arms to settle before claw release', flush=True)
            robot.wait_for_retreat_settle(config['integrated']['settle_timeout_s'])
        else:
            print('PLACE: 开爪前不检查手臂稳定阈值', flush=True)
        positions, _, _ = robot.claw_feedback()
        robot.move_claws([float(positions[0]), config['place']['open_percent']], purpose='release')
        if wait:
            robot.wait_for_retreat_settle(config['integrated']['settle_timeout_s'])

    def retreat(self):
        from scipy.spatial.transform import Rotation
        robot, settings = self.robot, self.config['place']
        _, _, base = robot.fresh()
        position, rotation = robot.right_ee_fk(base)
        adjusted = Rotation.from_euler('y', settings['retreat_pitch_offset_deg'], degrees=True) * rotation
        retreat_pos = position.copy()
        retreat_pos[0] -= settings['retreat_m']
        target = retreat_pos.tolist() + adjusted.as_euler('ZYX', degrees=True).tolist()
        print('RETREAT local FK target (m,deg):', target, flush=True)
        # Continue the EE trajectory after release without the joint-stability gate.
        # timed() still checks fresh feedback, joint limits and measured arrival.
        robot.ee_local(target, settings['retreat_duration_s'], require_still=False)
        robot.keep_after_retreat()

    def home(self):
        robot, config = self.robot, self.config
        config['limits']['arm_speed_deg_s'] = config['integrated']['return_arm_speed_deg_s']
        for index, point in enumerate(config['place']['return_waypoints_deg']):
            if index == len(config['place']['return_waypoints_deg']) - 1:
                config['limits']['arm_speed_deg_s'] = config['integrated'].get(
                    'home_final_arm_speed_deg_s', config['integrated']['return_arm_speed_deg_s'])
            if index > 0 and config['place'].get('home_both_after_initial', False):
                self.left_hold = None
                robot.left_hold_anchor = None
            target = (self.waypoint_with_left_hold(point, self.left_hold)
                      if self.left_hold is not None else point)
            robot.move_arms(target, hold_at_end=index == len(config['place']['return_waypoints_deg']) - 1)
        config['limits']['torso_speed_deg_s'] = config['integrated']['home_torso_speed_deg_s']
        target = robot.np.radians(config['integrated']['home_torso_deg'])
        robot.torso_limits(target)
        robot.move_torso(target)

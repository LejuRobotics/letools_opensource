float64 initTime           # 对应 scalar_t initTime_
float64[] initState        # 对应 vector_t initState_
float64 finalTime          # 对应 scalar_t finalTime_

# ModeSchedule 和 TargetTrajectories 
mode_schedule modeSchedule
mpc_target_trajectories targetTrajectories

# 包含 PrimalSolution 
mpc_flattened_controller mpc_flattened_controller

# swingplanner存储的swingPlannerMultipliers
float64[] swingPlannerMultipliers

# Control modes
int8 POSITION_CONTROL = 0  # Position control mode
int8 VELOCITY_CONTROL = 1  # Velocity control mode

# Message header
std_msgs/Header header

# Control mode to be used
int8 control_mode

# Data array
# 数据数组，单手时长度必须为6，双手长度必须为12
# - 位置控制模式下，每个元素的数据的范围为[0, 100], 0 为完全打开，100 为完全关闭
# - 速度控制模式下，每个元素的数据的范围为[-100, 100] 负数表示打开，正数表示关闭
int16[] data
## 具体使用案例请参考如下文件，为整体的数据流参考，但不作为最终展示的案例
## 分层结构

```text
04_use_uwb_to_move UWB 移动示例
├── 数据输入层
│   └── UWB 定位数据                    # 提供目标或机器人相对位置信息
│
├── 服务/逻辑层
│   └── 示例目录内脚本                  # 根据 UWB 数据组织移动逻辑
│
└── 运动执行层
    └── 底盘移动控制                    # 将定位结果转为机器人移动行为
```
```bash
/home/kuavo/kuavo_ros_application/src/dynamic_biped/examples/04_use_uwb_to_move
```

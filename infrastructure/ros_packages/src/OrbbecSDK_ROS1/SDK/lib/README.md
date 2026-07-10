# Orbbec SDK library
## 在 LeTools 中的位置

```text
OrbbecSDK_ROS1/SDK/lib
├── 相机驱动依赖层
│   └── Orbbec 闭源 SDK 动态库、深度引擎、驱动文件
│
├── ROS 相机驱动层
│   └── OrbbecSDK_ROS1 使用这些库访问奥比中光相机
│
└── 上层应用层
    └── LeTools 的相机测试、视觉检测、AprilTag 等功能依赖相机图像输出
```

> 该目录主要存放厂商 SDK 库文件，一般不需要新手修改。

This Directory contains the Orbbec SDK library, depth engine and drivers files. Those files are close source now and will be open source in the near future.

The license for those files can be found in the LICENSE file in the root directory of the Orbbec SDK repository.


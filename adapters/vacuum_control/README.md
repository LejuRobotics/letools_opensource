# 真空控制系统 - 轻量化迁移指南

## 📦 目录结构

```
vacuum_control/              # 真空控制系统完整模块
├── __init__.py             # 包入口
├── relay.py                # 继电器控制（气泵+破真空）
├── pressure_sensor.py      # 气压传感器读取
├── test_vacuum.py          # 测试脚本
└── requirements.txt        # Python 依赖
```

---

## 🚀 快速迁移步骤

### 1️⃣ 复制整个模块
将整个 `vacuum_control` 文件夹复制到您的新项目中即可。

### 2️⃣ 安装依赖
```bash
cd vacuum_control
pip install -r requirements.txt
```

### 3️⃣ 测试运行
```bash
python test_vacuum.py
```

---

## 💻 使用示例

### 基础使用
```python
from vacuum_control import control_vacuum_pump, control_relay, read_pressure_kpa
import time

# 1. 吸气
control_vacuum_pump(relay_index=0, action='ON')
time.sleep(2)

# 2. 读取气压
left_pressure = read_pressure_kpa(sensor_id=2)
right_pressure = read_pressure_kpa(sensor_id=1)
print(f"左臂: {left_pressure} kPa, 右臂: {right_pressure} kPa")

# 3. 松开
control_vacuum_pump(relay_index=0, action='OFF')
control_relay(relay_index=1, action='ON')  # 破真空
time.sleep(0.2)
control_relay(relay_index=1, action='OFF')
```

---

## 📋 依赖清单


- `pymodbus` - Modbus 协议支持（控制继电器）
- `minimalmodbus` - 简易 Modbus（读取气压）
- `pyserial` - 串口通讯基础

---

## 🔌 硬件连接

| 设备 | 串口设备(Linux) | 说明 |
|------|----------------|------|
| 继电器 | `/dev/kuavo_relay` | 控制气泵和破真空 |
| 右臂气压传感器 | `/dev/kuavo_pressure_right` | `sensor_id=1` |
| 左臂气压传感器 | `/dev/kuavo_pressure_left` | `sensor_id=2` |

---

## ⚙️ API 文档

### `control_vacuum_pump(relay_index, action, duration)`
控制气泵开关

**参数：**
- `relay_index`: 0（气泵）或 1（破真空）
- `action`: 'ON' 或 'OFF'
- `duration`: 持续时间（秒）

**返回：** `bool` - 是否成功

---

### `control_relay(port, slave_id, relay_index, action)`
直接控制继电器（底层接口）

**参数：**
- `port`: 串口号（None 则自动查找）
- `slave_id`: Modbus 从站 ID
- `relay_index`: 继电器索引
- `action`: 'ON' 或 'OFF'

**返回：** `bool` - 是否成功

---

### `read_pressure_kpa(sensor_id)`
读取气压值

**参数：**
- `sensor_id`: 1（右臂）或 2（左臂）

**返回：** `float` 气压值(kPa)，失败返回 `None`

---

## ⚠️ 注意事项

1. **串口权限**：在 Linux 上可能需要给串口设备读写权限
   ```bash
   sudo chmod 666 /dev/ttyUSB*
   ```

2. **继电器索引**：
   - 0 = 气泵
   - 1 = 破真空

3. **传感器 ID**：
   - 1 = 右臂
   - 2 = 左臂

---

## 📊 模块说明

| 文件 | 功能 | 迁移必要性 |
|------|------|-----------|
| `__init__.py` | 包入口 | ✅ 必须 |
| `relay.py` | 继电器控制 | ✅ 必须 |
| `pressure_sensor.py` | 气压传感器 | ✅ 必须 |
| `test_vacuum.py` | 测试脚本 | ⚪ 可选 |
| `requirements.txt` | 依赖列表 | ✅ 必须 |

---

## ✅ 迁移检查清单

- [ ] `vacuum_control` 文件夹完整复制
- [ ] 安装了 3 个依赖库
- [ ] 串口设备有读写权限
- [ ] 运行 `test_vacuum.py` 测试通过
- [ ] 在您的业务代码中导入使用

---

## 🎉 完成！


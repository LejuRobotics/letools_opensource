# 02 扩展 core 层

`core` 是不依赖项目内其他层的契约层。往这里加东西，改的是**所有人共同遵守的约定**。

> 落层判定与依赖方向见 `01-扩展总则与分层规范`。`core` 依赖任何其它层都是越界，没有例外。

## 一、新增接口契约

`core/interfaces/` 下现有四份接口，粒度对齐即可新增：

| 接口 | 职责 |
|---|---|
| `i_hardware.py` | 硬件能力统一入口（底盘 / 手臂 / 腿部 / 头部 / 力控 / 模式 / 反馈 / 末端执行器） |
| `i_skill.py` | 技能生命周期契约 |
| `i_camera.py` | 相机能力 |
| `i_perception.py` | 感知能力 |

新增步骤：

1. 新建 `core/interfaces/i_<capability>.py`，用 `abc` 或普通基类定义方法签名；
2. 方法**返回值统一用 `core/domain/result.py` 的 `Result`**，不要返回裸 `bool` 或抛异常跨层传递失败；
3. 参数尽量用 `core/domain/` 下已有的模型（`Pose6D`、`JointState`、`Trajectory` 等）；
4. 写 docstring 说明每个方法的能力边界；
5. 在 `core/interfaces/__init__.py` 导出；
6. 在对应层的实现类里补实现（`adapters` 实现 `IHardware`/`ICamera`，`skills` 实现 `ISkill`）。

**保持抽象粒度一致**：接口描述的是「能力」而不是「某个厂商怎么实现」。厂商特有的方法（`_jiateng` / `_jibot` 后缀那类）不得进 `IHardware`，应留在 `adapters` 的 Mixin 内。

## 二、新增领域模型

`core/domain/` 下现有模型分两类：

- **返回值/状态类**：`result.py`（`Result`）、`observation.py`、`joint_state.py`、`tag.py`、`perception.py`、`camera.py`
- **参数类**：`skill_params.py`（`SkillParams` 基类）、`pose.py`、`trajectory.py`、`chassis_options.py`、`ruckig_params.py`、`end_effector.py`、`task.py`

新增步骤：

1. 新建 `core/domain/<name>.py`，用 `@dataclass` 定义；
2. **参数对象继承 `SkillParams`**（如 `MoveToPoseParams`、`PickObjectParams`），这样超时/重试/`extra_kwargs` 等公共字段自动带上，也能被配置反序列化复用；
3. **返回值统一用 `Result`**：`Result.ok(msg, data)` / `Result.fail(msg, error_code, data)`；
4. 字段给默认值，避免构造时必填项堆叠；
5. 在 `core/domain/__init__.py` 导出。

`SkillParams` 基类实际提供的公共字段（`core/domain/skill_params.py`）：

```python
@dataclass
class SkillParams:
    skill_name: str = ""
    timeout: float = 30.0        # 技能执行超时时间（秒）
    retry_count: int = 3         # 失败重试次数
    extra_kwargs: Dict[str, Any] = field(default_factory=dict)
```

## 三、新增枚举

`core/domain/enums.py` 集中放跨层共享的枚举（现有 `FrameType`、`MPCControlMode`、`ArmSide`）。新增时：

- 枚举成员带显式 `value`，不要依赖隐式自增（会被序列化进 JSON / ROS 消息）；
- 在 `core/domain/__init__.py` 导出，保持 `from core.domain import XxxEnum` 可用。

> ⚠ **注意**：当前固件会接收并回显 `MPCControlMode` 值，但不按它选择实际控制分支；现有接口调用仍可能承担兼容、软暂停守卫或控制流设置。详见 [`控制路径与选型`](../03-接口详细文档/09-控制路径与选型.md) 与 [`术语表`](../01-介绍/07-术语表.md)。新增枚举前应确认消费方和实际控制效果。

## 四、新增异常

- 领域通用异常放 `core/common/exceptions.py`；
- 层内一次性错误优先用 `Result.fail(msg, error_code=...)` 返回，而不是新增异常类型；
- 若确需异常，命名以 `Error` 结尾并带可读 message。

## 五、红线

- `core` **不得** import `drivers` / `adapters` / `skills` / `orchestration` / `apps`。目前 `core` 没有例外（E1–E3 均不涉及 `core`）。所有层都依赖 `core`，`core` 一旦依赖其他层就会形成循环依赖。
- 第三方依赖只用通用库（当前为 `numpy`、`scipy`、`PyYAML`、`colorlog`），不引入 ROS、SDK 或具体硬件相关的包；新增第三方依赖需同步更新仓库根 `requirements.txt`。

## 六、验证

- 单测入口：`core/common/tests/`（已列入 `pytest.ini` 的 `testpaths`，直接 `pytest` 会拾起）；
- 新增接口后至少补一条「契约测试」，断言返回值类型是 `Result` / 是某个 `SkillParams` 子类。

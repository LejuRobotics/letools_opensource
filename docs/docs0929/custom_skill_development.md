# 自定义Skill开发指南

[返回 README](../README.md) · [Skill模板](../skills/templates/custom_skill/README.md)

## 开发流程

1. 确认`IHardware`已有所需能力；没有时先设计通用接口和适配器；
2. 复制`skills/templates/custom_skill/`并重命名；
3. 用dataclass声明参数、单位、默认值和timeout；
4. `on_initialize`校验参数，不产生运动；
5. `on_execute`调用一个明确的硬件能力并返回`Result`；
6. 实现`cancel`和`is_finished`；
7. 使用FakeHardware覆盖成功、失败、超时、取消；
8. 再创建薄节点和场景JSON；
9. 按dry-run → MuJoCo → 受控真机顺序验收。

## 禁止事项

- Skill直接依赖行为树JSON；
- Node直接调用SDK/API绕过Skill；
- 在代码中写入未经标定的安全阈值；
- dry-run初始化ROS、相机或真实硬件；
- 忽略`Result.error_code`和取消语义。

## 验收

```bash
pytest skills/templates/custom_skill/test_custom_skill.py -m unit
```

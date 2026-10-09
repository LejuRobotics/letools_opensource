# 自定义Skill模板

1. 复制本目录到合适的`skills/atomic/<domain>/`；
2. 重命名类、参数类和`skill_name`；
3. 在`on_execute()`中只调用`IHardware`能力；
4. 为成功、失败、超时和取消补充测试；
5. 使用节点模板做薄封装。

不得在模板中直接导入厂商SDK或把机器人安全阈值写成无来源常量。

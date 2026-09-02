# -*- coding: utf-8 -*-
"""skills 层测试包：单独运行/测试 LeTools/skills 下的原子技能。

对齐 T4（apps/test_kuavo_5w_sdk_adapter）目录风格，但测试粒度为「技能层」——
调用 SkillBase 子类的生命周期（initialize → execute → is_finished），
而非直接调用 adapter 方法，体现分层架构中技能层与适配器层的区别。
"""

# orchestration/engine — 行为树引擎

编排核心，负责 JSON → py_trees 树构建 + 50Hz 主循环驱动。

## 文件职责

| 文件 | 职责 |
|------|------|
| `behavior_tree_factory.py` | **核心工厂**：从 `py_tree.json` / `py_tree_child.json` 构建 py_trees 行为树。递归解析 JSON → 创建复合节点（Sequence/Selector/Parallel）→ 扫描 `orchestration/nodes/` 自动发现叶子节点 → 动态 import 并实例化。支持子树引用、黑板注入、宏替换、并行加载。 |
| `behavior_tree_controller.py` | **运行控制器**：封装 50Hz 主循环 `tick()`，监控根节点状态（SUCCESS/FAILURE → 终止），提供 ROS 服务注册（start/pause/resume）。 |
| `behavior_tree_engine.py` | **（遗留）轻量引擎**：早期顺序执行引擎，不依赖 py_trees。已被 `BehaviorTreeFactory` + `BehaviorTreeController` 取代。 |
| `py_trees_compat.py` | **py_trees 版本兼容层**：补丁 0.7.x / 2.x 版本的 Blackboard Client API 差异。`run_behavior_tree_json.py` 在 import factory 之前优先导入此模块。 |

## 调用关系

```
run_behavior_tree_json.py
  │
  ├── import py_trees_compat          ← 必须最先导入，打补丁
  │
  ├── BehaviorTreeFactory(blackboard, subtree_json_path)
  │     ├── load_tree_from_json()     ← 读取 py_tree.json
  │     ├── _build_tree_recursive()   ← 递归构建
  │     ├── _handle_subtree()         ← 子树引用展开
  │     ├── _create_node_instance()   ← 叶子节点 → 扫描 nodes/
  │     └── _build_node_index()       ← 扫描 nodes/*.py 建立类名索引
  │
  └── BehaviorTreeController(factory)
        └── start_behavior_tree()     ← 50Hz 主循环 tick
```

## 节点发现机制

`_build_node_index()` 扫描 `orchestration/nodes/` 目录，对每个 `*.py` 文件：
- 注册 `stem`（文件名去 .py）→ 模块路径
- 注册 `SnakeToPascal(stem)` → 模块路径
- JSON 中 `"name"` 字段匹配任一 key 即可找到节点类

## 关键设计

- **子树复用**：子 JSON 通过 `copy.deepcopy` 隔离状态，同一子树可被多处引用而不互相干扰
- **并行加载**：`enable_parallel_loading=True` 时用 ThreadPoolExecutor 并行构建子节点
- **黑板**：通过 `py_trees.blackboard.Client` 在主树加载前写入 board.json，叶子节点通过 `self.global_blackboard` 读写

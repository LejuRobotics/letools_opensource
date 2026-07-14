import ast
import os
import json
import uuid


def parse_manifest_from_file(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        tree = ast.parse(f.read(), filename=filepath)

    manifests = []

    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):

            for decorator in node.decorator_list:
                if isinstance(decorator, ast.Call) and getattr(decorator.func, "id", None) == "define_manifest":

                    manifest = {}
                    kw_map = {kw.arg: kw for kw in decorator.keywords}

                    # 1 判断是否存在 outputs
                    tree = kw_map.get("tree_type")
                    if not tree:
                        continue

                    # 2 解析 value
                    tree_value = ast.literal_eval(tree.value)

                    # 3 如果是空字符串跳过
                    if isinstance(tree_value, str) and tree_value.strip() == "":
                        continue
                    for kw in decorator.keywords:
                        key = kw.arg
                        value = ast.literal_eval(kw.value)
                        manifest[key] = value

                    manifest["class_name"] = node.name
                    manifest["file"] = filepath

                    manifests.append(manifest)

    return manifests


def scan_folder(folder):
    results = []

    for root, _, files in os.walk(folder):
        for file in files:
            if file.endswith(".py"):
                path = os.path.join(root, file)

                try:
                    manifests = parse_manifest_from_file(path)
                    results.extend(manifests)
                except Exception as e:
                    print(f"parse failed: {path} -> {e}")

    return results


from collections import defaultdict

# 全局变量
GLOBAL_INIT_BOARD = []
GLOBAL_INIT_KEYS = set()
def webTree(manifests):
    skill_node = next((x for x in treeObject if x["name"] == "skill"), None)

    if not skill_node:
        return treeObject

    # 1 按 tree_type 分组
    groups = defaultdict(list)
    for m in manifests:
        groups[m.get("tree_type", "default")].append(m)

    result = []

    # 2 遍历每个组
    loop = 1
    for group_name, nodes in groups.items():
        loop += 1
        parent = {
            "name": group_name,
            "label": group_name,
            "inputs": [],
            "order": loop,
            "drag": False,
            "children": []
        }

        # 3 生成 children
        for i, m in enumerate(nodes, start=1):

            inputs = []
            params = m.get("params", [])
            init_board = []
            read_board = []
            write_board = []
            outputs = m.get("inputs", [])

            for j, p in enumerate(params, start=1):
                inputs.append({
                    "key": p.get("name"),
                    "label": p.get("description", p.get("name")),
                    "purpose": "CUSTOM",
                    "type": p.get("type", "string"),
                    "order": j
                })

            for j, p in enumerate(outputs, start=1):
                default_value = p.get("default_value")
                if not default_value or str(default_value).strip() == "":
                    r_item = {
                        "key": p.get("default_key"),
                        "remark": p.get("description", p.get("name")),
                        "type": p.get("type", "string"),
                    }
                    read_board.append(r_item)
                    continue
                item = {
                    "key": p.get("default_key"),
                    "remark": p.get("description", p.get("name")),
                    "type": p.get("type", "string"),
                    "value": default_value,
                    "defaultValue": default_value
                }
                init_board.append(item)

                # 全局去重
                key = item["key"]
                if key not in GLOBAL_INIT_KEYS:
                    GLOBAL_INIT_KEYS.add(key)
                    GLOBAL_INIT_BOARD.append(item)

            o_board = m.get("outputs", [])
            for j, p in enumerate(o_board, start=1):
                write_board.append({
                    "key": p.get("default_key",""),
                    "label": p.get("description", p.get("default_key")),
                    "type": p.get("type", "string"),
                    "order": j
                })

            child = {
                "name": m["class_name"],
                "label": m.get("label", m["class_name"]),
                "parentName": group_name,
                "inputs": inputs,
                "order": i,
                "drag": True,
                "initBoard": init_board,
                "readBoard": read_board,
                "writeBoard":write_board,
                "children": []
            }

            parent["children"].append(child)

        result.append(parent)

    skill_node["children"] = result

    return treeObject


treeObject = [
    {
        "name": "TreeNode",
        "label": "TreeNode",
        "inputs": [],
        "order": 0,
        "drag": False,
        "children": [],
    },
    {
        "name": "assembly",
        "label": "assembly",
        "inputs": [],
        "order": 3,
        "drag": False,
        "children": [
            {
                "name": "choose",
                "label": "choose",
                "parentName": "assembly",
                "inputs": [],
                "order": 1,
                "drag": False,
                "children": [
                    {
                        "name": "Selector",
                        "label": "Selector",
                        "parentName": "choose",
                        "shape": "CUSTOM_OCTAGON",
                        "inputs": [
                            {
                                "key": "memory",
                                "label": "memory",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 1
                            }
                        ],
                        "order": 1,
                        "drag": True,
                        "children": []
                    }
                ]
            },
            {
                "name": "concurrency",
                "label": "concurrency",
                "parentName": "assembly",
                "inputs": [],
                "order": 2,
                "drag": False,
                "children": [
                    {
                        "name": "Parallel",
                        "label": "Parallel",
                        "parentName": "concurrency",
                        "shape": "CUSTOM_PARALLELOGRAM",
                        "inputs": [
                            {
                                "key": "policy",
                                "label": "policy",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 1
                            },
                            {
                                "key": "Selected-labels",
                                "label": "Selected-labels",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 2
                            }
                        ],
                        "order": 1,
                        "drag": True,
                        "children": []
                    }
                ]
            },
            {
                "name": "decorate",
                "label": "decorate",
                "parentName": "assembly",
                "inputs": [],
                "order": 4,
                "drag": False,
                "children": [
                    {
                        "name": "Condition",
                        "label": "Condition",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [
                            {
                                "key": "status",
                                "label": "status",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 1
                            }
                        ],
                        "order": 1,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "Count",
                        "label": "Count",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [],
                        "order": 2,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "Decorator",
                        "label": "Decorator",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [],
                        "order": 3,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "EternalGuard",
                        "label": "EternalGuard",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [
                            {
                                "key": "condition",
                                "label": "condition",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 1
                            },
                            {
                                "key": "blackboard_keys",
                                "label": "blackboard_keys",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 2
                            }
                        ],

                        "order": 4,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "FailureIsRunning",
                        "label": "FailureIsRunning",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [],

                        "order": 5,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "FailureIsSuccess",
                        "label": "FailureIsSuccess",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [],
                        "order": 6,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "ForEach",
                        "label": "ForEach",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [
                            {
                                "key": "source_key",
                                "label": "source_key",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 1
                            },
                            {
                                "key": "target_key",
                                "label": "target_key",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 2
                            }
                        ],

                        "order": 7,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "Inverter",
                        "label": "Inverter",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [],

                        "order": 8,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "OneShot",
                        "label": "OneShot",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [
                            {
                                "key": "policy",
                                "label": "policy",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 1
                            }
                        ],

                        "order": 9,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "PassThrough",
                        "label": "PassThrough",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [],

                        "order": 10,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "Repeat",
                        "label": "Repeat",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [
                            {
                                "key": "num_success",
                                "label": "num_success",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 1
                            }
                        ],

                        "order": 11,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "Retry",
                        "label": "Retry",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [
                            {
                                "key": "num_failures",
                                "label": "num_failures",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 1
                            }
                        ],

                        "order": 12,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "RunningIsFailure",
                        "label": "RunningIsFailure",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [],

                        "order": 13,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "RunningIsSuccess",
                        "label": "RunningIsSuccess",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [],

                        "order": 14,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "StatusToBlackboard",
                        "label": "StatusToBlackboard",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [
                            {
                                "key": "variable_name",
                                "label": "variable_name",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 1
                            }
                        ],

                        "order": 15,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "SuccessIsFailure",
                        "label": "SuccessIsFailure",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [],

                        "order": 16,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "SuccessIsRunning",
                        "label": "SuccessIsRunning",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [],

                        "order": 17,
                        "drag": True,
                        "children": []
                    },
                    {
                        "name": "Timeout",
                        "label": "Timeout",
                        "parentName": "decorate",
                        "shape": "CUSTOM_RECT",
                        "inputs": [
                            {
                                "key": "duration",
                                "label": "duration",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 1
                            }
                        ],

                        "order": 18,
                        "drag": True,
                        "children": []
                    }
                ]
            },
            {
                "name": "serial",
                "label": "serial",
                "parentName": "assembly",

                "inputs": [],
                "order": 3,
                "drag": False,
                "children": [
                    {
                        "name": "Sequence",
                        "label": "Sequence",
                        "parentName": "serial",
                        "shape": "CUSTOM_RECT",
                        "inputs": [
                            {
                                "key": "memory",
                                "label": "memory",
                                "purpose": "CUSTOM",
                                "type": "string",
                                "order": 1
                            }
                        ],

                        "order": 1,
                        "drag": True,
                        "children": []
                    }
                ]
            }
        ]
    },
    {
        "name": "common",
        "label": "common",
        "inputs": [],
        "order": 1,
        "drag": False,
        "children": [
            {
                "name": "base",
                "label": "base",
                "parentName": "common",
                "inputs": [],
                "order": 1,
                "drag": False,
                "children": [
                    {
                        "name": "root",
                        "label": "root",
                        "parentName": "base",
                        "shape": "CUSTOM_RECTCOMP",
                        "inputs": [],
                        "order": 1,
                        "drag": True,
                        "children": []
                    }
                ]
            }
        ]
    },
    {
        "name": "custom",
        "label": "custom",
        "inputs": [],
        "order": 4,
        "drag": False,
        "children": []
    },
    {
        "name": "skill",
        "label": "skill",
        "inputs": [],
        "order": 2,
        "drag": False,
        "children": [
        ]
    }
];
############################################
###创建node
node_map = {}


def build_node_library_index(node_library):
    def dfs(node, path):

        current = path + [node]

        if node.get("drag", False):
            parent_source = []

            for level, p in enumerate(current):
                parent_source.append({
                    "key": p["name"],
                    "title": p["label"],
                    "level": level
                })

            node_map[node["name"]] = {
                "shape": node.get("shape", "CUSTOM_RECTCOMP"),
                "inputs": node.get("inputs", []),
                "parentSource": parent_source
            }

        for child in node.get("children", []):
            dfs(child, current)

    for root in node_library:
        dfs(root, [])

    # return node_map


def generate_node_id():
    return str(uuid.uuid4())

def normalize_params(params: dict) -> dict:
    result = {}
    for k, v in params.items():
        # 如果已经是目标结构（包含 value），直接保留
        if isinstance(v, dict) and "value" in v:
            result[k] = v
        else:
            # 否则转换
            result[k] = {
                "value": v,
                "source": "CUSTOM"
            }
    return result

def py_tree_to_page_tree_custom(py_node, x=0, y=0, x_gap=200, y_gap=150):
    """
    转换 py_tree 到页面结构，满足：
    - 同级节点按下标靠左排列
    - 孙子节点在父节点上方
    """
    # 当前节点
    node_id = generate_node_id()
    node_name = py_node["name"]

    if node_name.endswith(".json"):
        parent_source = [
            {"key": "TreeNode", "title": "", "level": 0},
            {"key": node_name, "title": "", "level": 1}
        ]
        input_p = []
    else:
        try:
            parent_source = node_map.get(node_name, {}).get("parentSource", [])
            input_p = node_map[py_node["name"]]["inputs"]
        except KeyError:
            print(f"----- %s ------ ", node_name)
    page_node = {
        "cells": "node",
        "data": {
            "fetchData": {
                "label": py_node["name"],
                "shape": "CUSTOM_RECTCOMP",
                "data":{},
                "key": py_node["name"],
                "parentSource": parent_source,
                "isFillAll": True,
                "inputs":input_p
            },
            "handleData": {
                "description": py_node["label"],
                "name": py_node["name"],
                # "inparams": {k: v for k, v in py_node.get("params", {}).items()}
                "inparams": normalize_params(py_node.get("params", {}))
            }
        },
        "id": node_id,
        "label": py_node["label"],
        "position": {"x": x, "y": y},
        "shape": "CUSTOM_RECTCOMP",
        "size": {"width": 148, "height": 107}
    }

    nodes = [page_node]
    edges = []

    childs = py_node.get("childs", [])
    if not childs:
        return nodes, edges

    # 子节点的初始 x 坐标：左起
    child_x = x - x_gap * (len(childs) - 1) / 2
    child_y = y + y_gap  # 孙子在父节点上方

    for child in childs:
        child_nodes, child_edges = py_tree_to_page_tree_custom(child, child_x, child_y, x_gap, y_gap)
        nodes.extend(child_nodes)
        edges.extend(child_edges)

        # 连接父子节点
        edges.append({
            "cells": "edge",
            "data": {},
            "id": generate_node_id(),
            "label": None,
            "source": {"cell": node_id, "port": "bottom"},
            "target": {"cell": child_nodes[0]["id"], "port": "top"},
            "tools": {"items": []}
        })
        child_x += x_gap  # 同级节点右移

    return nodes, edges


def convert_py_tree_to_page(py_tree):
    nodes, edges = py_tree_to_page_tree_custom(py_tree, x=500, y=500)
    return {
        "root": nodes[0],
        "nodes": nodes,
        "edges": edges
    }

def convert_py_tree(py_tree,py_interface):
    py_tree["name"] = "root"
    nodes, edges = py_tree_to_page_tree_custom(py_tree, x=500, y=500)
    return {
        "root": nodes[0],
        "nodes": nodes,
        "edges": edges,
        "interface": py_interface
    }

def convertOldBoard(data):
    # 读取原始文件
    result = {"process": []}

    # 遍历结构
    for module_name, items in data.items():
        for item in items:
            new_item = item.copy()
            new_item["key"] = f"{module_name}_{item['key']}"
            result["process"].append(new_item)

    return result

def read_json(filename):
    path = os.path.join(INPUT_DIR, filename)
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def ensure_output_dir():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

def write_json(filename, data):
    path = os.path.join(OUTPUT_DIR, filename)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    # print(f"输出文件: {path}")


_ORCH_DIR = os.path.dirname(os.path.abspath(__file__))
_STUDIO_ROOT = os.path.dirname(_ORCH_DIR)
OUTPUT_DIR = os.path.join(_ORCH_DIR, "web_ui")
INPUT_DIR = os.path.join(_ORCH_DIR, "scenarios", "studio_smoke_v1")


if __name__ == "__main__":
    # 1. 转 node（阶段 1：扫描 orchestration/nodes）
    ensure_output_dir()
    print("-----node start-----")
    folder = os.path.join(_ORCH_DIR, "nodes")
    manifests = scan_folder(folder)
    grouped = webTree(manifests)

    # with open("tree.json", "w", encoding="utf-8") as f:
    #     json.dump(grouped, f, indent=2, ensure_ascii=False)
    write_json("node.json", grouped)

    print("-----node end-----")
    # 构建tree字典

    build_node_library_index(grouped)

    # 2.黑板
    print("-----board.json start-----")
    write_json("board.json", {"process": GLOBAL_INIT_BOARD})

    # write_json("board_new.json", {"process": GLOBAL_INIT_BOARD})
    # with open("board_web.json", "w", encoding="utf-8") as f:
    #     json.dump({"flow":GLOBAL_INIT_BOARD}, f, indent=2, ensure_ascii=False)
    print("-----board.json end-----")

    # print("-----board.json start-----")
    # board_data = read_json("board.json")
    #
    # write_json("board.json", convertOldBoard(board_data))
    # print("-----board.json end-----")

    # 3. 转主树
    print("-----py_tree.json start-----")
    py_main_data = read_json("py_tree.json")
    # with open("py_tree.json", "r", encoding="utf-8") as f:
    #     py_main_data = json.load(f)
    main_py = convert_py_tree(py_main_data["tree"],py_main_data.get("interface", {}))

    write_json("tree_main_web.json", main_py)

    # with open("tree_web.json", "w", encoding="utf-8") as f:
    #     json.dump(main_py, f, indent=2, ensure_ascii=False)
    print("-----py_tree.json end-----")

    #4. 转子树
    print("-----py_tree_child.json start-----")
    py_child_data = read_json("py_tree_child.json")

    # with open("py_tree_child.json", "r", encoding="utf-8") as f:
    #     py_child_data = json.load(f)
    child_web_tree = []
    for filename, content in py_child_data.items():
        try:
            res = {
                "label": filename,
                "name": filename,
                "flow": convert_py_tree_to_page(content["tree"]),
                "inputs": [],
                "childBoard": [],
            }
            child_web_tree.append(res)
        except Exception:
            print(filename+"   error")

    # with open("tree_web_child.json", "w", encoding="utf-8") as f:
    #     json.dump(child_web_tree, f, indent=2, ensure_ascii=False)
    write_json("tree_child_web.json", child_web_tree)
    print("-----tree_child_web.json end-----")
    # print(json.dumps(manifests, indent=2, ensure_ascii=False))
    # print(json.dumps(child_web_tree, indent=2, ensure_ascii=False))






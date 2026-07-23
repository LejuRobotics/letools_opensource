# encoding: utf-8
"""This file includes necessary params, info."""
import os
import mmcv
import os.path as osp

import numpy as np

# ---------------------------------------------------------------- #
# ROOT PATH INFO
# ---------------------------------------------------------------- #
cur_dir = osp.abspath(osp.dirname(__file__))
root_dir = osp.normpath(osp.join(cur_dir, ".."))
# directory storing experiment data (result, model checkpoints, etc).
output_dir = osp.join(root_dir, "output")

data_root = osp.join(root_dir, "datasets")
bop_root = osp.join(data_root, "BOP_DATASETS/")


# --------------------------------------------------------- #
# Basket DATASET
# ---------------------------------------------------------------- #
dataset_root = os.path.join(bop_root, "basket")

train_real_dir = os.path.join(dataset_root, "train")
# train_render_dir = osp.join(dataset_root, "train_render")

test_dir = os.path.join(dataset_root, "test")

# 3D models
model_dir = os.path.join(dataset_root, "models")
model_eval_dir = os.path.join(dataset_root, "models")
# scaled models (.obj)
# model_scaled_dir = osp.join(dataset_root, "models_rescaled")

# train_scenes = ["{:06d}".format(i) for i in range(50)]
# test_scenes = ["{:06d}".format(i) for i in range(1, 2)]

# id2obj = {1: "basket_4322"}
# id2obj = {1: "gripper"}

id2obj = {
    1: "basket_4322",
    2: "basket_4622",
    3: "basket_4611",
    4: "basket_4633",
    5: "basket_4311",
}

objects = list(id2obj.values())
obj_num = len(id2obj)
obj2id = {_name: _id for _id, _name in id2obj.items()}

model_paths = [osp.join(model_dir, "obj_{:06d}.ply").format(_id) for _id in id2obj]
texture_paths = None
model_colors = [((i + 1) * 10, (i + 1) * 10, (i + 1) * 10) for i in range(obj_num)]  # for renderer

# diameters = (np.array([524.275]))

# Camera info
width = 640
height = 480
zNear = 0.25
zFar = 6.0
center = (height / 2, width / 2)

camera_matrix = np.eye(3) # np.loadtxt(os.path.join(dataset_root, "intrinsics_orbbec.txt"))
vertex_scale = 0.001


def get_models_info():
    """key is str(obj_id)"""
    models_info_path = osp.join(model_dir, "models_info.json")
    assert osp.exists(models_info_path), models_info_path
    models_info = mmcv.load(models_info_path)  # key is str(obj_id)
    return models_info


# ref core/gdrn_modeling/tools/basket/basket_1_compute_fps.py
def get_fps_points():
    fps_points_path = osp.join(model_dir, "fps_points.pkl")
    assert osp.exists(fps_points_path), fps_points_path
    fps_dict = mmcv.load(fps_points_path)
    return fps_dict


# ref core/gdrn_modeling/tools/basket/basket_1_compute_keypoints_3d.py
def get_keypoints_3d():
    keypoints_3d_path = osp.join(model_dir, "keypoints_3d.pkl")
    assert osp.exists(keypoints_3d_path), keypoints_3d_path
    kpts_dict = mmcv.load(keypoints_3d_path)
    return kpts_dict
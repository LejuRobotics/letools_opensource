# Author: Tomas Hodan (hodantom@cmp.felk.cvut.cz)
# Center for Machine Perception, Czech Technical University in Prague

"""Parameters of the BOP datasets."""

import math
import glob
import os
from os.path import join

from lib.pysixd import inout


def get_camera_params(datasets_path, dataset_name, cam_type=None):
    """Returns camera parameters for the specified dataset.

    Note that parameters returned by this functions are meant only for simulation
    of the used sensor when rendering training images. To get per-image camera
    parameters (which may vary), use path template 'scene_camera_tpath' contained
    in the dictionary returned by function get_split_params.

    :param datasets_path: Path to a folder with datasets.
    :param dataset_name: Name of the dataset for which to return the parameters.
    :param cam_type: Type of camera.
    :return: Dictionary with camera parameters for the specified dataset.
    """

    cam_filename = "camera.json"

    # Path to the camera file.
    cam_params_path = join(datasets_path, dataset_name, cam_filename)

    p = {
        # Path to a file with camera parameters.
        "cam_params_path": cam_params_path
    }

    # Add a dictionary containing the intrinsic camera matrix ('K'), image size
    # ('im_size'), and scale of the depth images ('depth_scale', optional).
    p.update(inout.load_cam_params(cam_params_path))

    return p


def get_model_params(datasets_path, dataset_name, model_type=None):
    """Returns parameters of object models for the specified dataset.

    :param datasets_path: Path to a folder with datasets.
    :param dataset_name: Name of the dataset for which to return the parameters.
    :param model_type: Type of object models.
    :return: Dictionary with object model parameters for the specified dataset.
    """
    # Name of the folder with object models.
    models_folder_name = "models" if model_type is None else f"models_{model_type}"

    # Discover object IDs from the models folder; no fallback
    models_path_probe = join(datasets_path, dataset_name, models_folder_name)

    ids_found = []
    if os.path.isdir(models_path_probe):
        for fname in os.listdir(models_path_probe):
            if fname.startswith("obj_") and fname.endswith(".ply"):
                stem = fname[len("obj_"):-len(".ply")]
                if stem.isdigit():
                    ids_found.append(int(stem))
    if not ids_found:
        raise ValueError(f"No object models found in {models_path_probe}")

    obj_ids = sorted(ids_found)

    # ID's of objects with ambiguous views evaluated using the ADI pose error
    # function (the others are evaluated using ADD). See Hodan et al. (ECCVW'16).
    symmetric_obj_ids = obj_ids

    # Path to the folder with object models.
    models_path = join(datasets_path, dataset_name, models_folder_name)

    p = {
        # ID's of all objects included in the dataset.
        "obj_ids": obj_ids,
        # ID's of objects with symmetries.
        "symmetric_obj_ids": symmetric_obj_ids,
        # Path template to an object model file.
        "model_tpath": join(models_path, "obj_{obj_id:06d}.ply"),
        # Path to a file with meta information about the object models.
        "models_info_path": join(models_path, "models_info.json"),
    }

    return p


def get_split_params(datasets_path, dataset_name, split, split_type=None):
    """Returns parameters (camera params, paths etc.) for the specified
    dataset.

    :param datasets_path: Path to a folder with datasets.
    :param dataset_name: Name of the dataset for which to return the parameters.
    :param split: Name of the dataset split ('train', 'val', 'test').
    :param split_type: Name of the split type (e.g. for T-LESS, possible types of
      the 'train' split are: 'primesense', 'render_reconst').
    :return: Dictionary with parameters for the specified dataset split.
    """
    p = {
        "name": dataset_name,
        "split": split,
        "split_type": split_type,
        "base_path": join(datasets_path, dataset_name),
        "depth_range": None,
        "azimuth_range": None,
        "elev_range": None,
    }

    rgb_ext = ".png"
    gray_ext = ".png"
    depth_ext = ".png"

    p["im_modalities"] = ["rgb", "depth"]

    # Basket
    if dataset_name == "basket":
        p["scene_ids"] = None
        p["im_size"] = (640, 480)

        if split == "test":
            p["depth_range"] = (500.0, 2000.0)  # Range of camera-object distances.
            p["azimuth_range"] = (0, 2 * math.pi)
            p["elev_range"] = (-0.4363, 0.5 * math.pi)  # (-25, 90) [deg].

    else:
        raise ValueError("Unknown BOP dataset ({}).".format(dataset_name))

    base_path = join(datasets_path, dataset_name)
    split_path = join(base_path, split)
    if split_type is not None and split_type != "bb8":
        if split_type == "pbr":
            p["scene_ids"] = list(range(50))
        split_path += "_" + split_type

    p.update(
        {
            "base_path": base_path,
            # Path to the split directory.
            "split_path": split_path,
            # Path template to a file with per-image camera parameters.
            "scene_camera_tpath": join(split_path, "{scene_id:06d}", "scene_camera.json"),
            # Path template to a file with GT annotations.
            "scene_gt_tpath": join(split_path, "{scene_id:06d}", "scene_gt.json"),
            # Path template to a file with meta information about the GT annotations.
            "scene_gt_info_tpath": join(split_path, "{scene_id:06d}", "scene_gt_info.json"),
            # Path template to a file with the coco GT annotations.
            "scene_gt_coco_tpath": join(split_path, "{scene_id:06d}", "scene_gt_coco.json"),
            # Path template to a gray image.
            "gray_tpath": join(split_path, "{scene_id:06d}", "gray", "{im_id:06d}" + gray_ext),
            # Path template to an RGB image.
            "rgb_tpath": join(split_path, "{scene_id:06d}", "rgb", "{im_id:06d}" + rgb_ext),
            # Path template to a depth image.
            "depth_tpath": join(
                split_path,
                "{scene_id:06d}",
                "depth",
                "{im_id:06d}" + depth_ext,
            ),
            # Path template to a mask of the full object silhouette.
            "mask_tpath": join(
                split_path,
                "{scene_id:06d}",
                "mask",
                "{im_id:06d}_{gt_id:06d}.png",
            ),
            # Path template to a mask of the visible part of an object silhouette.
            "mask_visib_tpath": join(
                split_path,
                "{scene_id:06d}",
                "mask_visib",
                "{im_id:06d}_{gt_id:06d}.png",
            ),
        }
    )

    return p


def get_present_scene_ids(dp_split):
    """Returns ID's of scenes present in the specified dataset split.

    :param dp_split: Path to a folder with datasets.
    :return: List with scene ID's.
    """
    scene_dirs = [d for d in glob.glob(os.path.join(dp_split["split_path"], "*")) if os.path.isdir(d)]
    scene_ids = [int(os.path.basename(scene_dir)) for scene_dir in scene_dirs]
    scene_ids = sorted(scene_ids)
    return scene_ids

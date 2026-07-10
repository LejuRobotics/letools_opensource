#include "PoseTransformer.h"
#include <Eigen/Geometry>

namespace autoHeadChase {

// Converts a pose from a local frame to the world frame
Eigen::VectorXd PoseTransformer::transformPoseToWorld(const Eigen::VectorXd& pose_in_local, const Eigen::VectorXd& frame_pose_in_world) {
    // Convert poses to transformation matrices
    Eigen::Matrix4d matrix_local = poseToTransform(pose_in_local);
    Eigen::Matrix4d matrix_frame_in_world = poseToTransform(frame_pose_in_world);

    // Transform pose to the world frame
    Eigen::Matrix4d matrix_pose_in_world = matrix_frame_in_world * matrix_local;

    // Convert back to pose format
    return transformToPose(matrix_pose_in_world);
}

// Converts a pose from the world frame to a local frame
Eigen::VectorXd PoseTransformer::transformPoseToLocal(const Eigen::VectorXd& pose_in_world, const Eigen::VectorXd& frame_pose_in_world) {
    // Convert poses to transformation matrices
    Eigen::Matrix4d matrix_world = poseToTransform(pose_in_world);
    Eigen::Matrix4d matrix_frame_in_world = poseToTransform(frame_pose_in_world);

    // Compute the inverse of the frame pose in the world frame
    Eigen::Matrix4d matrix_world_to_frame = matrix_frame_in_world.inverse();

    // Transform the world pose to the local frame
    Eigen::Matrix4d matrix_pose_in_local = matrix_world_to_frame * matrix_world;

    // Convert back to pose format
    return transformToPose(matrix_pose_in_local);
}

// Helper function to convert pose to a 4x4 transformation matrix
Eigen::Matrix4d PoseTransformer::poseToTransform(const Eigen::VectorXd& pose) {
    Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();

    // Extract position
    transform_matrix(0, 3) = pose(0);
    transform_matrix(1, 3) = pose(1);
    transform_matrix(2, 3) = pose(2);

    // Extract quaternion and convert to rotation matrix
    Eigen::Quaterniond quat(pose(6), pose(3), pose(4), pose(5));  // w, x, y, z
    Eigen::Matrix3d rotation_matrix = quat.normalized().toRotationMatrix();

    // Fill in the rotation part of the transform matrix
    transform_matrix.block<3, 3>(0, 0) = rotation_matrix;

    return transform_matrix;
}

// Helper function to convert transformation matrix back to pose
Eigen::VectorXd PoseTransformer::transformToPose(const Eigen::Matrix4d& transform_matrix) {
    Eigen::VectorXd pose(7);

    // Extract position
    pose(0) = transform_matrix(0, 3);
    pose(1) = transform_matrix(1, 3);
    pose(2) = transform_matrix(2, 3);

    // Extract rotation matrix and convert to quaternion
    Eigen::Matrix3d rotation_matrix = transform_matrix.block<3, 3>(0, 0);
    Eigen::Quaterniond quat(rotation_matrix);

    // Store quaternion
    pose(3) = quat.x();
    pose(4) = quat.y();
    pose(5) = quat.z();
    pose(6) = quat.w();

    return pose;
}

}  // namespace autoHeadChase

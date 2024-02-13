import open3d as o3d
import numpy as np
import copy
import time

# source: https://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.2,
                                      front=[0.0, -1.0, 0.0],
                                      lookat=[0.4, 0.0, 0.0],
                                      up=[0.0, 0.0, 1.0])
    
def preprocess_point_cloud(pcd, voxel_size=0.01):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = 0.1
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = 0.2
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(source, target, voxel_size=0.01, visualize=False):
    print(":: Load two point clouds and disturb initial pose.")

    if visualize:
        draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source_down, target_down, source_fpfh, target_fpfh

# voxel_size = 0.05  # means 5cm for this dataset
# source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
    # voxel_size)

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size=0.01):
    distance_threshold = 0.005
    angle_threshold = 0.03
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    # result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
    #     source_down, target_down, source_fpfh, target_fpfh, True,
    #     distance_threshold,
    #     o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
    #     3, [
    #         o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
    #         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
    #         #,o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(angle_threshold)
    #     ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

# result_ransac = execute_global_registration(source_down, target_down,
#                                             source_fpfh, target_fpfh,
#                                             voxel_size)
# print(result_ransac)
# draw_registration_result(source_down, target_down, result_ransac.transformation)

def refine_registration(source, target, source_fpfh, target_fpfh, transf_guess, voxel_size=0.01):
    distance_threshold = 0.001
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)

    # source.estimate_normals(
    #     o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    # target.estimate_normals(
    #     o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))

    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, transf_guess,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000))
    return result

# result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
#                                  voxel_size)
# print(result_icp)
# draw_registration_result(source, target, result_icp.transformation)


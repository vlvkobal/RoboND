#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

import gazebo_msgs.srv
from std_srvs.srv import Empty
import time

task_state = 'start'
collision_base = pcl.PointCloud_PointXYZRGB()

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(7)
    outlier_filter.set_std_dev_mul_thresh(-0.6)
    cloud_filtered = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.605
    axis_max = 1.0
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # TODO: Rotate PR2 in place to capture side tables for the collision map
    joint_angle = Float64()
    global task_state
    global collision_base
    collision_map_pub.publish(pcl_to_ros(collision_base))

    if task_state == 'start':
        joint_angle.data = np.pi/2
        joint_command_pub.publish(joint_angle)
        current_joint_angle = get_joint_properties('world_joint').position[0]
        if abs(current_joint_angle - np.pi/2) < 0.0001:
            task_state = 'left_sector'
            passthrough = cloud_filtered.make_passthrough_filter()
            filter_axis = 'y'
            passthrough.set_filter_field_name(filter_axis)
            axis_min = 0.43
            axis_max = 1.0
            passthrough.set_filter_limits(axis_min, axis_max)
            cloud_filtered = passthrough.filter()
            collision_base = cloud_filtered
            collision_map_pub.publish(pcl_to_ros(collision_base))
            return
        else:
            return
    elif task_state == 'left_sector':
        joint_angle.data = -np.pi/2
        joint_command_pub.publish(joint_angle)
        current_joint_angle = get_joint_properties('world_joint').position[0]
        if abs(current_joint_angle - -np.pi/2) < 0.0001:
            task_state = 'right_sector'
            passthrough = cloud_filtered.make_passthrough_filter()
            filter_axis = 'y'
            passthrough.set_filter_field_name(filter_axis)
            axis_min = -1.0
            axis_max = -0.43
            passthrough.set_filter_limits(axis_min, axis_max)
            cloud_filtered = passthrough.filter()
            collision_base.from_list(collision_base.to_list() + cloud_filtered.to_list())
            collision_map_pub.publish(pcl_to_ros(collision_base))
            return
        else:
            return
    elif task_state == 'right_sector':
        joint_angle.data = 0
        joint_command_pub.publish(joint_angle)
        current_joint_angle = get_joint_properties('world_joint').position[0]
        if abs(current_joint_angle) < 0.0001:
            task_state = 'finish'
            return
        else:
            return
    else:
        pass

    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.45
    axis_max = 0.45
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.006
    seg.set_distance_threshold(max_distance)
    plane_inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    cloud_table = cloud_filtered.extract(plane_inliers, negative=False)
    collision_base.from_list(collision_base.to_list() + cloud_table.to_list())
    collision_map_pub.publish(pcl_to_ros(collision_base))
    cloud_objects = cloud_filtered.extract(plane_inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(1500)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels),
                                                   detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    # try:
    #     pr2_mover(detected_objects)
    # except rospy.ROSInterruptException:
    #     pass
    if detected_objects:
        pr2_mover(detected_objects)

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    request_parameters_list = []
    test_scene_num = Int32()
    test_scene_num.data = 3
    arm_name = String()
    object_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    processed_objects = []

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')

    # TODO: Loop through the pick list
    picked_objects = []
    for i in range(0, len(object_list_param)):

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        for j in range(0, len(object_list)):
            if ((j not in processed_objects)
                    and (object_list[j].label == object_list_param[i]['name'])):
                picked_objects.append(j)
                object_name.data = str(object_list[j].label)
                points_arr = ros_to_pcl(object_list[j].cloud).to_array()
                centroid = np.mean(points_arr, axis=0)[:3]
                pick_pose.position.x = np.asscalar(centroid[0])
                pick_pose.position.y = np.asscalar(centroid[1])
                pick_pose.position.z = np.asscalar(centroid[2])

                # TODO: Create 'place_pose' for the object
                dropbox_param = rospy.get_param('/dropbox')
                for k in range(0,len(dropbox_param)):
                    if dropbox_param[k]['group'] == object_list_param[i]['group']:
                        place_pose.position.x = float(dropbox_param[k]['position'][0])
                        place_pose.position.y = float(dropbox_param[k]['position'][1])
                        place_pose.position.z = float(dropbox_param[k]['position'][2])

                # TODO: Assign the arm to be used for pick_place
                        arm_name.data = dropbox_param[k]['name']

                # TODO: Create a list of dictionaries (made with make_yaml_dict()) for
                # later output to yaml format
                request_parameters_list.append(make_yaml_dict(test_scene_num,
                                                              arm_name, object_name,
                                                              pick_pose, place_pose))

                # Add objects to collision map
                collision_pcl = pcl.PointCloud_PointXYZRGB(collision_base)
                for n in range(0, len(object_list)):
                    if n not in picked_objects:
                        collision_pcl.from_list(collision_pcl.to_list()
                            + ros_to_pcl(object_list[n].cloud).to_list())
                collision_map_pub.publish(pcl_to_ros(collision_pcl))

                # Wait for 'pick_place_routine' service to come up
                rospy.wait_for_service('pick_place_routine')

                try:
                    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                    # TODO: Insert your message variables to be sent as a service request
                    resp = pick_place_routine(test_scene_num, object_name, arm_name,
                                              pick_pose, place_pose)

                    print ("Response: ",resp.success)

                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                clear_octomap()
                processed_objects.append(j)
                break

    # TODO: Output your request parameters into output yaml file
    if request_parameters_list:
        # yaml_filename = 'output_' + `test_scene_num.data` + '.yaml'
        yaml_filename = 'output.yaml'
        send_to_yaml(yaml_filename, request_parameters_list)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('capture_node')

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    joint_command_pub = rospy.Publisher("/pr2/world_joint_controller/command", Float64, queue_size=1)
    collision_map_pub = rospy.Publisher("/pr2/3d_map/points", PointCloud2, queue_size=1)

    # Services
    get_joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties',
                                                  gazebo_msgs.srv.GetJointProperties)
    clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)

    # TODO: Load Model From disk
    model = pickle.load(open('model_3.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

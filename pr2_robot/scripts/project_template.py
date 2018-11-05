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

# Function to search list of dictionaries and return a selected value in selected dictionary
def search_dictionaries(key1, value1, key2, list_of_dictionaries):
    selected_dic = [element for element in list_of_dictionaries if element[key1] == value1][0]
    selected_val = selected_dic.get(key2)
    return selected_val

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    # Create a VoxelGrid filter object for our input point cloud
    vox = pcl_data.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    # Note: this (1) is a poor choice of leaf size
    # Experiment and find the appropriate size!
    LEAF_SIZE = 0.005

    # Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()

    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.456
    axis_max = 0.456
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()

    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'x'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.4
    axis_max = 0.8
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance
    # for segmenting the table
    max_distance = 0.006
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_outliers) # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.025)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(5000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
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
        pcl_cluster = extracted_outliers.extract(pts_list)

        # TODO: convert the cluster from pcl to ROS using helper function
        sample_cloud = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        chists = compute_color_histograms(sample_cloud, using_hsv=True)
        normals = get_normals(sample_cloud)
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
        do.cloud = ros_cluster_cloud
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    yaml_only = True # no motion

    # TODO: Initialize variables
    test_scene_num = Int32()
    object_name    = String()
    object_group = String()
    pick_pose      = Pose()
    place_pose     = Pose()
    arm_name       = String()
    yaml_dict_list = []

    # Update test scene number based on the selected test.
    test_scene_num.data = 3

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param     = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables


    # TODO: Rotate PR2 in place to capture side tables for the collision map
    # # Rotate Right
    # pr2_base_mover_pub.publish(-1.57)
    # rospy.sleep(15.0)
    # # Rotate Left
    # pr2_base_mover_pub.publish(1.57)
    # rospy.sleep(30.0)
    # # Rotate Center
    # pr2_base_mover_pub.publish(0)

    # Calculate detected objects centroids.
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

    request_count = 0
    success_count = 0

    # TODO: Loop through the pick list
    for i in range(0, len(object_list_param)):
        request_count += 1

    	# Parse parameters into individual variables
        object_name.data = object_list_param[i]['name']
        object_group.data = object_list_param[i]['group']

       	# Loop through the pick list and look for the requested object
        for the_object in object_list:
    	    match_count = 0
    	    if the_object.label == object_name.data:
                match_count += 1

        		# Get the PointCloud for a given object and obtain it's centroid
                labels.append(the_object.label)
                points_arr = ros_to_pcl(the_object.cloud).to_array()
                centroid = np.mean(points_arr, axis=0)[:3]
                top = np.max(points_arr, axis=0)[:3]
                bot = np.min(points_arr, axis=0)[:3]
                height = np.asscalar(top[2])-np.asscalar(bot[2])
                centroid = [np.asscalar(centroid[0]),np.asscalar(centroid[1]),np.asscalar(centroid[2])]
                centroids.append(centroid)
                print "Found %s at: %f %f %f height=%f" % (object_name.data, centroid[0], centroid[1], centroid[2], height)

            	# Assign the arm to be used for pick_place
                if object_group.data == 'green':
                    arm_name.data = 'right'
                    place_pose.position.x = -0.1-float(success_count)*0.2	# move back a little bit for each object
                    place_pose.position.y = -0.71				# so as not to stack...
                    place_pose.position.z = 0.605
                else:
                    arm_name.data = 'left'
                    place_pose.position.x = -0.1-float(success_count)*0.2
                    place_pose.position.y = 0.71
                    place_pose.position.z = 0.605

                pick_pose.position.x = centroid[0]
                pick_pose.position.y = centroid[1]
                pick_pose.position.z = centroid[2]-height

                print "Scene %d, picking up object %s that I found, with my %s arm, and placing it in the %s bin." % (test_scene_num.data, object_name.data, arm_name.data, object_group.data)

                # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
                yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
                yaml_dict_list.append(yaml_dict)


        if yaml_only == False:

            # Wait for 'pick_place_routine' service to come up
            rospy.wait_for_service('pick_place_routine')

            try:
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                # Insert message variables to be sent as a service request
                resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

                print "Response to pick_place_routine service request: ", resp.success
                if resp.success == True:
                    success_count += 1

            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

	# Output your request parameters into output yaml file
	send_to_yaml('output_%d.yaml' % test_scene_num.data, yaml_dict_list)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    # TODO: here you need to create two publishers
    # Call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with
    # Message Types "Marker" and "DetectedObjectsArray" , respectively
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    pr2_base_mover_pub   = rospy.Publisher("/pr2/world_joint_controller/command", Float64, queue_size=10)

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

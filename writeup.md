[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

# Project: Perception Pick & Place

Writeup by Peng Xu

Date: Nov 2018

---

## 1. Overview

In this project, you must assimilate your work from previous exercises to successfully complete a tabletop pick and place operation using PR2. The PR2 has been outfitted with an RGB-D sensor much like the one you used in previous exercises. This sensor however is a bit noisy much like real sensors.

Given a cluttered tabletop scenario, you must implement a perception pipeline using your work from Exercises 1, 2 and 3 to identify target objects from a so-called “Pick-List” in a particular order, pick up those objects and place them in corresponding dropboxes.

## 2. Implementation

### Exercise 1 Pipeline: Filtering and RANSAC Plane fitting.

The steps are the following:

- Downsample the point cloud by applying a Voxel Grid Filter.
- Add a statistical outlier filter to remove noise from the data.
- Apply a Passthrough Filter to isolate the table and objects.
- Perform RANSAC plane fitting to identify the table.
- Use the Passthrough Filter to create new point clouds containing the table and objects separately.

### Exercise 2 Pipeline: clustering for segmentation.

- Add clustering for segmentation to the pipeline.
- Create a python ros node that subscribes to /sensor_stick/point_cloud topic.
- Create publishers and topics to publish the segmented table and tabletop objects as separate point clouds
- Apply Euclidean clustering on the table-top objects

### Exercise 3 Pipeline: Features extraction, SVM model training and object recognition.

- Extract color and normal histogram features and train an SVM linear classifier.

- Populate the compute_color_histograms() and compute_normal_histograms() functions within features.py in /sensor_stick/src/sensor_stick to generate correct histogram results.

### Pick and Place Setup

For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image!
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  

## 3. Future work


## 4. Reference

# sensor_fusion_baseline
Sensor Fusion 2023ì4 accepts BBoxArray from yolo_camera_detector, motion compensated point cloud and selects points within each bbox to find accurate depth and consequently publish the final ConeArray message.

The fusion.cpp contains the constructor as well as the callback function called detectCones.

At the moment the filtered lidar points are chosen based on the estimated distance from the camera bounding box size. More specifically, first the bounding box section is selected, which at the moment is shrinked in the horizontal direction and augmented in the vertical direction. Shrinking the BBox width is to avoid taking points which are projected on the ground, considering the cone shape. On the othe hand, augmenting the height of the BBoxes is to make the search robust to calibration mismatches in the vertical direction. These were observed in particular in cases where the car is accelerating.

After the points contained in this section are stored, they are projected one-by-one to the egomotion frame and their planar distance is calcuated. If this distance is close to the estimated distance coming from the BBox size, then the lidar point positions and distances are stored for the further processing. Otherwise, they are discarded. This is to consider only the points which are close to the actual cone, providing an easy way to filter the cone section in the pointcloud by utilizing the camera information.

The stored points from above are then passed through a median filter, which selectes the point with the median distance.

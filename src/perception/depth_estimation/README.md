 # depth_estimation
 Monocular Depth Estimation code for the 2023 perception pipeline

 # Flow
 - depth_estimation receives the bounding box information of each frame from the yolo_camera_detector node in the '/perception/yolo_camera_detector/forward_bbox' topic.
 - PnP algorithm is used to estimate the position and depth of the cone. It uses 7 image points and 7 fixed points in the cone frame.
 - A camera cone array is published in the topic '/perception/camera/cone_array'.
 - Bounding box information with cone depth is published in '/perception/depth_estimation/bbox_depth'.
 - In debug mode, the regression points and cone depth is displayed in each frame.

 # Algorithm
 - This package uses 3 different algorithms to estimate cone pose. They can be switched using config parameters:
    - mde_on = false: Bounding box height is used to estimate the position of the cone using the principle of similar triangles
    - mde_on = true: There is a number of pixels per bounding box threshold to decide which algorithm should be used:
        - distance_threshold is low (~10): A ResNet, deployed using ONNX Runtime API is used to detect the 7 regression points.
        - distance_threshold is high (~100000): The 7 points are interpolated from the bounding box coordinates.

 # Classes
 - DepthEstimation:
    - Contains the subscribers, publishers and all the necessary parameters.
    - Also contains the callback function 'BboxDepth()'. In this function, for each bounding box, a function is called to estimate cone pose. 'DistanceEstimate()' uses the Resnet algorithm whereas 'DepthFromBoxHeight()' is uses bounding box height.
- KeypointDetector:
    - Initializes an ONNX Runtime session to run the ResNet inference.
    - For each cone, the bounding box is cropped out of the image and sent to the PredictPoints() function.
    - The image is preprocessed to a 1x3x80x80 tensor and passed through the ResNet which outputs a 1x14 tensor.
    - These numbers are scaled and rearranged (in the order shown below) to form imagePoints.

# Good to know
- We have 2 different cone dimensions because the resnet detects points that are closer to the conical part of the cone whereas the bounding box interpolation results in points that are closer to the cone's square base.
- An ideal distance_threshold would be 250-300.
- Order of points for cones:

             1
            / \
           /   \
          2-----5
         /       \
        /         \
       3-----------6
      /             \
     /               \
    4-----------------7

# Scope of improvement:
- Better PnP algorithms can be used such as P3P or PnP-RANSAC.

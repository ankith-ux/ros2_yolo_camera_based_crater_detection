YOLO-based Crater Detection for Autonomous Rover Navigation in ROS 2

![Image](https://github.com/user-attachments/assets/81f0c7b1-6727-4c4f-9f68-4844e7534e08)

![Image](https://github.com/user-attachments/assets/2d7a3f63-2a14-41b2-aa6d-0d639a36eded)

![Image](https://github.com/user-attachments/assets/6943ba84-9cb0-41a3-afe2-98a97b40a9fe)


This ROS 2 project provides a real-time crater detection system for an unmanned ground vehicle (UGV) using a 2D camera and a custom-trained YOLO (You Only Look Once) object detection model. The node processes a live camera feed, identifies craters, and projects their locations into 3D space, publishing them as a PointCloud2 message for use in navigation stacks like Nav2.
This approach serves as a powerful alternative or supplement to traditional LiDAR-based methods, leveraging the strengths of deep learning to identify specific terrain features from visual data.

Key Features
●	Deep Learning-Based Detection: Utilizes a custom-trained YOLO model from the ultralytics library for high-accuracy crater detection.
●	Camera-to-3D Projection: Implements a simplified but effective method to estimate the 3D position of detected craters from 2D image coordinates.
●	Persistent Hazard Mapping: Caches detected crater locations and transforms them into the /map frame, allowing the rover to build a persistent map of known hazards as it explores.
●	ROS 2 Integration: Fully integrated as a ROS 2 node, subscribing to Image and CameraInfo topics and publishing a standard PointCloud2 message.
●	Optimized for Performance: Includes optimizations such as frame skipping, GPU/CPU selection for the model, and caching of TF transforms to ensure real-time performance.

How the Algorithm Works
The detector_node.py script executes a step-by-step perception pipeline for each incoming camera frame:
1.	Image Subscription: The node subscribes to /camera/image_raw and /camera/camera_info to get the live video feed and the camera's intrinsic parameters.
2.	YOLO Inference: To save computational resources, the node processes every Nth frame. The image is resized and fed into the pre-trained YOLO model, which returns a list of bounding boxes for any detected craters that meet a confidence threshold.
3.	2D to 3D Projection: This is the core of the position estimation. For each detected crater:
a. The bottom-center of the bounding box (u, v) is used as the target point.
b. A forward distance is estimated based on the point's vertical (v) position in the image. The logic assumes that objects lower in the image are closer to the rover.
c. This estimated distance is used to create a preliminary 3D point in the camera's coordinate frame.
d. The point is then transformed into the rover's base_link frame by applying a rotation (to account for the camera's downward pitch) and a translation (to account for the camera's mounting position on the rover).
4.	Transform to Map Frame: The calculated 3D point in the base_link frame is transformed into the global map frame using the tf2 library. This ensures that the crater's position is known in a fixed, world-based coordinate system.
5.	Cache and Publish: The transformed 3D point is added to a persistent list of known crater locations. This entire list is then converted into a PointCloud2 message and published on the /yolo_craters topic. This allows visualization tools like RViz and navigation systems to see a growing map of all hazards the rover has ever detected.

Dependencies
●	ROS 2 Humble Hawksbill
●	Python 3.10+
●	Python Libraries:
○	ultralytics (for YOLO)
○	torch (and torchvision, etc., for the YOLO backend)
○	opencv-python
○	cv_bridge
○	numpy
○	tf_transformations
○	sensor_msgs-py

Setup and Usage
1.	Clone the Repository: Clone this project into your ROS 2 workspace's src directory.
2.	Install Dependencies: Make sure you have a Python virtual environment activated and install the required libraries.
pip install ultralytics torch opencv-python
3.	Place the Model: Ensure your trained best.pt model file is located in a models/ directory within your package.
4.	Build the Workspace:
colcon build --packages-select your_package_name

5.	Run the System:
○	Terminal 1: Launch your rover simulation that publishes the /camera/image_raw and /camera/camera_info topics.
○	Terminal 2: Source your environment and run the detector node:
  ros2 run your_package_name detector_node
○	Terminal 3 (Optional): Open RViz and add a PointCloud2 display subscribed to the /yolo_craters topic to visualize the detected hazards.

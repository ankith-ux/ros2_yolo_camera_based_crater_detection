import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

if not hasattr(np, 'float'):
    np.float = float

from ament_index_python.packages import get_package_share_directory
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg
from tf2_ros import TransformListener, Buffer
import tf_transformations
import torch  # 🔧 OPTIMIZED

class CraterDetector(Node):
    def __init__(self):
        super().__init__('yolo_crater_detector')

        package_share = get_package_share_directory('yolo_crater_detector')
        model_path = os.path.join(package_share, 'models', 'best.pt')
        self.model = YOLO(model_path)

        self.model.to('cpu')  # 🔧 OPTIMIZED for Intel iGPU

        self.declare_parameter('conf_threshold', 0.6)
        self.conf_threshold = self.get_parameter('conf_threshold').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        self.br = CvBridge()
        self.camera_info = None

        self.pc_pub = self.create_publisher(PointCloud2, '/yolo_craters', 10)
        self.detected_crater_positions = []

        self.frame_count = 0  # 🔧 OPTIMIZED
        self.last_transform = None  # 🔧 OPTIMIZED
        self.last_tf_time = self.get_clock().now()

    def camera_info_callback(self, data: CameraInfo):
        self.camera_info = data

    def compute_forward_distance(self, v: int, image_height: int) -> float:
        max_dist = 2.0
        min_dist = 0.5
        ratio = (image_height - v) / image_height
        return min_dist + ratio * (max_dist - min_dist)

    def image_callback(self, data: Image):
        if self.camera_info is None:
            self.get_logger().warn("Camera info not yet received.")
            return

        self.frame_count += 1  # 🔧 OPTIMIZED
        if self.frame_count % 5 != 0:  # skip every 4 frames
            return

        try:
            frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # 🔧 OPTIMIZED: Resize input
        resized_frame = cv2.resize(frame, (640, 480))
        scale_x = frame.shape[1] / 640
        scale_y = frame.shape[0] / 480

        with torch.no_grad():  # 🔧 OPTIMIZED
            results = self.model(resized_frame, conf=self.conf_threshold)[0]

        crater_points_base = []
        height, width = resized_frame.shape[:2]

        for r in results.boxes:
            if r.conf.item() < self.conf_threshold:
                continue

            x1, y1, x2, y2 = map(int, r.xyxy[0])
            u, v = (x1 + x2) // 2, y2

            forward_distance = self.compute_forward_distance(v, height)
            x, y, z = self.project_to_3d(u, v, forward_distance, self.camera_info)

            crater_radius = 0.2
            points_per_crater = 6
            for i in range(points_per_crater):
                angle = 2 * np.pi * i / points_per_crater
                px = x + crater_radius * np.cos(angle)
                py = y + crater_radius * np.sin(angle)
                crater_points_base.append((px, py, z))

            # 🔧 Scale detection box back to original image
            x1_full, y1_full = int(x1 * scale_x), int(y1 * scale_y)
            x2_full, y2_full = int(x2 * scale_x), int(y2 * scale_y)
            class_id = int(r.cls.item())
            label = f"{self.model.names[class_id]} {r.conf.item():.2f}"
            cv2.rectangle(frame, (x1_full, y1_full), (x2_full, y2_full), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1_full, y1_full - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 🔧 OPTIMIZED: Cache TF lookup
        now = self.get_clock().now()
        if (now - self.last_tf_time).nanoseconds > 1e9:  # refresh every 1s
            try:
                self.last_transform = self.tf_buffer.lookup_transform(
                    target_frame='map',
                    source_frame='base_link',
                    time=rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                self.last_tf_time = now
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed: {e}")
                return

        transform = self.last_transform
        if not transform:
            return

        map_points = []
        for x, y, z in crater_points_base:
            point_base = np.array([x, y, z, 1.0])
            trans = transform.transform.translation
            rot = transform.transform.rotation
            t_mat = tf_transformations.translation_matrix([trans.x, trans.y, trans.z])
            r_mat = tf_transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
            full_tf = t_mat @ r_mat
            pt_map = full_tf @ point_base
            map_points.append((pt_map[0], pt_map[1], pt_map[2]))

        # Cache unique detections
        for pt in map_points:
            if not any(np.linalg.norm(np.array(pt) - np.array(existing)) < 0.3
                       for existing in self.detected_crater_positions):
                self.detected_crater_positions.append(pt)

        self.publish_crater_cloud(self.detected_crater_positions, frame_id='map')

        # Show detection image
        cv2.imshow('YOLO Crater Detection', frame)
        cv2.waitKey(1)

    def project_to_3d(self, u: int, v: int, dummy_dist: float, camera_info: CameraInfo):
        point_cam = np.array([dummy_dist, 0.0, 0.0])
        pitch = 0.3
        R_pitch = np.array([
            [np.cos(pitch), 0, -np.sin(pitch)],
            [0, 1, 0],
            [np.sin(pitch), 0,  np.cos(pitch)]
        ])
        point_base = R_pitch @ point_cam
        cam_offset = np.array([0.175, 0.0, 0.07])
        world_pt = cam_offset + point_base
        return float(world_pt[0]), float(world_pt[1]), float(world_pt[2])

    def publish_crater_cloud(self, points, frame_id='map'):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        if points:
            cloud_msg = pc2.create_cloud_xyz32(header, points)
            self.pc_pub.publish(cloud_msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CraterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

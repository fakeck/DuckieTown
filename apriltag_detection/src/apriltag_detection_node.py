#!/usr/bin/env python3

# This node publishes the closest detected AprilTag id, 
# the corresponding map grid coordinates,
# and the tag pose in the camera frame
import rospy
import numpy as np
from dt_apriltags import Detector
import yaml
import cv2
import os
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from apriltag_detection.msg import ApriltagMsg
from typing import Tuple, Dict

def rotation_mat_to_quat(rot_mat: np.ndarray) -> dict:
    if rot_mat.shape != (3, 3):
        raise ValueError("Rotation matrix must be 3x3.")
    
    rotation = R.from_matrix(rot_mat)
    quat = rotation.as_quat() # scalar last
    
    return {
        "x": quat[0],
        "y": quat[1],
        "z": quat[2],
        "w": quat[3]
    }


class AprilTagDetectionNode:
    node_name: str
    robot_name: str
    render_overlay: bool
    detection_rate: int
    tag_size: float
    map: np.ndarray
    tags: Dict[int, list]
    goal_tag_id: int
    intrinsic_dict: dict
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    projection_matrix: np.ndarray

    detector: Detector

    timer: rospy.Timer
    
    image_sub: rospy.Subscriber
    last_image: CompressedImage
    
    tag_pub: rospy.Publisher
    overlay_pub: rospy.Publisher

    def __init__(self) -> None:
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing {self.node_name} node...")

        self.robot_name = os.getenv('VEHICLE_NAME', None)
        if self.robot_name is None:
            raise ValueError("$VEHICLE_NAME is not set, export it first.")
        rospy.loginfo(f"[{self.node_name}] Robot name: {self.robot_name}")

        # Decide if we render the overlay image, default is True
        self.render_overlay = rospy.get_param("~render_overlay", True)
        rospy.loginfo(f"[{self.node_name}] Render overlay: {self.render_overlay}")
        
        # Lower detection rate to reduce latency
        self.detection_rate = 5 # Hz
        rospy.loginfo(f"[{self.node_name}] Detection rate: {self.detection_rate} Hz")

        # keep everything in meter [m]
        self.tag_size = rospy.get_param("~tag_size")
        if self.tag_size is None:
            raise ValueError(f"[{self.node_name}] ~tag_size is not set")
        rospy.loginfo(f"[{self.node_name}] Tag size: %s", self.tag_size)
        self.tag_size = float(self.tag_size)

        self.goal_tag_id = rospy.get_param(f"/{self.robot_name}/goal_tag_id", None)
        if self.goal_tag_id is None:
            rospy.logwarn(f"[{self.node_name}] /{self.robot_name}/goal_tag_id is not set")
            self.goal_tag_id = -1
        else:
            rospy.loginfo(f"[{self.node_name}] Goal tag id: %s", self.goal_tag_id)
            self.goal_tag_id = int(self.goal_tag_id)

        # Read map and intrinsics from yaml files
        self.map, self.tags = self._read_map() # np.ndarray and dict{tag_id, [coord_x, coord_y]}
        self.intrinsic_dict = self._read_intrinsic()
        self.camera_matrix = self.intrinsic_dict['camera_matrix']
        self.distortion_coefficients = self.intrinsic_dict['distortion_coefficients']
        self.projection_matrix = self.intrinsic_dict['projection_matrix']

        self.detector = Detector(searchpath=["apriltags"],
                                    families="tag36h11",
                                    nthreads=1,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        
        self.last_image = None
        self.image_sub = rospy.Subscriber(f"/{self.robot_name}/camera_node/image/compressed", CompressedImage, self.__store_latest_image_cb)
        
        self.timer = rospy.Timer(rospy.Duration(1/self.detection_rate), self._process_latest_image)
        self.tag_pub = rospy.Publisher(f"/{self.robot_name}/apriltag_detection_node/tag_info", ApriltagMsg, queue_size=1)
        # Publish the overlay image, **don't change endfix /compressed**
        if self.render_overlay:
            self.overlay_pub = rospy.Publisher(f"/{self.robot_name}/apriltag_detection_node/overlay/compressed", CompressedImage, queue_size=1)

    def _read_map(self) -> Tuple[np.ndarray, Dict[int, list]]:
        yaml_path = rospy.get_param("~map_yaml_path")
        if yaml_path is None:
            raise ValueError("~map_yaml_path is not set")
        rospy.loginfo(f"[{self.node_name}] Map YAML path: %s", yaml_path)
        
        with open(yaml_path, "r") as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
            map = np.array(data["map"])
            tags = data["tags"]
        return map, tags

    def _read_intrinsic(self) -> dict:
        yaml_path = rospy.get_param("~camera_intrinsics_yaml_path")
        if yaml_path is None:
            raise ValueError(f"[{self.node_name}] ~camera_intrinsics_yaml_path is not set")
        rospy.loginfo(f"[{self.node_name}] Intrinsic YAML path: %s", yaml_path)
        
        with open(yaml_path, "r") as file:
            data = yaml.safe_load(file)

        # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_matrix = np.array(data['camera_matrix']['data']).reshape(
            data['camera_matrix']['rows'], data['camera_matrix']['cols']
        )
        # [k1, k2, p1, p2, k3]
        distortion_coefficients = np.array(data['distortion_coefficients']['data']).reshape(
            data['distortion_coefficients']['rows'], data['distortion_coefficients']['cols']
        )
        # K * [R|t]
        projection_matrix = np.array(data['projection_matrix']['data']).reshape(
            data['projection_matrix']['rows'], data['projection_matrix']['cols']
        )
        return {
            'camera_matrix': camera_matrix,
            'distortion_coefficients': distortion_coefficients,
            'projection_matrix': projection_matrix
        }
    
    def __store_latest_image_cb(self, image_msg: CompressedImage) -> None:
        self.last_image = image_msg

    def _process_latest_image(self, event) -> None:
        if self.last_image is None:
            return # No image to process yet
        
        image_msg = self.last_image
        # the img was published at 30hz, but this node only processes at 5hz
        image = cv2.imdecode(np.frombuffer(image_msg.data, np.uint8), cv2.IMREAD_COLOR)
        # undistort the image
        undistorted_image = cv2.undistort(image, self.camera_matrix, self.distortion_coefficients)
        # convert to grayscale
        gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        tags = self.detector.detect(gray, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=self.tag_size)
        
        detected_ids = [tag.tag_id for tag in tags]
        valid_ids = list(self.tags.keys()) + [self.goal_tag_id]
        checked_ids = []
        # rospy.loginfo(f"[{self.node_name}] Detected tags: %s", detected_ids)

        # Find the closest tag to the camera
        closest_tag = None
        min_distance = float('inf')

        for tag in tags:
            if tag.tag_id in checked_ids or tag.tag_id not in valid_ids:
                continue # Skip this tag if already checked or not valid on the map
            checked_ids.append(tag.tag_id)
            # Calculate the Euclidean distance of the tag from the camera origin
            distance = np.linalg.norm(tag.pose_t)
            if distance < min_distance:
                min_distance = distance
                closest_tag = tag

        if closest_tag is not None:
            # Publish the detected tag info
            tag_msg = ApriltagMsg()
            tag_msg.tag_id = closest_tag.tag_id
            tag_msg.tag_pose = PoseStamped()
            tag_msg.tag_pose.header.stamp = rospy.Time.now()
            tag_msg.tag_pose.header.frame_id = f"/{self.robot_name}/camera_frame"
            tag_msg.tag_pose.pose.position.x = closest_tag.pose_t[0]
            tag_msg.tag_pose.pose.position.y = closest_tag.pose_t[1]
            tag_msg.tag_pose.pose.position.z = closest_tag.pose_t[2]

            quat = rotation_mat_to_quat(closest_tag.pose_R)
            tag_msg.tag_pose.pose.orientation.x = quat['x']
            tag_msg.tag_pose.pose.orientation.y = quat['y']
            tag_msg.tag_pose.pose.orientation.z = quat['z']
            tag_msg.tag_pose.pose.orientation.w = quat['w']

            if closest_tag.tag_id == self.goal_tag_id:
                tag_msg.grid_coords = [-1, -1]
            else:
                tag_msg.grid_coords = self.tags[closest_tag.tag_id]
            self.tag_pub.publish(tag_msg)

            # Overlay the detected tag on the image
            if not self.render_overlay:
                return
            overlay_image = undistorted_image.copy()
            for tag in tags:
                # Ensure corners are in integer coordinates and draw lines between them
                for idx in range(len(tag.corners)):
                    start_point = tuple(tag.corners[idx - 1, :].astype(int))
                    end_point = tuple(tag.corners[idx, :].astype(int))
                    cv2.line(overlay_image, start_point, end_point, (0, 255, 0), 2)

                # Put the tag id on the image
                cv2.putText(overlay_image, str(tag.tag_id), 
                            (int(tag.corners[0, 0]), int(tag.corners[0, 1])), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            _, jpeg_image = cv2.imencode('.jpeg', overlay_image)

            compressed_image_msg = CompressedImage()
            compressed_image_msg.header = image_msg.header
            compressed_image_msg.header.stamp = rospy.Time.now()
            compressed_image_msg.format = "jpeg"
            compressed_image_msg.data = jpeg_image.tobytes()
            self.overlay_pub.publish(compressed_image_msg)
        else:
            # No tag detected
            tag_msg = ApriltagMsg()
            tag_msg.tag_id = -1 # No tag detected
            tag_msg.tag_pose = PoseStamped()
            tag_msg.tag_pose.header.stamp = rospy.Time.now()
            tag_msg.tag_pose.header.frame_id = "camera_frame"
            tag_msg.tag_pose.pose.position.x = 0
            tag_msg.tag_pose.pose.position.y = 0
            tag_msg.tag_pose.pose.position.z = 0
            tag_msg.tag_pose.pose.orientation.x = 0
            tag_msg.tag_pose.pose.orientation.y = 0
            tag_msg.tag_pose.pose.orientation.z = 0
            tag_msg.tag_pose.pose.orientation.w = 1
            tag_msg.grid_coords = [0, 0]
            self.tag_pub.publish(tag_msg)

            if not self.render_overlay:
                return
            # Publish the undistorted image
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header = image_msg.header
            compressed_image_msg.header.stamp = rospy.Time.now()
            compressed_image_msg.format = "jpeg"
            _, jpeg_image = cv2.imencode('.jpeg', undistorted_image)
            compressed_image_msg.data = jpeg_image.tobytes()
            self.overlay_pub.publish(compressed_image_msg)
                


if __name__ == "__main__":
    rospy.init_node("apriltag_detection_node")
    node = AprilTagDetectionNode()
    rospy.spin()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from yolov8_msgs.msg import Yolov8InferencePosition, InferenceResultPosition
from object_msgs.msg import FrontDistance, FrontDetection
from transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, PoseStamped
from unitree_go.msg import WebRtcReq
from unitree_api.msg import Request
import tf2_ros
import math
import json
import numpy as np
import time
import random


class VUI_COLOR:
    WHITE: str = 'white'
    RED: str = 'red'
    YELLOW: str = 'yellow'
    BLUE: str = 'blue'
    GREEN: str = 'green'
    CYAN: str = 'cyan'
    PURPLE: str = 'purple'

class ObjectAligner(Node):
    def __init__(self):
        super().__init__('object_aligner')

        self.Led_Color = VUI_COLOR()
        self.led_color_time = 1

        self.image_width = 1280
        self.image_height = 720

        self.center_exit = 125
        self.center_enter = 75
        self.lateral_exit = 100
        self.lateral_enter = 60

        self.too_far_exit = 1.0
        self.too_far_enter = 1.2
        self.too_close_exit = 2.5
        self.too_close_enter = 1.8

        self.error_speed = 0.0005
        self.retreat_speed = -0.12
        self.advance_speed = 0.12



        self.angle_threshold = 0.07  # radianes
        self.close_threshold = 0.37
        self.far_threshold = 0.6
        self.too_close_threshold = 0.34
        self.too_far_lateral_threshold = 0.12
        self.lateral_distance_position = -0.037
        self.lateral_distance_threshold = 0.017

        self.far_lateral_speed = 0.16
        self.rotation_error_speed = 0.6
        self.close_advance_error_speed = 0.4
        self.lateral_error_speed = 0.4

        self.cooldown = 1.75 # segundos

        self.last_lateral_move_time = self.get_clock().now()
        self.last_forward_move_time = self.get_clock().now()

        self.last_move_time = self.get_clock().now()
        self.last_move_type = "none"
        self.switch_cooldown = 1.75

        self.alignment_completed = False
        self.approach_completed = False
        self.completed_stable_time = 2.5 
        self.alignment_stable_since = None

        self.current_phase = 0  # 1: alineación, 2: acercamiento, 3: pateo

        self.time_since_lost_started = None
        self.timeout_to_phase_0 = 3.0

        self.min_speed = 0.12
        self.max_speed = 0.3

        self.target_kick_name = "kick_target"
        self.target_goal_name = "goal_target"

        self.kick_bbox = None
        self.goal_bbox = None

        self.kick_centered = False
        self.goal_centered = False
        self.advance_active = False
        self.retreat_active = False
        self.goal_aligned_lateral = False

        self.kick_completed = False

        self.front_object_position = None
        self.goal_marker_position = None

        self.robot_pose = None
        self.front_distances = None

        self.kick_offset_ok = True
        self.distance_ok = True
        self.lateral_ok = True
        self.goal_rotation_ok = True

        self.behavior_start_time = self.get_clock().now()
        self.phase_start_time = None
        self.phase_durations = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}  # segundos


        self.inference_sub = self.create_subscription(
            Yolov8InferencePosition,
            '/Yolov8_Inference_Position',
            self.inference_callback,
            10
        )

        self.front_object_detection_sub = self.create_subscription(
            FrontDetection,
            '/front_object_detection',
            self.front_detection_callback,
            10
        )

        self.pose_robot = self.create_subscription(
            PoseStamped,
            '/utlidar/robot_pose',
            self.pose_robot_callback,
            10
        )

        self.publisher_webrtc = self.create_publisher(WebRtcReq, '/webrtc_req', 10)
        self.publisher_sport_api = self.create_publisher(Request, '/api/sport/request', 10)
        self.publisher_vui_api = self.create_publisher(Request, '/api/vui/request', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        self.timer = self.create_timer(0.1, self.control_loop)


    def execute_kick(self):
        
        self.send_api_kick()
        self.get_logger().info(f"Pateo completado: distancia frontal actual {self.front_distances.distance_x}.")
        self.get_logger().info("Fase actual: Baixo nivel")
        self.kick_completed = True

        # Marca el tiempo final del comportamiento
        now = self.get_clock().now()
        
        # Sumar duración en la fase actual hasta este momento
        if self.phase_start_time is not None and self.current_phase is not None:
            phase_elapsed = (now - self.phase_start_time).nanoseconds / 1e9
            self.phase_durations[self.current_phase] += phase_elapsed
            self.get_logger().info(f"⏱ Tiempo en fase {self.current_phase} antes del pateo: {phase_elapsed:.2f}s")

        # Calcular duración total del comportamiento
        if self.behavior_start_time is not None:
            total_elapsed = (now - self.behavior_start_time).nanoseconds / 1e9
        else:
            total_elapsed = 0.0
        
        # Imprimir duración total
        self.get_logger().info(f"⏳ Tiempo total del comportamiento: {total_elapsed:.2f}s")
        
        # Imprimir tiempos por fase
        for phase, duration in self.phase_durations.items():
            self.get_logger().info(f"⏳ Tiempo total en fase {phase}: {duration:.2f}s")



    def send_api_kick(self):
        request = Request()
        request.header.identity.id = int(time.time() * 1e6) + random.randint(0, 1000)
        request.header.identity.api_id = 1016  # SPORT_API_ID_KICK
        request.header.lease.id = 0
        request.header.policy.priority = 0
        request.header.policy.noreply = True
        request.parameter = ""

        self.publisher_sport_api.publish(request)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

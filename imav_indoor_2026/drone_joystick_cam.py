import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy, Image
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition, VehicleAttitude
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import math
import time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class DroneController(Node):
    
    def __init__(self):
        super().__init__('drone_joystick_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # --- Publishers ---
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # --- Subscribers ---
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile=qos_profile)
        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_position_callback, qos_profile=qos_profile)
        self.local_orienation_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.local_orientation_callback, qos_profile=qos_profile)
        self.joystick_sub = self.create_subscription(Joy, '/joy',self.joystick_callback, 10)

        # --- State Variables ---
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.local_position = VehicleLocalPosition()
        self.local_orientation = VehicleAttitude()
        self.islanding = False

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_child_frame = 'drone_base_link'
        
        # --- Timers ---
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # --- Image ---
        self.image_sub = self.create_subscription(Image, '/world/imav2026_scaled/model/x500_depth_0/link/camera_link/sensor/IMX214/image', self.image_callback, 10)
        self.cv_bridge = CvBridge()
        self.cam_pitch = 0.0
        self.CAM_SENSITIVITY = 1.5

        # --- ADD YOLO MODEL ---
        # self.MODEL_PATH = '/home/parthbhardwaj/ros2_humble_ws/src/drone_controller/YOLO-weights/best.pt'
        # self.model = YOLO(self.MODEL_PATH)
        # self.get_logger().info(f'Loaded YOLO model from {self.MODEL_PATH}')
        # -------------------------

        # --- Velocity Variables ---
        self.MAX_VELOCITY = 3.0 
        self.BOOST = 6.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yawspeed = 0.0
        self.MAX_YAWSPEED = 1.5

        # --- Controller log ---
        self.LB = 6
        self.RB = 7
        self.LT = 5
        self.RT = 4
        self.LEFT_STICK_H = 0
        self.LEFT_STICK_V = 1
        self.RIGHT_STICK_H = 2
        self.RIGHT_STICK_V = 3
        self.Y_BUTTON = 4
        self.X_BUTTON = 3
        self.A_BUTTON = 0
        self.B_BUTTON = 1
        self.DPAD_UP_DOWN = 7
        self.axes = [0,0,0,0,0,0,0,0]
        self.buttons = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
    
    def local_position_callback(self, msg):
        self.local_position = msg
        # -- Testing ---
        try:
            self.publish_tf_transform()
        except Exception as e:
            self.get_logger().error(f"Failed to publish transform: {e}")
    
    def local_orientation_callback(self, msg):
        self.local_orientation = msg
    
    def publish_tf_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.tf_child_frame

        t.transform.translation.x = self.local_position.x
        t.transform.translation.y = self.local_position.y
        t.transform.translation.z = -self.local_position.z

        [qx, qy, qz, qw] = [0.0, 0.0, 0.0, 1.0]
        if self.local_orientation is not None:
            [qx, qy, qz, qw] = [self.local_orientation.q[0], self.local_orientation.q[1],
                                self.local_orientation.q[2], self.local_orientation.q[3]]
            

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = -float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)
    
    def calculate_velocity(self, axes_vals):
        vx = axes_vals[self.LEFT_STICK_V] * (self.MAX_VELOCITY if axes_vals[self.DPAD_UP_DOWN] == 0 else self.BOOST)
        vy = axes_vals[self.LEFT_STICK_H] * (self.MAX_VELOCITY if axes_vals[self.DPAD_UP_DOWN] == 0 else self.BOOST)
        vz = 0.0

        if axes_vals[self.LT] < 0.0 and axes_vals[self.RT] > 0.0:
            vz = -axes_vals[self.LT] * self.MAX_VELOCITY

        elif axes_vals[self.LT] > 0.0 and axes_vals[self.RT] < 0.0:
            vz = axes_vals[self.RT] * self.MAX_VELOCITY
        
        angle = self.local_position.heading
        global_vx = vx * math.cos(angle) + vy * math.sin(angle)
        global_vy = vx * math.sin(angle) - vy * math.cos(angle)

        return global_vx, global_vy, vz

    def joystick_callback(self, msg):
        self.axes = msg.axes
        self.buttons = msg.buttons  
        
    
    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = cv2.resize(cv_image, (640, 480))
            # results = self.model(cv_image, verbose=False)
            # annotated_frame = results[0].plot()
            cv2.imshow("X500 depth Camera Feed", cv_image) 
            # cv2.imshow("Original Feed", annotated_frame)
            cv2.waitKey(1) 
            
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def timer_callback(self):

        if self.islanding:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.islanding = False
            return
        
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info('Drone not armed... Press Y button to arm.')
            if self.buttons[self.Y_BUTTON] == 1:
                self.get_logger().info('Arming the drone...')
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            return

        self.publish_offboard_control_mode()

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info('Switching to OFFBOARD mode')
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) 
            return


        self.vx, self.vy, self.vz = self.calculate_velocity(self.axes)
        self.yawspeed = -self.MAX_YAWSPEED * self.axes[self.RIGHT_STICK_H]

        self.cam_pitch += self.axes[self.RIGHT_STICK_V] * self.CAM_SENSITIVITY
        self.cam_pitch = max(-90.0, min(0.0, self.cam_pitch))

        self.publish_trajectory_setpoint(self.vx, self.vy, self.vz, self.yawspeed)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL, param1=self.cam_pitch, 
                                     param2=0.0, param3=0.0, param7=2.0)
        
        if time.time() % 5 < 0.1:
            pass  
            self.get_logger().info(f'Position: x={self.local_position.x:.2f}, y={self.local_position.y:.2f}, z={self.local_position.z:.2f}')
            self.get_logger().info(f'CAM Pitch = {self.cam_pitch:.2f}°')

        if self.buttons[self.X_BUTTON] == 1:
            # self.get_logger().info('Landing Initiated...')
            self.islanding = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False 
        msg.velocity = True  
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, vx, vy, vz, yawspeed):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        nan_val = float('nan')
        
        # Position set to NaN
        msg.position[0] = nan_val
        msg.position[1] = nan_val
        msg.position[2] = nan_val
        
        # Velocity set
        msg.velocity[0] = vx
        msg.velocity[1] = vy
        msg.velocity[2] = vz
        
        # Yaw set
        msg.yaw = nan_val
        msg.yawspeed = yawspeed

        # Acceleration set to NaN
        msg.acceleration[0] = nan_val
        msg.acceleration[1] = nan_val
        msg.acceleration[2] = nan_val
        
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=3.0, param7=0.0):

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    try:
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        pass
    finally:
        drone_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
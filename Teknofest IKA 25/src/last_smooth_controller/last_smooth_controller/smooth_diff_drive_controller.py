#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
from threading import Lock

from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from .pid_controller import PIDController
from .velocity_filter import VelocityFilter
from .motion_planner import MotionPlanner


class SmoothDiffDriveController(Node):
    def __init__(self):
        super().__init__('smooth_diff_drive_controller')
        
        # Node parametreleri
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_separation', 0.80),
                ('wheel_radius', 0.185),
                ('wheels_per_side', 3),
                ('max_linear_velocity', 8.0),
                ('max_angular_velocity', 6.0),
                ('max_linear_acceleration', 10.0),
                ('max_angular_acceleration', 12.0),
                ('control_frequency', 100.0),
                ('cmd_vel_timeout', 0.5),
                ('enable_smooth_control', True),
                ('enable_velocity_filtering', True),
                ('enable_motion_planning', True),
                ('wheel_slip_threshold', 0.1),
                ('odom_frame_id', 'odom'),
                ('base_frame_id', 'base_link'),
            ]
        )
        
        # Parametreleri al
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheels_per_side = self.get_parameter('wheels_per_side').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.max_linear_acc = self.get_parameter('max_linear_acceleration').value
        self.max_angular_acc = self.get_parameter('max_angular_acceleration').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.enable_smooth = self.get_parameter('enable_smooth_control').value
        self.enable_filtering = self.get_parameter('enable_velocity_filtering').value
        self.enable_planning = self.get_parameter('enable_motion_planning').value
        self.wheel_slip_threshold = self.get_parameter('wheel_slip_threshold').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        
        # QoS profili
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, qos_profile)
            
        # Publishers
        self.wheel_cmd_pub = self.create_publisher(
            Twist, '/smooth_cmd_vel', qos_profile)
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', qos_profile)
        self.filtered_cmd_pub = self.create_publisher(
            Twist, '/cmd_vel_filtered', qos_profile)
        
        # TF Broadcaster - RViz odometry sorunu için
        self.tf_broadcaster = TransformBroadcaster(self)
            
        # Internal state
        self.lock = Lock()
        self.last_cmd_time = self.get_clock().now()
        self.current_cmd_vel = Twist()
        self.target_cmd_vel = Twist()
        self.filtered_cmd_vel = Twist()
        
        # Robot state
        self.robot_position = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.robot_velocity = [0.0, 0.0]  # [linear, angular]
        self.wheel_positions = [0.0] * 6  # 6 tekerlek pozisyonu
        self.wheel_velocities = [0.0] * 6  # 6 tekerlek hızı
        self.last_wheel_positions = [0.0] * 6
        
        # Control components
        self.pid_controllers = []
        for i in range(6):
            pid = PIDController(
                kp=0.3, ki=0.05, kd=0.01,  # Daha yumuşak PID parametreleri
                max_output=self.max_linear_vel * 0.8,  # Biraz daha düşük limit
                min_output=-self.max_linear_vel * 0.8
            )
            self.pid_controllers.append(pid)
            
        self.velocity_filter = VelocityFilter(
            cutoff_freq=8.0,  # Daha düşük kesim frekansı
            sample_rate=self.control_freq
        ) if self.enable_filtering else None
        
        self.motion_planner = MotionPlanner(
            max_linear_vel=self.max_linear_vel,
            max_angular_vel=self.max_angular_vel,
            max_linear_acc=self.max_linear_acc,
            max_angular_acc=self.max_angular_acc
        ) if self.enable_planning else None
        
        # Control timer
        timer_period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        # Odometry timer - Daha düşük frekans
        self.odom_timer = self.create_timer(0.04, self.publish_odometry)  # 25Hz
        
        self.get_logger().info('Smooth Diff Drive Controller initialized')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation}m')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'Control frequency: {self.control_freq}Hz')
        
    def cmd_vel_callback(self, msg):
        """CMD_VEL mesajlarını al"""
        self.get_logger().info(f'RECEIVED cmd_vel - Linear: {msg.linear.x:.3f}, Angular: {msg.angular.z:.3f}')
        
        with self.lock:
            self.target_cmd_vel = msg
            self.last_cmd_time = self.get_clock().now()
            
        self.get_logger().info(f'STORED target_cmd_vel - Linear: {self.target_cmd_vel.linear.x:.3f}, Angular: {self.target_cmd_vel.angular.z:.3f}')
            
    def joint_state_callback(self, msg):
        """Tekerlek durumlarını al"""
        if len(msg.name) >= 6:
            # Tekerlek pozisyonlarını güncelle
            wheel_names = [
                'front_left_wheel_joint', 'mid_left_wheel_joint', 'back_left_wheel_joint',
                'front_right_wheel_joint', 'mid_right_wheel_joint', 'back_right_wheel_joint'
            ]
            
            for i, name in enumerate(wheel_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.wheel_positions[i] = msg.position[idx]
                    if len(msg.velocity) > idx:
                        self.wheel_velocities[i] = msg.velocity[idx]
                        
    def control_loop(self):
        """Ana kontrol döngüsü"""
        current_time = self.get_clock().now()
        
        # CMD_VEL timeout kontrolü
        time_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.cmd_vel_timeout:
            with self.lock:
                self.target_cmd_vel = Twist()  # Stop the robot
                
        with self.lock:
            target_vel = self.target_cmd_vel
            
        # Debug target velocity
        if abs(target_vel.linear.x) > 0.01 or abs(target_vel.angular.z) > 0.01:
            self.get_logger().info(f'Target cmd_vel - Linear: {target_vel.linear.x:.3f}, Angular: {target_vel.angular.z:.3f}')
            
        # Motion planning uygula
        if self.enable_planning and self.motion_planner:
            planned_vel = self.motion_planner.plan_velocity(
                current_vel=self.current_cmd_vel,
                target_vel=target_vel,
                dt=1.0/self.control_freq
            )
        else:
            planned_vel = target_vel
            
        # Debug planned velocity
        if abs(planned_vel.linear.x) > 0.01 or abs(planned_vel.angular.z) > 0.01:
            self.get_logger().info(f'Planned cmd_vel - Linear: {planned_vel.linear.x:.3f}, Angular: {planned_vel.angular.z:.3f}')
            
        # Velocity filtering uygula
        if self.enable_filtering and self.velocity_filter:
            self.filtered_cmd_vel = self.velocity_filter.filter_velocity(planned_vel)
        else:
            self.filtered_cmd_vel = planned_vel
            
        # Debug filtered velocity
        if abs(self.filtered_cmd_vel.linear.x) > 0.01 or abs(self.filtered_cmd_vel.angular.z) > 0.01:
            self.get_logger().info(f'Filtered cmd_vel - Linear: {self.filtered_cmd_vel.linear.x:.3f}, Angular: {self.filtered_cmd_vel.angular.z:.3f}')
            
        # Smooth control uygula
        if self.enable_smooth:
            prev_vel = self.current_cmd_vel
            self.current_cmd_vel = self.smooth_velocity_transition(
                self.current_cmd_vel, self.filtered_cmd_vel, 1.0/self.control_freq)
            # Debug smooth transition
            if abs(self.current_cmd_vel.linear.x) > 0.01 or abs(self.current_cmd_vel.angular.z) > 0.01:
                self.get_logger().info(f'Smooth transition: {prev_vel.linear.x:.3f} -> {self.current_cmd_vel.linear.x:.3f} (target: {self.filtered_cmd_vel.linear.x:.3f})')
        else:
            self.current_cmd_vel = self.filtered_cmd_vel
            
        # Tekerlek hızlarını hesapla
        wheel_speeds = self.calculate_wheel_speeds(self.current_cmd_vel)
        
        # Debug logging for input
        if abs(self.current_cmd_vel.linear.x) > 0.01 or abs(self.current_cmd_vel.angular.z) > 0.01:
            self.get_logger().info(f'Input cmd_vel - Linear: {self.current_cmd_vel.linear.x:.3f}, Angular: {self.current_cmd_vel.angular.z:.3f}')
            self.get_logger().info(f'Calculated wheel speeds: {[f"{speed:.3f}" for speed in wheel_speeds]}')
        
        # PID kontrolü uygula
        controlled_speeds = []
        for i, (target_speed, current_speed) in enumerate(zip(wheel_speeds, self.wheel_velocities)):
            if self.enable_smooth:
                controlled_speed = self.pid_controllers[i].compute(target_speed, current_speed)
                controlled_speeds.append(controlled_speed)
            else:
                controlled_speeds.append(target_speed)
                
        # Debug PID output
        if any(abs(speed) > 0.01 for speed in controlled_speeds):
            self.get_logger().info(f'PID controlled speeds: {[f"{speed:.3f}" for speed in controlled_speeds]}')
                
        # Wheel slip detection
        self.detect_wheel_slip()
        
        # Tekerlek komutlarını yayınla
        self.publish_wheel_commands(controlled_speeds)
        
        # Filtered cmd_vel'i yayınla
        self.filtered_cmd_pub.publish(self.filtered_cmd_vel)
        
    def calculate_wheel_speeds(self, cmd_vel):
        """CMD_VEL'den tekerlek hızlarını hesapla"""
        linear_vel = cmd_vel.linear.x
        angular_vel = cmd_vel.angular.z
        
        # Differential drive kinematics
        left_wheel_speed = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_wheel_speed = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        
        # 6 tekerlek için hızları dağıt (her tarafta 3 tekerlek)
        wheel_speeds = [
            left_wheel_speed,   # front_left
            left_wheel_speed,   # mid_left  
            left_wheel_speed,   # back_left
            right_wheel_speed,  # front_right
            right_wheel_speed,  # mid_right
            right_wheel_speed   # back_right
        ]
        
        return wheel_speeds
        
    def smooth_velocity_transition(self, current_vel, target_vel, dt):
        """Yumuşak hız geçişi sağla"""
        smooth_vel = Twist()
        
        # Linear velocity smoothing - Daha responsive
        linear_diff = target_vel.linear.x - current_vel.linear.x
        max_linear_change = self.max_linear_acc * dt  # %60 kısıtlaması kaldırıldı
        
        if abs(linear_diff) > max_linear_change:
            linear_diff = math.copysign(max_linear_change, linear_diff)
            
        smooth_vel.linear.x = current_vel.linear.x + linear_diff
        
        # Angular velocity smoothing - Daha responsive
        angular_diff = target_vel.angular.z - current_vel.angular.z
        max_angular_change = self.max_angular_acc * dt  # %60 kısıtlaması kaldırıldı
        
        if abs(angular_diff) > max_angular_change:
            angular_diff = math.copysign(max_angular_change, angular_diff)
            
        smooth_vel.angular.z = current_vel.angular.z + angular_diff
        
        # Velocity limits - Daha yüksek limitler
        smooth_vel.linear.x = max(-self.max_linear_vel * 0.9, 
                                 min(self.max_linear_vel * 0.9, smooth_vel.linear.x))
        smooth_vel.angular.z = max(-self.max_angular_vel * 0.9, 
                                  min(self.max_angular_vel * 0.9, smooth_vel.angular.z))
        
        return smooth_vel
        
    def detect_wheel_slip(self):
        """Tekerlek kayması tespiti"""
        # Sol ve sağ tekerleklerin ortalama hızlarını hesapla
        left_avg_speed = np.mean(self.wheel_velocities[:3])
        right_avg_speed = np.mean(self.wheel_velocities[3:])
        
        # Her tekerleğin grup ortalamasından sapmasını kontrol et
        for i in range(3):  # Sol tekerlekler
            slip = abs(self.wheel_velocities[i] - left_avg_speed)
            if slip > self.wheel_slip_threshold:
                self.get_logger().warn(f'Left wheel {i} slip detected: {slip:.3f}')
                
        for i in range(3, 6):  # Sağ tekerlekler
            slip = abs(self.wheel_velocities[i] - right_avg_speed)
            if slip > self.wheel_slip_threshold:
                self.get_logger().warn(f'Right wheel {i-3} slip detected: {slip:.3f}')
                
    def publish_wheel_commands(self, wheel_speeds):
        """Tekerlek komutlarını yayınla"""
        # 6 tekerlek hızından sol ve sağ ortalama hızları hesapla
        left_wheel_speed = (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2]) / 3.0
        right_wheel_speed = (wheel_speeds[3] + wheel_speeds[4] + wheel_speeds[5]) / 3.0
        
        # Debug logging
        if abs(left_wheel_speed) > 0.01 or abs(right_wheel_speed) > 0.01:
            self.get_logger().info(f'Wheel speeds - Left: {left_wheel_speed:.3f}, Right: {right_wheel_speed:.3f}')
        
        # Differential drive kinematics ile Twist mesajına çevir
        linear_vel = (left_wheel_speed + right_wheel_speed) * self.wheel_radius / 2.0
        angular_vel = (right_wheel_speed - left_wheel_speed) * self.wheel_radius / self.wheel_separation
        
        # Debug logging
        if abs(linear_vel) > 0.01 or abs(angular_vel) > 0.01:
            self.get_logger().info(f'Output cmd_vel - Linear: {linear_vel:.3f}, Angular: {angular_vel:.3f}')
        
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_vel
        cmd_msg.angular.z = angular_vel
        self.wheel_cmd_pub.publish(cmd_msg)
        
    def publish_odometry(self):
        """Odometry yayınla"""
        # Tekerlek pozisyonlarından odometry hesapla
        self.calculate_odometry()
        
        current_time = self.get_clock().now()
        
        # Odometry mesajı oluştur
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        
        odom_msg.pose.pose.position.x = self.robot_position[0]
        odom_msg.pose.pose.position.y = self.robot_position[1]
        odom_msg.pose.pose.orientation.z = math.sin(self.robot_position[2] / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.robot_position[2] / 2.0)
        
        odom_msg.twist.twist.linear.x = self.robot_velocity[0]
        odom_msg.twist.twist.angular.z = self.robot_velocity[1]
        
        self.odom_pub.publish(odom_msg)
        
        # TF Transform yayınla - RViz odometry sorunu çözümü
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        
        # Position
        transform.transform.translation.x = self.robot_position[0]
        transform.transform.translation.y = self.robot_position[1]
        transform.transform.translation.z = 0.0
        
        # Orientation (quaternion)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(self.robot_position[2] / 2.0)
        transform.transform.rotation.w = math.cos(self.robot_position[2] / 2.0)
        
        # TF transform'u yayınla
        self.tf_broadcaster.sendTransform(transform)
        
    def calculate_odometry(self):
        """Tekerlek pozisyonlarından odometry hesapla"""
        # Sol ve sağ tekerleklerin ortalama değişimini hesapla
        left_wheel_delta = 0.0
        right_wheel_delta = 0.0
        
        for i in range(3):  # Sol tekerlekler
            delta = self.wheel_positions[i] - self.last_wheel_positions[i]
            left_wheel_delta += delta / 3.0
            
        for i in range(3, 6):  # Sağ tekerlekler
            delta = self.wheel_positions[i] - self.last_wheel_positions[i]
            right_wheel_delta += delta / 3.0
            
        # Linear ve angular hareket hesapla
        left_distance = left_wheel_delta * self.wheel_radius
        right_distance = right_wheel_delta * self.wheel_radius
        
        linear_distance = (left_distance + right_distance) / 2.0
        angular_distance = (right_distance - left_distance) / self.wheel_separation
        
        # Robot pozisyonunu güncelle
        self.robot_position[0] += linear_distance * math.cos(self.robot_position[2])
        self.robot_position[1] += linear_distance * math.sin(self.robot_position[2])
        self.robot_position[2] += angular_distance
        
        # Hızları güncelle (basit diferansiyel)
        dt = 1.0 / 50.0  # 50Hz odometry
        self.robot_velocity[0] = linear_distance / dt
        self.robot_velocity[1] = angular_distance / dt
        
        # Son pozisyonları kaydet
        self.last_wheel_positions = self.wheel_positions.copy()


def main(args=None):
    rclpy.init(args=args)
    controller = SmoothDiffDriveController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        controller.get_logger().error(f'Controller error: {e}')
    finally:
        try:
            controller.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main() 
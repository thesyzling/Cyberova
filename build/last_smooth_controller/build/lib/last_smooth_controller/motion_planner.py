#!/usr/bin/env python3

import numpy as np
import math
from geometry_msgs.msg import Twist


class MotionPlanner:
    """
    6 tekerlekli robot için motion planning sınıfı
    Pürüzsüz ve optimize edilmiş hareket yörüngeleri oluşturur
    """
    
    def __init__(self, max_linear_vel=2.0, max_angular_vel=1.0, 
                 max_linear_acc=3.0, max_angular_acc=2.0):
        """
        Motion Planner inicializasyonu
        
        Args:
            max_linear_vel (float): Maksimum linear hız (m/s)
            max_angular_vel (float): Maksimum angular hız (rad/s)
            max_linear_acc (float): Maksimum linear ivme (m/s²)
            max_angular_acc (float): Maksimum angular ivme (rad/s²)
        """
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.max_linear_acc = max_linear_acc
        self.max_angular_acc = max_angular_acc
        
        # Planning parameters
        self.planning_horizon = 1.0  # Planning horizon (seconds)
        self.time_step = 0.01       # Time step for trajectory generation
        self.smoothness_weight = 0.5  # Smoothness vs speed trade-off
        
        # Current trajectory state
        self.current_trajectory = []
        self.trajectory_index = 0
        self.trajectory_start_time = 0.0
        
        # Acceleration limits for smooth motion
        self.acceleration_ramp_time = 0.5  # Time to reach max acceleration
        
    def plan_velocity(self, current_vel, target_vel, dt):
        """
        Mevcut hızdan hedef hıza yumuşak geçiş planla
        
        Args:
            current_vel (Twist): Mevcut hız
            target_vel (Twist): Hedef hız
            dt (float): Zaman adımı
            
        Returns:
            Twist: Planlanan hız komutu
        """
        planned_vel = Twist()
        
        # Linear velocity planning
        planned_vel.linear.x = self._plan_single_axis_velocity(
            current_vel.linear.x, target_vel.linear.x, 
            self.max_linear_vel, self.max_linear_acc, dt)
            
        planned_vel.linear.y = self._plan_single_axis_velocity(
            current_vel.linear.y, target_vel.linear.y,
            self.max_linear_vel, self.max_linear_acc, dt)
            
        # Angular velocity planning
        planned_vel.angular.z = self._plan_single_axis_velocity(
            current_vel.angular.z, target_vel.angular.z,
            self.max_angular_vel, self.max_angular_acc, dt)
            
        return planned_vel
        
    def _plan_single_axis_velocity(self, current_vel, target_vel, max_vel, max_acc, dt):
        """
        Tek eksen için hız planlama
        """
        velocity_error = target_vel - current_vel
        
        # Maximum velocity change this time step
        max_velocity_change = max_acc * dt
        
        # Limit the velocity change
        if abs(velocity_error) <= max_velocity_change:
            planned_vel = target_vel
        else:
            direction = 1.0 if velocity_error > 0 else -1.0
            planned_vel = current_vel + direction * max_velocity_change
            
        # Apply velocity limits
        planned_vel = max(-max_vel, min(max_vel, planned_vel))
        
        return planned_vel
        
    def generate_trajectory(self, start_pose, goal_pose, start_vel=None, goal_vel=None):
        """
        Başlangıç pozisyonundan hedef pozisyona yumuşak yörünge oluştur
        
        Args:
            start_pose (list): [x, y, theta] başlangıç pozisyonu
            goal_pose (list): [x, y, theta] hedef pozisyonu
            start_vel (Twist): Başlangıç hızı (opsiyonel)
            goal_vel (Twist): Hedef hızı (opsiyonel)
            
        Returns:
            list: Trajectory points listesi
        """
        if start_vel is None:
            start_vel = Twist()
        if goal_vel is None:
            goal_vel = Twist()
            
        # Calculate trajectory duration
        distance = math.sqrt((goal_pose[0] - start_pose[0])**2 + 
                           (goal_pose[1] - start_pose[1])**2)
        angle_diff = abs(goal_pose[2] - start_pose[2])
        
        # Estimate trajectory time based on distance and max velocities
        time_for_distance = distance / (self.max_linear_vel * 0.8)  # 80% of max speed
        time_for_rotation = angle_diff / (self.max_angular_vel * 0.8)
        trajectory_time = max(time_for_distance, time_for_rotation, 1.0)  # Minimum 1 second
        
        # Generate trajectory points
        trajectory = []
        num_points = int(trajectory_time / self.time_step)
        
        for i in range(num_points + 1):
            t = i * self.time_step
            normalized_time = t / trajectory_time
            
            # Use quintic polynomial for smooth trajectory
            pos_coeff = self._quintic_polynomial(normalized_time)
            vel_coeff = self._quintic_polynomial_derivative(normalized_time) / trajectory_time
            
            # Interpolate position
            x = start_pose[0] + (goal_pose[0] - start_pose[0]) * pos_coeff
            y = start_pose[1] + (goal_pose[1] - start_pose[1]) * pos_coeff
            theta = start_pose[2] + (goal_pose[2] - start_pose[2]) * pos_coeff
            
            # Calculate velocities
            vel_x = (goal_pose[0] - start_pose[0]) * vel_coeff
            vel_y = (goal_pose[1] - start_pose[1]) * vel_coeff
            vel_theta = (goal_pose[2] - start_pose[2]) * vel_coeff
            
            # Create trajectory point
            traj_point = {
                'time': t,
                'pose': [x, y, theta],
                'velocity': [vel_x, vel_y, vel_theta]
            }
            trajectory.append(traj_point)
            
        return trajectory
        
    def _quintic_polynomial(self, t):
        """
        Quintic polynomial for smooth trajectory generation
        Boundary conditions: f(0)=0, f(1)=1, f'(0)=0, f'(1)=0, f''(0)=0, f''(1)=0
        """
        return 6*t**5 - 15*t**4 + 10*t**3
        
    def _quintic_polynomial_derivative(self, t):
        """Quintic polynomial'in türevi"""
        return 30*t**4 - 60*t**3 + 30*t**2
        
    def plan_turn_in_place(self, current_heading, target_heading, current_angular_vel=0.0):
        """
        Yerinde dönüş hareketi planla
        
        Args:
            current_heading (float): Mevcut heading (radyan)
            target_heading (float): Hedef heading (radyan)
            current_angular_vel (float): Mevcut angular velocity
            
        Returns:
            Twist: Planlanan hareket komutu
        """
        # Normalize angle difference
        angle_diff = target_heading - current_heading
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        # Calculate desired angular velocity
        max_angular_vel_for_turn = min(self.max_angular_vel, 
                                     math.sqrt(2 * self.max_angular_acc * abs(angle_diff)))
        
        if abs(angle_diff) < 0.05:  # Close enough to target
            target_angular_vel = 0.0
        else:
            direction = 1.0 if angle_diff > 0 else -1.0
            target_angular_vel = direction * max_angular_vel_for_turn
            
        # Apply acceleration limits
        angular_vel_change = target_angular_vel - current_angular_vel
        max_angular_change = self.max_angular_acc * 0.01  # Assuming 100Hz control
        
        if abs(angular_vel_change) > max_angular_change:
            direction = 1.0 if angular_vel_change > 0 else -1.0
            target_angular_vel = current_angular_vel + direction * max_angular_change
            
        # Create command
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0  # No forward motion during turn
        cmd_vel.angular.z = target_angular_vel
        
        return cmd_vel
        
    def plan_arc_motion(self, linear_vel, angular_vel, safety_factor=0.8):
        """
        Arc hareket planla (eğri yol)
        
        Args:
            linear_vel (float): İstenilen linear hız
            angular_vel (float): İstenilen angular hız
            safety_factor (float): Güvenlik faktörü (0-1)
            
        Returns:
            Twist: Optimize edilmiş hareket komutu
        """
        cmd_vel = Twist()
        
        # Calculate required radius for the arc
        if abs(angular_vel) > 0.001:  # Avoid division by zero
            radius = abs(linear_vel / angular_vel)
            
            # Check if the motion is kinematically feasible
            # For differential drive, maximum curvature is limited
            min_radius = 0.1  # Minimum turning radius (meters)
            
            if radius < min_radius:
                # Reduce linear velocity to maintain minimum radius
                linear_vel = min_radius * abs(angular_vel)
                
        # Apply safety factor and velocity limits
        cmd_vel.linear.x = max(-self.max_linear_vel * safety_factor, 
                              min(self.max_linear_vel * safety_factor, linear_vel))
        cmd_vel.angular.z = max(-self.max_angular_vel * safety_factor, 
                               min(self.max_angular_vel * safety_factor, angular_vel))
        
        return cmd_vel
        
    def plan_stop_motion(self, current_vel, stop_distance=None):
        """
        Yumuşak durma hareketi planla
        
        Args:
            current_vel (Twist): Mevcut hız
            stop_distance (float): Durma mesafesi (opsiyonel)
            
        Returns:
            Twist: Durma komutu
        """
        cmd_vel = Twist()
        
        # Calculate deceleration needed
        current_speed = abs(current_vel.linear.x)
        current_angular_speed = abs(current_vel.angular.z)
        
        if stop_distance is not None and stop_distance > 0:
            # Calculate required deceleration for given stop distance
            required_decel = current_speed**2 / (2 * stop_distance)
            decel_to_use = min(required_decel, self.max_linear_acc)
        else:
            # Use maximum comfortable deceleration
            decel_to_use = self.max_linear_acc * 0.7  # 70% of max for comfort
            
        # Calculate time step deceleration
        dt = 0.01  # Assuming 100Hz control
        linear_decel_step = decel_to_use * dt
        angular_decel_step = self.max_angular_acc * 0.7 * dt
        
        # Apply deceleration
        if current_speed > linear_decel_step:
            direction = 1.0 if current_vel.linear.x > 0 else -1.0
            cmd_vel.linear.x = current_vel.linear.x - direction * linear_decel_step
        else:
            cmd_vel.linear.x = 0.0
            
        if current_angular_speed > angular_decel_step:
            direction = 1.0 if current_vel.angular.z > 0 else -1.0
            cmd_vel.angular.z = current_vel.angular.z - direction * angular_decel_step
        else:
            cmd_vel.angular.z = 0.0
            
        return cmd_vel
        
    def set_planning_parameters(self, max_linear_vel=None, max_angular_vel=None,
                               max_linear_acc=None, max_angular_acc=None,
                               planning_horizon=None, smoothness_weight=None):
        """Planning parametrelerini güncelle"""
        if max_linear_vel is not None:
            self.max_linear_vel = max_linear_vel
        if max_angular_vel is not None:
            self.max_angular_vel = max_angular_vel
        if max_linear_acc is not None:
            self.max_linear_acc = max_linear_acc
        if max_angular_acc is not None:
            self.max_angular_acc = max_angular_acc
        if planning_horizon is not None:
            self.planning_horizon = planning_horizon
        if smoothness_weight is not None:
            self.smoothness_weight = smoothness_weight
            
    def get_planning_info(self):
        """Planning bilgilerini döndür"""
        return {
            'max_linear_vel': self.max_linear_vel,
            'max_angular_vel': self.max_angular_vel,
            'max_linear_acc': self.max_linear_acc,
            'max_angular_acc': self.max_angular_acc,
            'planning_horizon': self.planning_horizon,
            'smoothness_weight': self.smoothness_weight,
            'trajectory_points': len(self.current_trajectory)
        } 
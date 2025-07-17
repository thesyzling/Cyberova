#!/usr/bin/env python3

import numpy as np
from collections import deque
from geometry_msgs.msg import Twist


class VelocityFilter:
    """
    6 tekerlekli robot için velocity filtering sınıfı
    Pürüzsüz hareket için hız komutlarını filtreler
    """
    
    def __init__(self, cutoff_freq=10.0, sample_rate=100.0, filter_type='lowpass'):
        """
        Velocity Filter inicializasyonu
        
        Args:
            cutoff_freq (float): Kesim frekansı (Hz)
            sample_rate (float): Örnekleme frekansı (Hz)
            filter_type (str): Filter tipi ('lowpass', 'moving_average', 'exponential')
        """
        self.cutoff_freq = cutoff_freq
        self.sample_rate = sample_rate
        self.filter_type = filter_type
        
        # Low-pass filter coefficients
        self.alpha = self._calculate_alpha(cutoff_freq, sample_rate)
        
        # Filter state
        self.filtered_linear_x = 0.0
        self.filtered_linear_y = 0.0
        self.filtered_angular_z = 0.0
        
        # Moving average filter
        self.window_size = max(1, int(sample_rate / cutoff_freq))
        self.linear_x_window = deque(maxlen=self.window_size)
        self.linear_y_window = deque(maxlen=self.window_size)
        self.angular_z_window = deque(maxlen=self.window_size)
        
        # Exponential filter
        self.exp_alpha = 0.1  # Exponential smoothing factor
        
        # Spike detection
        self.spike_threshold = 2.0  # Maximum allowed change per sample
        self.last_linear_x = 0.0
        self.last_linear_y = 0.0
        self.last_angular_z = 0.0
        
    def _calculate_alpha(self, cutoff_freq, sample_rate):
        """Low-pass filter alpha coefficient hesapla"""
        if cutoff_freq <= 0 or sample_rate <= 0:
            return 1.0
            
        dt = 1.0 / sample_rate
        rc = 1.0 / (2.0 * np.pi * cutoff_freq)
        return dt / (rc + dt)
        
    def filter_velocity(self, cmd_vel):
        """
        Velocity komutunu filtrele
        
        Args:
            cmd_vel (Twist): Giriş velocity komutu
            
        Returns:
            Twist: Filtrelenmiş velocity komutu
        """
        # Spike detection ve correction
        corrected_vel = self._detect_and_correct_spikes(cmd_vel)
        
        # Filter type'a göre filtreleme uygula
        if self.filter_type == 'lowpass':
            filtered_vel = self._apply_lowpass_filter(corrected_vel)
        elif self.filter_type == 'moving_average':
            filtered_vel = self._apply_moving_average_filter(corrected_vel)
        elif self.filter_type == 'exponential':
            filtered_vel = self._apply_exponential_filter(corrected_vel)
        else:
            filtered_vel = corrected_vel
            
        return filtered_vel
        
    def _detect_and_correct_spikes(self, cmd_vel):
        """
        Ani hız değişimlerini (spike) tespit et ve düzelt
        """
        corrected_vel = Twist()
        
        # Linear X
        linear_x_change = abs(cmd_vel.linear.x - self.last_linear_x)
        if linear_x_change > self.spike_threshold:
            # Spike detected, limit the change
            direction = 1.0 if cmd_vel.linear.x > self.last_linear_x else -1.0
            corrected_vel.linear.x = self.last_linear_x + direction * self.spike_threshold
        else:
            corrected_vel.linear.x = cmd_vel.linear.x
            
        # Linear Y
        linear_y_change = abs(cmd_vel.linear.y - self.last_linear_y)
        if linear_y_change > self.spike_threshold:
            direction = 1.0 if cmd_vel.linear.y > self.last_linear_y else -1.0
            corrected_vel.linear.y = self.last_linear_y + direction * self.spike_threshold
        else:
            corrected_vel.linear.y = cmd_vel.linear.y
            
        # Angular Z
        angular_z_change = abs(cmd_vel.angular.z - self.last_angular_z)
        if angular_z_change > self.spike_threshold:
            direction = 1.0 if cmd_vel.angular.z > self.last_angular_z else -1.0
            corrected_vel.angular.z = self.last_angular_z + direction * self.spike_threshold
        else:
            corrected_vel.angular.z = cmd_vel.angular.z
            
        # Son değerleri güncelle
        self.last_linear_x = corrected_vel.linear.x
        self.last_linear_y = corrected_vel.linear.y
        self.last_angular_z = corrected_vel.angular.z
        
        return corrected_vel
        
    def _apply_lowpass_filter(self, cmd_vel):
        """Low-pass filter uygula"""
        filtered_vel = Twist()
        
        # First-order low-pass filter: y[n] = α*x[n] + (1-α)*y[n-1]
        self.filtered_linear_x = (self.alpha * cmd_vel.linear.x + 
                                 (1.0 - self.alpha) * self.filtered_linear_x)
        self.filtered_linear_y = (self.alpha * cmd_vel.linear.y + 
                                 (1.0 - self.alpha) * self.filtered_linear_y)
        self.filtered_angular_z = (self.alpha * cmd_vel.angular.z + 
                                  (1.0 - self.alpha) * self.filtered_angular_z)
        
        filtered_vel.linear.x = self.filtered_linear_x
        filtered_vel.linear.y = self.filtered_linear_y
        filtered_vel.angular.z = self.filtered_angular_z
        
        return filtered_vel
        
    def _apply_moving_average_filter(self, cmd_vel):
        """Moving average filter uygula"""
        filtered_vel = Twist()
        
        # Window'lara yeni değerleri ekle
        self.linear_x_window.append(cmd_vel.linear.x)
        self.linear_y_window.append(cmd_vel.linear.y)
        self.angular_z_window.append(cmd_vel.angular.z)
        
        # Ortalamaları hesapla
        filtered_vel.linear.x = np.mean(self.linear_x_window)
        filtered_vel.linear.y = np.mean(self.linear_y_window)
        filtered_vel.angular.z = np.mean(self.angular_z_window)
        
        return filtered_vel
        
    def _apply_exponential_filter(self, cmd_vel):
        """Exponential smoothing filter uygula"""
        filtered_vel = Twist()
        
        # Exponential smoothing: y[n] = α*x[n] + (1-α)*y[n-1]
        self.filtered_linear_x = (self.exp_alpha * cmd_vel.linear.x + 
                                 (1.0 - self.exp_alpha) * self.filtered_linear_x)
        self.filtered_linear_y = (self.exp_alpha * cmd_vel.linear.y + 
                                 (1.0 - self.exp_alpha) * self.filtered_linear_y)
        self.filtered_angular_z = (self.exp_alpha * cmd_vel.angular.z + 
                                  (1.0 - self.exp_alpha) * self.filtered_angular_z)
        
        filtered_vel.linear.x = self.filtered_linear_x
        filtered_vel.linear.y = self.filtered_linear_y
        filtered_vel.angular.z = self.filtered_angular_z
        
        return filtered_vel
        
    def set_filter_parameters(self, cutoff_freq=None, sample_rate=None, 
                            filter_type=None, exp_alpha=None, spike_threshold=None):
        """Filter parametrelerini güncelle"""
        if cutoff_freq is not None:
            self.cutoff_freq = cutoff_freq
            self.alpha = self._calculate_alpha(cutoff_freq, self.sample_rate)
            self.window_size = max(1, int(self.sample_rate / cutoff_freq))
            
        if sample_rate is not None:
            self.sample_rate = sample_rate
            self.alpha = self._calculate_alpha(self.cutoff_freq, sample_rate)
            self.window_size = max(1, int(sample_rate / self.cutoff_freq))
            
        if filter_type is not None:
            self.filter_type = filter_type
            
        if exp_alpha is not None:
            self.exp_alpha = exp_alpha
            
        if spike_threshold is not None:
            self.spike_threshold = spike_threshold
            
        # Window size değişirse deque'ları yeniden oluştur
        self.linear_x_window = deque(list(self.linear_x_window)[-self.window_size:], 
                                   maxlen=self.window_size)
        self.linear_y_window = deque(list(self.linear_y_window)[-self.window_size:], 
                                   maxlen=self.window_size)
        self.angular_z_window = deque(list(self.angular_z_window)[-self.window_size:], 
                                    maxlen=self.window_size)
        
    def reset(self):
        """Filter state'ini sıfırla"""
        self.filtered_linear_x = 0.0
        self.filtered_linear_y = 0.0
        self.filtered_angular_z = 0.0
        
        self.linear_x_window.clear()
        self.linear_y_window.clear()
        self.angular_z_window.clear()
        
        self.last_linear_x = 0.0
        self.last_linear_y = 0.0
        self.last_angular_z = 0.0
        
    def get_filter_info(self):
        """Filter bilgilerini döndür"""
        return {
            'filter_type': self.filter_type,
            'cutoff_freq': self.cutoff_freq,
            'sample_rate': self.sample_rate,
            'alpha': self.alpha,
            'window_size': self.window_size,
            'exp_alpha': self.exp_alpha,
            'spike_threshold': self.spike_threshold
        }


class AdaptiveVelocityFilter(VelocityFilter):
    """
    Adaptive velocity filter - robot durumuna göre filter parametrelerini ayarlar
    """
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # Adaptive parameters
        self.velocity_threshold_low = 0.1    # Düşük hız eşiği
        self.velocity_threshold_high = 1.0   # Yüksek hız eşiği
        
        # Farklı hız aralıkları için filter parametreleri
        self.low_speed_alpha = 0.3      # Düşük hızda daha aggressive filtering
        self.medium_speed_alpha = 0.5   # Orta hızda normal filtering
        self.high_speed_alpha = 0.8     # Yüksek hızda daha az filtering
        
        # Hareket durumu tracking
        self.movement_state = 'stopped'  # 'stopped', 'low', 'medium', 'high'
        self.state_change_threshold = 0.05  # State değişimi için eşik
        
    def filter_velocity(self, cmd_vel):
        """
        Adaptive velocity filtering - robot hızına göre filter parametrelerini ayarlar
        """
        # Current velocity magnitude hesapla
        linear_magnitude = abs(cmd_vel.linear.x)
        angular_magnitude = abs(cmd_vel.angular.z)
        total_magnitude = linear_magnitude + angular_magnitude
        
        # Movement state'i belirle
        new_state = self._determine_movement_state(total_magnitude)
        
        # State değişiminde filter parametrelerini güncelle
        if new_state != self.movement_state:
            self._update_filter_for_state(new_state)
            self.movement_state = new_state
            
        # Base class filter'ı uygula
        return super().filter_velocity(cmd_vel)
        
    def _determine_movement_state(self, velocity_magnitude):
        """Hareket durumunu belirle"""
        if velocity_magnitude < self.velocity_threshold_low:
            return 'stopped'
        elif velocity_magnitude < self.velocity_threshold_high:
            return 'medium'
        else:
            return 'high'
            
    def _update_filter_for_state(self, state):
        """State'e göre filter parametrelerini güncelle"""
        if state == 'stopped':
            # Durduğunda daha aggressive filtering
            self.exp_alpha = self.low_speed_alpha
            self.spike_threshold = 0.5
        elif state == 'medium':
            # Normal hareket
            self.exp_alpha = self.medium_speed_alpha
            self.spike_threshold = 1.0
        elif state == 'high':
            # Hızlı hareket - daha az filtering
            self.exp_alpha = self.high_speed_alpha
            self.spike_threshold = 2.0
            
    def set_adaptive_parameters(self, low_threshold=None, high_threshold=None,
                              low_alpha=None, medium_alpha=None, high_alpha=None):
        """Adaptive parametreleri ayarla"""
        if low_threshold is not None:
            self.velocity_threshold_low = low_threshold
        if high_threshold is not None:
            self.velocity_threshold_high = high_threshold
        if low_alpha is not None:
            self.low_speed_alpha = low_alpha
        if medium_alpha is not None:
            self.medium_speed_alpha = medium_alpha
        if high_alpha is not None:
            self.high_speed_alpha = high_alpha 
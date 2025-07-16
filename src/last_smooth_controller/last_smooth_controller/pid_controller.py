#!/usr/bin/env python3

import time
import numpy as np


class PIDController:
    """
    6 tekerlekli robot için optimize edilmiş PID Controller sınıfı
    """
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_output=None, min_output=None, 
                 windup_limit=None, sample_time=0.01):
        """
        PID Controller inicializasyonu
        
        Args:
            kp (float): Proportional gain
            ki (float): Integral gain  
            kd (float): Derivative gain
            max_output (float): Maksimum çıkış değeri
            min_output (float): Minimum çıkış değeri
            windup_limit (float): Integral windup limiti
            sample_time (float): Örnekleme zamanı (saniye)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.max_output = max_output
        self.min_output = min_output
        self.windup_limit = windup_limit if windup_limit else abs(max_output) if max_output else 100.0
        
        self.sample_time = sample_time
        
        # Internal state
        self.last_error = 0.0
        self.last_time = time.time()
        self.integral = 0.0
        self.derivative = 0.0
        
        # Error history for advanced filtering
        self.error_history = []
        self.max_history_length = 10
        
        # Adaptive gains
        self.adaptive_enabled = False
        self.kp_original = kp
        self.ki_original = ki
        self.kd_original = kd
        
    def compute(self, setpoint, measured_value, dt=None):
        """
        PID kontrolcüsünü hesapla
        
        Args:
            setpoint (float): Hedef değer
            measured_value (float): Ölçülen değer
            dt (float): Zaman farkı (opsiyonel)
            
        Returns:
            float: Kontrol çıkışı
        """
        current_time = time.time()
        
        if dt is None:
            dt = current_time - self.last_time
            
        if dt < self.sample_time:
            return self.last_output if hasattr(self, 'last_output') else 0.0
            
        # Error hesapla
        error = setpoint - measured_value
        
        # Error history güncelle
        self.error_history.append(error)
        if len(self.error_history) > self.max_history_length:
            self.error_history.pop(0)
            
        # Adaptive gain ayarlaması
        if self.adaptive_enabled:
            self._update_adaptive_gains(error)
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        
        # Integral windup protection
        if self.windup_limit:
            self.integral = np.clip(self.integral, -self.windup_limit, self.windup_limit)
            
        integral_term = self.ki * self.integral
        
        # Derivative term
        if dt > 0:
            self.derivative = (error - self.last_error) / dt
        else:
            self.derivative = 0.0
            
        # Derivative filtering (simple low-pass)
        alpha = 0.1  # Filter coefficient
        if hasattr(self, 'filtered_derivative'):
            self.filtered_derivative = alpha * self.derivative + (1 - alpha) * self.filtered_derivative
        else:
            self.filtered_derivative = self.derivative
            
        derivative_term = self.kd * self.filtered_derivative
        
        # PID output
        output = proportional + integral_term + derivative_term
        
        # Output limiting
        if self.max_output is not None and self.min_output is not None:
            output = np.clip(output, self.min_output, self.max_output)
        elif self.max_output is not None:
            output = min(output, self.max_output)
        elif self.min_output is not None:
            output = max(output, self.min_output)
            
        # Update for next iteration
        self.last_error = error
        self.last_time = current_time
        self.last_output = output
        
        return output
        
    def _update_adaptive_gains(self, error):
        """
        Adaptive gain güncelleme - hata büyüklüğüne göre gain'leri ayarla
        """
        error_magnitude = abs(error)
        
        # Büyük hatalar için daha aggressive gains
        if error_magnitude > 1.0:
            gain_multiplier = min(2.0, 1.0 + error_magnitude * 0.5)
        # Küçük hatalar için daha conservative gains  
        elif error_magnitude < 0.1:
            gain_multiplier = max(0.5, 1.0 - (0.1 - error_magnitude) * 2.0)
        else:
            gain_multiplier = 1.0
            
        self.kp = self.kp_original * gain_multiplier
        self.ki = self.ki_original * gain_multiplier * 0.8  # Integral gain'i daha az artır
        self.kd = self.kd_original * gain_multiplier
        
    def enable_adaptive_gains(self, enable=True):
        """Adaptive gain özelliğini aktif/pasif et"""
        self.adaptive_enabled = enable
        
    def reset(self):
        """PID controller'ı sıfırla"""
        self.last_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.error_history.clear()
        if hasattr(self, 'filtered_derivative'):
            delattr(self, 'filtered_derivative')
        if hasattr(self, 'last_output'):
            delattr(self, 'last_output')
            
    def set_gains(self, kp=None, ki=None, kd=None):
        """PID gain'lerini güncelle"""
        if kp is not None:
            self.kp = kp
            self.kp_original = kp
        if ki is not None:
            self.ki = ki
            self.ki_original = ki
        if kd is not None:
            self.kd = kd
            self.kd_original = kd
            
    def set_output_limits(self, min_output=None, max_output=None):
        """Çıkış limitlerini ayarla"""
        self.min_output = min_output
        self.max_output = max_output
        
        if max_output:
            self.windup_limit = abs(max_output)
            
    def get_error_statistics(self):
        """Error istatistiklerini döndür"""
        if not self.error_history:
            return {'mean': 0.0, 'std': 0.0, 'max': 0.0, 'min': 0.0}
            
        errors = np.array(self.error_history)
        return {
            'mean': np.mean(errors),
            'std': np.std(errors),
            'max': np.max(errors),
            'min': np.min(errors),
            'current': errors[-1] if len(errors) > 0 else 0.0
        }
        
    def get_component_values(self):
        """PID bileşenlerinin değerlerini döndür"""
        return {
            'proportional': self.kp * self.last_error,
            'integral': self.ki * self.integral, 
            'derivative': self.kd * self.derivative,
            'total': getattr(self, 'last_output', 0.0)
        }


class MultiWheelPIDController:
    """
    6 tekerlekli robot için multi-wheel PID controller
    Her tekerlek için ayrı PID instance'ı yönetir
    """
    
    def __init__(self, num_wheels=6, **pid_params):
        """
        Multi-wheel PID controller inicializasyonu
        
        Args:
            num_wheels (int): Tekerlek sayısı
            **pid_params: PID parametreleri
        """
        self.num_wheels = num_wheels
        self.controllers = []
        
        # Her tekerlek için ayrı PID controller oluştur
        for i in range(num_wheels):
            controller = PIDController(**pid_params)
            self.controllers.append(controller)
            
        # Synchronization parameters
        self.sync_enabled = True
        self.sync_threshold = 0.1  # Synchronization error threshold
        
    def compute_all(self, setpoints, measured_values, dt=None):
        """
        Tüm tekerlekler için PID hesapla
        
        Args:
            setpoints (list): Hedef değerler listesi
            measured_values (list): Ölçülen değerler listesi
            dt (float): Zaman farkı
            
        Returns:
            list: Kontrol çıkışları listesi
        """
        if len(setpoints) != self.num_wheels or len(measured_values) != self.num_wheels:
            raise ValueError(f"Expected {self.num_wheels} values, got {len(setpoints)} setpoints and {len(measured_values)} measurements")
            
        outputs = []
        
        # Her tekerlek için PID hesapla
        for i, (sp, mv) in enumerate(zip(setpoints, measured_values)):
            output = self.controllers[i].compute(sp, mv, dt)
            outputs.append(output)
            
        # Wheel synchronization
        if self.sync_enabled:
            outputs = self._synchronize_wheels(outputs, setpoints)
            
        return outputs
        
    def _synchronize_wheels(self, outputs, setpoints):
        """
        Tekerlek senkronizasyonu - aynı taraftaki tekerleklerin hızlarını senkronize et
        """
        synchronized_outputs = outputs.copy()
        
        # Sol tekerlekleri senkronize et (0, 1, 2)
        left_avg = np.mean(outputs[:3])
        left_setpoint_avg = np.mean(setpoints[:3])
        
        if left_setpoint_avg != 0:  # Sadece hareket halindeyken senkronize et
            for i in range(3):
                error = abs(outputs[i] - left_avg)
                if error > self.sync_threshold:
                    # Ortalamaya doğru yumuşak geçiş
                    correction_factor = 0.1
                    synchronized_outputs[i] += (left_avg - outputs[i]) * correction_factor
                    
        # Sağ tekerlekleri senkronize et (3, 4, 5)
        right_avg = np.mean(outputs[3:])
        right_setpoint_avg = np.mean(setpoints[3:])
        
        if right_setpoint_avg != 0:  # Sadece hareket halindeyken senkronize et
            for i in range(3, 6):
                error = abs(outputs[i] - right_avg)
                if error > self.sync_threshold:
                    # Ortalamaya doğru yumuşak geçiş
                    correction_factor = 0.1
                    synchronized_outputs[i] += (right_avg - outputs[i]) * correction_factor
                    
        return synchronized_outputs
        
    def reset_all(self):
        """Tüm PID controller'ları sıfırla"""
        for controller in self.controllers:
            controller.reset()
            
    def set_gains_all(self, kp=None, ki=None, kd=None):
        """Tüm controller'ların gain'lerini ayarla"""
        for controller in self.controllers:
            controller.set_gains(kp, ki, kd)
            
    def set_gains_individual(self, wheel_index, kp=None, ki=None, kd=None):
        """Belirli bir tekerlek için gain'leri ayarla"""
        if 0 <= wheel_index < self.num_wheels:
            self.controllers[wheel_index].set_gains(kp, ki, kd)
            
    def enable_synchronization(self, enable=True, threshold=None):
        """Tekerlek senkronizasyonunu aktif/pasif et"""
        self.sync_enabled = enable
        if threshold is not None:
            self.sync_threshold = threshold
            
    def get_all_statistics(self):
        """Tüm tekerleklerin istatistiklerini döndür"""
        stats = {}
        for i, controller in enumerate(self.controllers):
            stats[f'wheel_{i}'] = controller.get_error_statistics()
        return stats 
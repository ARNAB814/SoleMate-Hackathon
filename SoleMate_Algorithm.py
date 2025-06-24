#!/usr/bin/env python3
"""
SoleMate Smart Insole - Enhanced Fall Detection & Posture Monitoring
Hardware: Raspberry Pi 4 + MPU6050 + RGB LED + Buzzer + LCD + Vibration Sensor
Enhanced with improved algorithms and error handling
"""

import smbus2
import time
import RPi.GPIO as GPIO
import threading
from datetime import datetime
import math
import statistics
from collections import deque

# I2C Bus setup
bus = smbus2.SMBus(1)
MPU6050_ADDR = 0x68
LCD_ADDR = 0x27

# GPIO Pin assignments
LED_PINS = {'red': 18, 'green': 23, 'blue': 24}
BUZZER_PIN = 25
VIBRATION_PIN = 17

# MPU6050 Register addresses
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

class LCD1602:
    """Enhanced I2C LCD controller with better error handling"""
    def __init__(self, addr=0x27):
        self.addr = addr
        self.available = True
        try:
            self.init_lcd()
            print("LCD initialized successfully")
        except Exception as e:
            print(f"LCD not available: {e}")
            self.available = False
    
    def write_word(self, data):
        """Write data to LCD with proper timing"""
        if not self.available:
            return
        try:
            temp = data
            temp |= 0x08  # Enable backlight
            bus.write_byte(self.addr, temp)
            time.sleep(0.0001)
        except:
            self.available = False
    
    def send_command(self, comm):
        """Send command to LCD"""
        if not self.available:
            return
        # Send upper 4 bits
        buf = comm & 0xF0
        buf |= 0x04  # RS = 0, RW = 0, EN = 1
        self.write_word(buf)
        time.sleep(0.002)
        buf &= 0xFB  # Make EN = 0
        self.write_word(buf)
        
        # Send lower 4 bits
        buf = (comm & 0x0F) << 4
        buf |= 0x04  # RS = 0, RW = 0, EN = 1
        self.write_word(buf)
        time.sleep(0.002)
        buf &= 0xFB  # Make EN = 0
        self.write_word(buf)
    
    def send_data(self, data):
        """Send data to LCD"""
        if not self.available:
            return
        # Send upper 4 bits
        buf = data & 0xF0
        buf |= 0x05  # RS = 1, RW = 0, EN = 1
        self.write_word(buf)
        time.sleep(0.002)
        buf &= 0xFB  # Make EN = 0
        self.write_word(buf)
        
        # Send lower 4 bits
        buf = (data & 0x0F) << 4
        buf |= 0x05  # RS = 1, RW = 0, EN = 1
        self.write_word(buf)
        time.sleep(0.002)
        buf &= 0xFB  # Make EN = 0
        self.write_word(buf)
    
    def init_lcd(self):
        """Initialize LCD in 4-bit mode"""
        self.send_command(0x33)  # Initialize to 8-bit mode
        time.sleep(0.005)
        self.send_command(0x32)  # Initialize to 4-bit mode
        time.sleep(0.005)
        self.send_command(0x28)  # 2 lines, 5x7 dots
        time.sleep(0.005)
        self.send_command(0x0C)  # Display on, cursor off
        time.sleep(0.005)
        self.send_command(0x01)  # Clear screen
        time.sleep(0.005)
    
    def clear(self):
        """Clear LCD display"""
        self.send_command(0x01)
        time.sleep(0.002)
    
    def write(self, x, y, text):
        """Write text at specific position"""
        if not self.available:
            return
        if x < 0 or x > 15 or y < 0 or y > 1:
            return
        
        # Move cursor
        addr = 0x80 + 0x40 * y + x
        self.send_command(addr)
        
        # Send text
        for char in text[:16-x]:
            self.send_data(ord(char))
    
    def display_string(self, line1="", line2=""):
        """Display text on LCD"""
        if not self.available:
            return
        self.clear()
        if line1:
            self.write(0, 0, line1)
        if line2:
            self.write(0, 1, line2)

class MPU6050:
    """Enhanced MPU6050 sensor interface with calibration"""
    def __init__(self):
        try:
            # Wake up the sensor
            bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
            time.sleep(0.1)
            
            # Test communication
            who_am_i = bus.read_byte_data(MPU6050_ADDR, 0x75)
            if who_am_i != 0x68:
                raise Exception(f"MPU6050 not found. Got: 0x{who_am_i:02X}")
            
            # Configure accelerometer (±2g range)
            bus.write_byte_data(MPU6050_ADDR, 0x1C, 0x00)
            # Configure gyroscope (±250°/s range)
            bus.write_byte_data(MPU6050_ADDR, 0x1B, 0x00)
            
            print("MPU6050 initialized successfully")
            self.available = True
            
        except Exception as e:
            print(f"MPU6050 initialization failed: {e}")
            self.available = False
        
    def read_raw_data(self, addr):
        """Read raw 16-bit data from sensor"""
        if not self.available:
            return 0
        try:
            high = bus.read_byte_data(MPU6050_ADDR, addr)
            low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
            value = (high << 8) | low
            if value > 32768:
                value = value - 65536
            return value
        except:
            return 0
    
    def get_accelerometer_data(self):
        """Get acceleration in g-force units"""
        acc_x = self.read_raw_data(ACCEL_XOUT_H) / 16384.0
        acc_y = self.read_raw_data(ACCEL_YOUT_H) / 16384.0  
        acc_z = self.read_raw_data(ACCEL_ZOUT_H) / 16384.0
        return acc_x, acc_y, acc_z
    
    def get_gyroscope_data(self):
        """Get rotation rates in degrees/second"""
        gyro_x = self.read_raw_data(GYRO_XOUT_H) / 131.0
        gyro_y = self.read_raw_data(GYRO_YOUT_H) / 131.0
        gyro_z = self.read_raw_data(GYRO_ZOUT_H) / 131.0
        return gyro_x, gyro_y, gyro_z

class SoleMate:
    """Enhanced SoleMate application class with improved algorithms"""
    
    def __init__(self):
        self.setup_gpio()
        self.mpu = MPU6050()
        self.lcd = LCD1602()
        
        # Enhanced monitoring variables
        self.balance_score = 100
        self.fall_count = 0
        self.is_running = True
        self.baseline_calibrated = False
        self.baseline_acc = [0, 1, 0]  # Expected standing position
        
        # Data history for better analysis
        self.acc_history = deque(maxlen=20)  # Last 2 seconds at 10Hz
        self.magnitude_history = deque(maxlen=20)
        self.balance_history = deque(maxlen=10)
        
        # Enhanced fall detection parameters
        self.fall_threshold_low = 0.5   # Free fall threshold (g)
        self.fall_threshold_high = 2.5  # Impact threshold (g)
        self.gyro_threshold = 200       # Angular velocity threshold (deg/s)
        
        # Fall detection state machine
        self.fall_state = "normal"      # normal, freefall, impact
        self.fall_start_time = 0
        self.false_alarm_count = 0
        
        # Posture analysis
        self.posture_deviation_threshold = 0.3
        self.poor_posture_duration = 0
        
        print("SoleMate initialized successfully!")
        if self.lcd.available:
            self.lcd.display_string("SoleMate Ready", "Calibrating...")
        
    def setup_gpio(self):
        """Initialize GPIO pins"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup LED pins
        for pin in LED_PINS.values():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
            
        # Setup buzzer
        GPIO.setup(BUZZER_PIN, GPIO.OUT)
        GPIO.output(BUZZER_PIN, GPIO.LOW)
        
        # Setup vibration sensor with pull-up resistor
        GPIO.setup(VIBRATION_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
    def set_led_color(self, color):
        """Control RGB LED colors"""
        # Turn off all LEDs first
        for pin in LED_PINS.values():
            GPIO.output(pin, GPIO.LOW)
            
        # Turn on specific color
        if color in LED_PINS:
            GPIO.output(LED_PINS[color], GPIO.HIGH)
            
    def buzz_alert(self, duration=1.0, pattern='solid'):
        """Sound buzzer alert with different patterns"""
        def buzz_thread():
            if pattern == 'solid':
                GPIO.output(BUZZER_PIN, GPIO.HIGH)
                time.sleep(duration)
                GPIO.output(BUZZER_PIN, GPIO.LOW)
            elif pattern == 'beep':
                for _ in range(3):
                    GPIO.output(BUZZER_PIN, GPIO.HIGH)
                    time.sleep(0.2)
                    GPIO.output(BUZZER_PIN, GPIO.LOW)
                    time.sleep(0.2)
            elif pattern == 'pulse':
                for _ in range(int(duration * 5)):
                    GPIO.output(BUZZER_PIN, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(BUZZER_PIN, GPIO.LOW)
                    time.sleep(0.1)
        
        # Run buzzer in separate thread to avoid blocking
        threading.Thread(target=buzz_thread, daemon=True).start()
                
    def calibrate_baseline(self):
        """Enhanced calibration with stability check"""
        print("Calibrating baseline - stand still for 5 seconds...")
        if self.lcd.available:
            self.lcd.display_string("Stand Still", "Calibrating 5s")
        
        readings = []
        stable_readings = 0
        
        for i in range(50):  # 5 seconds at 10Hz
            acc_x, acc_y, acc_z = self.mpu.get_accelerometer_data()
            magnitude = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
            
            readings.append([acc_x, acc_y, acc_z, magnitude])
            
            # Check stability (magnitude should be close to 1g)
            if abs(magnitude - 1.0) < 0.2:
                stable_readings += 1
                
            time.sleep(0.1)
            
        if stable_readings < 30:  # Less than 60% stable readings
            print("Warning: Unstable calibration. Please stand still and try again.")
            if self.lcd.available:
                self.lcd.display_string("Unstable!", "Try again")
            time.sleep(2)
            return self.calibrate_baseline()
            
        # Calculate baseline from stable readings only
        stable_data = [r for r in readings if abs(r[3] - 1.0) < 0.2]
        
        self.baseline_acc = [
            statistics.mean(r[0] for r in stable_data),
            statistics.mean(r[1] for r in stable_data),
            statistics.mean(r[2] for r in stable_data)
        ]
        
        self.baseline_calibrated = True
        print(f"Baseline calibrated: X={self.baseline_acc[0]:.3f}, Y={self.baseline_acc[1]:.3f}, Z={self.baseline_acc[2]:.3f}")
        if self.lcd.available:
            self.lcd.display_string("Calibrated!", "Starting monitor")
        time.sleep(2)
        
    def calculate_balance_score(self, acc_x, acc_y, acc_z):
        """Enhanced balance score calculation"""
        if not self.baseline_calibrated:
            return 50  # Default score
            
        # Calculate deviation from baseline
        deviation_x = abs(acc_x - self.baseline_acc[0])
        deviation_y = abs(acc_y - self.baseline_acc[1])
        deviation_z = abs(acc_z - self.baseline_acc[2])
        
        # Weight Z-axis more heavily (vertical balance)
        weighted_deviation = deviation_x + deviation_y + 2 * deviation_z
        
        # Convert to score with non-linear scaling
        score = max(0, 100 - int(weighted_deviation * 100))
        
        # Apply smoothing using history
        self.balance_history.append(score)
        if len(self.balance_history) > 1:
            score = int(statistics.mean(self.balance_history))
            
        return min(100, max(0, score))
    
    def analyze_posture(self, acc_x, acc_y, acc_z):
        """Analyze posture quality"""
        if not self.baseline_calibrated:
            return "unknown"
            
        # Calculate tilt angles
        pitch = math.atan2(acc_x, math.sqrt(acc_y**2 + acc_z**2)) * 180 / math.pi
        roll = math.atan2(acc_y, acc_z) * 180 / math.pi
        
        # Determine posture quality
        if abs(pitch) < 10 and abs(roll) < 10:
            self.poor_posture_duration = 0
            return "excellent"
        elif abs(pitch) < 20 and abs(roll) < 20:
            self.poor_posture_duration = 0
            return "good"
        elif abs(pitch) < 30 and abs(roll) < 30:
            self.poor_posture_duration += 0.5
            return "fair"
        else:
            self.poor_posture_duration += 0.5
            return "poor"
        
    def enhanced_fall_detection(self, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
        """Enhanced fall detection with state machine"""
        current_time = time.time()
        
        # Calculate metrics
        total_acc = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
        total_gyro = math.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2)
        
        # Store in history
        self.acc_history.append((acc_x, acc_y, acc_z))
        self.magnitude_history.append(total_acc)
        
        # Calculate acceleration variance (indicates sudden changes)
        acc_variance = 0
        if len(self.magnitude_history) > 5:
            acc_variance = statistics.variance(list(self.magnitude_history)[-5:])
        
        # Check vibration sensor
        vibration_triggered = GPIO.input(VIBRATION_PIN) == GPIO.LOW
        
        # State machine for fall detection
        if self.fall_state == "normal":
            # Check for free fall (low acceleration + high variance)
            if total_acc < self.fall_threshold_low and acc_variance > 0.1:
                self.fall_state = "freefall"
                self.fall_start_time = current_time
                print("🟡 Free fall detected - monitoring for impact...")
                return "monitoring"
                
        elif self.fall_state == "freefall":
            # Check for impact after free fall
            fall_duration = current_time - self.fall_start_time
            
            if fall_duration > 2.0:  # Free fall too long, reset
                self.fall_state = "normal"
                self.false_alarm_count += 1
                return "false_alarm"
                
            # Look for impact: high acceleration + high rotation + vibration
            impact_score = 0
            if total_acc > self.fall_threshold_high:
                impact_score += 3
            if total_gyro > self.gyro_threshold:
                impact_score += 2
            if vibration_triggered:
                impact_score += 2
            if acc_variance > 0.5:
                impact_score += 1
                
            if impact_score >= 5:  # Strong evidence of fall
                self.fall_state = "impact"
                print(f"🔴 FALL CONFIRMED! Impact score: {impact_score}")
                return "fall_confirmed"
                
        elif self.fall_state == "impact":
            # Recovery period - return to normal after 3 seconds
            if current_time - self.fall_start_time > 3.0:
                self.fall_state = "normal"
                
        # Check for sudden impact without free fall (e.g., stumbling)
        if (self.fall_state == "normal" and 
            total_acc > self.fall_threshold_high and 
            vibration_triggered and 
            acc_variance > 0.3):
            print("🟠 Sudden impact detected")
            return "impact_only"
            
        return "normal"
        
    def monitor_loop(self):
        """Enhanced monitoring loop with comprehensive analysis"""
        if not self.baseline_calibrated:
            self.calibrate_baseline()
            
        print("Starting enhanced SoleMate monitoring...")
        print("Monitoring: Balance, Posture, Falls, and Movement patterns")
        
        loop_count = 0
        last_alert_time = 0
        
        while self.is_running:
            try:
                # Read sensor data
                acc_x, acc_y, acc_z = self.mpu.get_accelerometer_data()
                gyro_x, gyro_y, gyro_z = self.mpu.get_gyroscope_data()
                
                # Calculate balance score
                self.balance_score = self.calculate_balance_score(acc_x, acc_y, acc_z)
                
                # Analyze posture
                posture_quality = self.analyze_posture(acc_x, acc_y, acc_z)
                
                # Enhanced fall detection
                fall_status = self.enhanced_fall_detection(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
                
                current_time = time.time()
                
                # Handle fall detection results
                if fall_status == "fall_confirmed":
                    self.fall_count += 1
                    print(f"🚨 FALL DETECTED! Total falls: {self.fall_count}")
                    if self.lcd.available:
                        self.lcd.display_string("FALL DETECTED!", f"Count: {self.fall_count}")
                    self.set_led_color('red')
                    self.buzz_alert(3.0, 'beep')
                    last_alert_time = current_time
                    
                elif fall_status == "monitoring":
                    self.set_led_color('blue')
                    if current_time - last_alert_time > 5:  # Avoid spam
                        self.buzz_alert(0.5, 'pulse')
                        last_alert_time = current_time
                        
                elif fall_status == "impact_only":
                    print("⚠️ Impact detected - possible stumble")
                    self.set_led_color('blue')
                    
                else:
                    # Normal operation - show status based on balance and posture
                    if self.balance_score >= 80 and posture_quality in ["excellent", "good"]:
                        self.set_led_color('green')
                    elif self.balance_score >= 60 and posture_quality in ["excellent", "good", "fair"]:
                        self.set_led_color('blue')
                    else:
                        self.set_led_color('red')
                        if self.poor_posture_duration > 30:  # 30 seconds of poor posture
                            if current_time - last_alert_time > 30:  # Alert every 30 seconds
                                self.buzz_alert(1.0, 'pulse')
                                last_alert_time = current_time
                
                # Update display every 2 seconds
                if loop_count % 4 == 0:
                    timestamp = datetime.now().strftime("%H:%M")
                    if self.lcd.available:
                        if fall_status not in ["fall_confirmed"]:
                            line1 = f"Bal:{self.balance_score}% {posture_quality[:4].title()}"
                            line2 = f"Falls:{self.fall_count} {timestamp}"
                            self.lcd.display_string(line1, line2)
                
                # Print comprehensive debug info
                total_acc = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
                total_gyro = math.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2)
                
                print(f"Balance: {self.balance_score:3d}% | Posture: {posture_quality:9s} | "
                      f"Acc: {total_acc:5.2f}g | Gyro: {total_gyro:6.1f}°/s | "
                      f"Status: {fall_status}")
                
                loop_count += 1
                time.sleep(0.5)  # 2Hz update rate
                
            except KeyboardInterrupt:
                print("\nShutting down SoleMate...")
                break
            except Exception as e:
                print(f"Error in monitoring loop: {e}")
                time.sleep(1)
                
    def cleanup(self):
        """Clean up GPIO and display"""
        self.is_running = False
        GPIO.cleanup()
        if self.lcd.available:
            self.lcd.clear()
            self.lcd.display_string("SoleMate", "Powered Off")
        print("SoleMate shutdown complete.")
        print(f"Session summary: {self.fall_count} falls detected, {self.false_alarm_count} false alarms")

def main():
    """Main application entry point"""
    print("=" * 50)
    print("SoleMate Smart Insole System v2.0")
    print("Enhanced Fall Detection & Posture Monitoring")
    print("=" * 50)
    
    try:
        solemate = SoleMate()
        solemate.monitor_loop()
    except Exception as e:
        print(f"Error starting SoleMate: {e}")
    finally:
        if 'solemate' in locals():
            solemate.cleanup()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
# Use lgpio or RPi.GPIO depending on your Raspberry Pi OS version
import RPi.GPIO as GPIO

class IBT2EncoderDriver(Node):
    def __init__(self):
        super().__init__('ibt2_encoder_driver')
        
        # Pin Definitions (BCM numbering)
        self.RPWM_PIN = 18
        self.LPWM_PIN = 13
        self.ENC_A = 22
        self.ENC_B = 27
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.RPWM_PIN, GPIO.OUT)
        GPIO.setup(self.LPWM_PIN, GPIO.OUT)
        self.pwm_r = GPIO.PWM(self.RPWM_PIN, 1000) # 1 kHz frequency
        self.pwm_l = GPIO.PWM(self.LPWM_PIN, 1000)
        self.pwm_r.start(0)
        self.pwm_l.start(0)
        
        # Encoder setup
        GPIO.setup(self.ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        self.encoder_ticks = 0
        GPIO.add_event_detect(self.ENC_A, GPIO.RISING, callback=self.encoder_callback)
        
        # ROS Subscriptions and Publishers
        self.sub_speed = self.create_subscription(Float32, 'motor_speed', self.speed_callback, 10)
        self.pub_enc = self.create_publisher(Int32, 'encoder_ticks', 10)
        
        self.timer = self.create_timer(0.1, self.publish_encoder)

    def encoder_callback(self, channel):
        if GPIO.input(self.ENC_B):
            self.encoder_ticks += 1
        else:
            self.encoder_ticks -= 1

    def speed_callback(self, msg: Float32):
        speed = msg.data
        if speed > 0:
            self.pwm_r.ChangeDutyCycle(abs(speed))
            self.pwm_l.ChangeDutyCycle(0)
        elif speed < 0:
            self.pwm_r.ChangeDutyCycle(0)
            self.pwm_l.ChangeDutyCycle(abs(speed))
        else:
            self.pwm_r.ChangeDutyCycle(0)
            self.pwm_l.ChangeDutyCycle(0)

    def publish_encoder(self):
        msg = Int32()
        msg.data = self.encoder_ticks
        self.pub_enc.publish(msg)

    def destroy_node(self):
        self.pwm_r.stop()
        self.pwm_l.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IBT2EncoderDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

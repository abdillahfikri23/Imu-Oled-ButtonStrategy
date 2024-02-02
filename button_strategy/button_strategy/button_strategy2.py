import RPi.GPIO as GPIO
import rclpy
from bfc_msgs.msg import Button
from sensor_msgs.msg import Imu
import time

STRATEGY_PIN = 15  # Pin untuk tombol strategi
KILL_PIN = 7     # Pin untuk tombol kill

GPIO.setmode(GPIO.BOARD)
GPIO.setup(STRATEGY_PIN, GPIO.IN)
GPIO.setup(KILL_PIN, GPIO.IN)

print("ready")
strategyNumber = 0
kill = 0
roll = 0
pitch = 0
yaw = 0

def read_button_state(pin):
    return GPIO.input(pin)
    
 
class ImuSubscriberNode:
    def __init__(self):
        #self.node = rclpy.create_node('imu_subscriber')
        #self.sub = self.node.create_subscription(Imu, 'imu_bno', self.imu_callback, 10)
        
        self.pub_button = self.node.create_publisher(Button, 'button_new', 10)
        self.msg_button = Button()
        
    #def imu_callback(self, msg):
        
        #roll_degrees = msg.angular_velocity.x
        #pitch_degrees = msg.angular_velocity.y
        #yaw_degrees = msg.angular_velocity.z 
                
        #print("Imu_Roll =", roll_degrees)
        #print("Imu_Pitch =", pitch_degrees)
        #print("Imu_Yaw =", yaw_degrees)
        
        global strategyNumber
        global kill
    
        last_button_state = GPIO.HIGH
        button_pressed_time = 0
        kill_pressed_time = 0
   
        print("Program dimulai. Tekan tombol untuk menampilkan angka 0 hingga 4.")

        while True:
              
            # Tombol strategi baru saja ditekan
            strategy_state = read_button_state(STRATEGY_PIN)
            if strategy_state == GPIO.LOW and button_pressed_time == 0:
                button_pressed_time = time.time()
            
            # Tombol kill baru saja ditekan
            kill_state = read_button_state(KILL_PIN)
            if kill_state == GPIO.LOW and kill_pressed_time == 0:
                kill_pressed_time = time.time()

            # Tombol dilepas (strategi)
            elif strategy_state == GPIO.HIGH and button_pressed_time > 0:
                button_duration = time.time() - button_pressed_time

                if button_duration < 1.2:
                    strategyNumber = (strategyNumber + 1) % 5
                    print("Strategy =", strategyNumber)
                    print("Imu_Roll =", roll_degrees)
                else:
                    strategyNumber = 4
                    print("Kembali ke angka 4")

                button_pressed_time = 0

            # Tombol dilepas (kill)
            elif kill_state == GPIO.HIGH and kill_pressed_time > 0:
                kill_duration = time.time() - kill_pressed_time
                
                if kill_duration < 1.2:
                    kill = (kill + 1) % 2
                    print("Kill =", kill)
                else:
                    kill = 6
                    print("Kalibrasi Imu")
                    
                kill_pressed_time = 0
       
            time.sleep(0.1)
            self.msg_button.strategy = strategyNumber
            self.msg_button.kill = kill
            self.pub_button.publish(self.msg_button)


def main(args=None):
    rclpy.init(args=args)
    imu_subscriber_node = ImuSubscriberNode()
    rclpy.spin(imu_subscriber_node.node)
    
    
    node.destroy_node()
    rclpy.shutdown() 
    GPIO.cleanup()

if __name__ == '__main__':
    main()


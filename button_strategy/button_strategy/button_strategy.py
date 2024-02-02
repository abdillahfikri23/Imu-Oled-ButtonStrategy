import RPi.GPIO as GPIO
import rclpy
from bfc_msgs.msg import Button
from sensor_msgs.msg import Imu
import time

STRATEGY_PIN = 32  # Pin untuk tombol strategi
KILL_PIN = 15     # Pin untuk tombol kill

GPIO.setmode(GPIO.BOARD)
GPIO.setup(STRATEGY_PIN, GPIO.IN)
GPIO.setup(KILL_PIN, GPIO.IN)

strategyNumber = 0
kill = 0
roll = 0
pitch = 0
yaw = 0

def read_button_state(pin):
    return GPIO.input(pin)


def main(args=None):
    global strategyNumber
    global kill
    
    last_button_state = GPIO.HIGH
    button_pressed_time = 0
    kill_pressed_time = 0
    
    rclpy.init(args=args)
    node = rclpy.create_node('button_imu')
    pub_button = node.create_publisher(Button, 'button_new', 10)
    msg_button = Button()

    try:
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
                    #print("Strategy =", strategyNumber)
                else:
                    strategyNumber = 4
                    #print("Kembali ke angka 4")

                    button_pressed_time = 0

            # Tombol dilepas (kill)
            elif kill_state == GPIO.HIGH and kill_pressed_time > 0:
                kill_duration = time.time() - kill_pressed_time
                
                if kill_duration < 1.2:
                    kill = (kill + 1) % 2
                    #print("Kill =", kill)
                else:
                    #Indikator untuk kalibrasi imu
                    kill = 3
                    #print("Kill =", kill)
                    
                kill_pressed_time = 0
       
            time.sleep(0.1)
            msg_button.strategy = strategyNumber
            msg_button.kill = kill
            pub_button.publish(msg_button)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown() 
        GPIO.cleanup()

if __name__ == '__main__':
    main()


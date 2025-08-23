# Execute source /opt/ros/noetic/setup.bash before running this code

# Set Pin Factory to pigpio https://gpiozero.readthedocs.io/en/stable/api_pins.html#changing-the-pin-factory
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Device, AngularServo
from gpiozero.tools import scaled
from signal import pause

Device.pin_factory = PiGPIOFactory()

##### Taken from gpiozero source #####
from random import random
from time import sleep
from itertools import cycle
from math import sin, cos, pi, isclose
from statistics import mean

import rosgraph
import rospy
from std_msgs.msg import Float32MultiArray

import zmq
import threading


SERVO_GPIO_PIN_1 = 12   # Servo 1 PWM pin
SERVO_GPIO_PIN_2 = 13   # Servo 2 PWM pin

SERVO_UPDATE_RATE_1 = 0.01 # How frequent does the servo angle is updated
SERVO_UPDATE_RATE_2 = 0.01 # seconds


def map_2_range(value, old_min=200, old_max=1800, new_min=-10, new_max=10):
    if value < old_min:
        return new_min
    elif value > old_max:
        return new_max
    new_value = int((value-old_min) * (new_max-new_min) / (old_max-old_min) + new_min)
    return new_value

def channels_2_dir(old_channels):
    assert len(old_channels) == 18
    channels = list(map(map_2_range, old_channels[0:16]))
    LeftVert = channels[2]
    LeftHori = channels[3]
    RightVert = channels[1]
    RightHori = channels[0]
    LeftSwitch = channels[6]
    #print(f"LeftVert={LeftVert}, LeftHori={LeftHori}")
    #print(f"RightVert={RightVert}, RightHori={RightHori}")
    #print(f"SwitchOn={LeftSwitch}")
    return LeftVert, RightHori, LeftSwitch
    
# Parse packet into coherent 'channel' list
def parsePacket(packet):
    channel = [-1] * 18
    channel[0] = (packet[2] << 8 & 0b0111_0000_0000) | packet[1]
    channel[1] = (packet[3] << 5 & 0b0111_1110_0000) | (packet[2] >> 3)
    channel[2] = (packet[5] << 10 & 0b0100_0000_0000) | (packet[4] << 2) | (packet[3] >> 6)
    channel[3] = (packet[6] << 7 & 0b0111_1000_0000) | (packet[5] >> 1)
    channel[4] = (packet[7] << 4 & 0b0111_1111_0000) | (packet[6] >> 4)
    channel[5] = (packet[9] << 9 & 0b0110_0000_0000) | (packet[8] << 1) | (packet[7] >> 7)
    channel[6] = (packet[10] << 6 & 0b0111_1100_0000) | (packet[9] >> 2)
    channel[7] = (packet[11] << 3) | (packet[10] >> 5)
    channel[8] = (packet[13] << 8 & 0b0111_0000_0000) | packet[12]
    channel[9] = (packet[14] << 5 & 0b0111_1110_0000) | (packet[13] >> 3)
    channel[10] = (packet[16] << 10 & 0b0100_0000_0000) | (packet[15] << 2) | (packet[14] >> 6)
    channel[11] = (packet[17] << 7 & 0b0111_1000_0000) | (packet[16] >> 1)
    channel[12] = (packet[18] << 4 & 0b0111_1111_0000) | (packet[17] >> 4)
    channel[13] = (packet[20] << 9 & 0b0110_0000_0000) | (packet[19] << 1) | (packet[18] >> 7)
    channel[14] = (packet[21] << 6 & 0b0111_1100_0000) | (packet[20] >> 2)
    channel[15] = (packet[22] << 3) | (packet[21] >> 5)
    channel[16] = packet[23] & 0b00000001
    channel[17] = packet[23] & 0b00000010

    frame_lost = bool(packet[23] & 0b00000100)
    failsafe = bool(packet[23] & 0b00001000)
    
    return channel, frame_lost, failsafe


class ServoTarget:
    def __init__(self):
        self._lock = threading.Lock()
        self.A = 0.0
        self.B = 0.0

    def set_AB(self, arr):
        with self._lock:
            if len(arr) >= 1: self.A = float(arr[0])
            if len(arr) >= 2: self.B = float(arr[1])
        #print(f"Shared set to {self.A} and {self.B}")

    def get_AB(self):
        with self._lock:
            return self.A, self.B

class ROS_Subscriber_Bringup:
    def __init__(self, shared_targets: ServoTarget, node_name="Servo_subscriber"):
        self.node_name = node_name
        self.shared = shared_targets
        self.ready = threading.Event()
        self._thread = threading.Thread(target=self._wait_and_init, daemon=True)
        self._thread.start()

    def _cmd_callback(self, msg: Float32MultiArray):
        # Update the shared object from ROS callback
        if not msg.data:
            print("No Msg Data")
            return
        print("Set shared data")
        self.shared.set_AB(msg.data)

    def _wait_and_init(self):
        backoff = 2
        while True:
            while not rospy.core.is_shutdown():
                try:
                    print("ROS initializing...")
                    # If you're in a worker thread, disable signal handlers:
                    rospy.init_node(self.node_name, anonymous=True, disable_signals=True)

                    # Double-check that rospy thinks it's initialized
                    if not rospy.core.is_initialized():
                        raise RuntimeError("rospy not initialized after init_node")

                    sub = rospy.Subscriber("/cmd/servo", Float32MultiArray, self._cmd_callback, queue_size=10)
                    rospy.loginfo("ROS is up. Subscribed to /cmd/servo (std_msgs/Float32MultiArray).")

                    # Keep this thread alive for callbacks
                    while not rospy.is_shutdown():
                        sleep(0.1)
                    return

                except Exception as e:
                    print(f"ROS initialization failed, retrying: {e}")
            sleep(1.0)  # small backoff before retry
            print("ROScore Not found")
            sleep(backoff)


class Servo:

    def __init__(self, shared_targets: ServoTarget):
        self.A = 0
        self.B = 0
        self.period = 40

        # Servo 1 : Neutral at 1475us, 10deg / 100us
        self.servo1 = AngularServo(SERVO_GPIO_PIN_1, min_angle=-50, max_angle=50, min_pulse_width=0.000975, max_pulse_width=0.001975)
        self.servo1.source_delay = SERVO_UPDATE_RATE_1
        self.servo1.source = self.sin_values(phase_lag=0)

        # Servo 2 : Neutral at 1400us, 10deg / 100us
        self.servo2 = AngularServo(SERVO_GPIO_PIN_2, min_angle=-50, max_angle=50, min_pulse_width=0.0009, max_pulse_width=0.0019)
        self.servo2.source_delay = SERVO_UPDATE_RATE_2
        self.servo2.source = self.sin_values(phase_lag=0) # Set the phase lag

        self.context = zmq.Context()
        #self.socket = self.context.socket(zmq.SUB)
        #self.socket.connect("tcp://localhost:7777")
        #self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
        self.shared = shared_targets
        #self.latest_packet = shared_targets.get_AB()

        self.rf_socket = self.context.socket(zmq.SUB)
        self.rf_socket.connect("tcp://localhost:5555")
        self.rf_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        print("Starting SBUS subscription...")
        self.latest_rfpacket = [0]*26
        sleep(0.5)

        self.running = True
        self.thread = threading.Thread(target=self.update, daemon=True)

        self.channels = None
        

    def update(self):

        while self.running:
            #while True:
            #    try:
            #        self.latest_packet = self.socket.recv_pyobj(flags=zmq.NOBLOCK)
            #    except zmq.Again:
            #        #print("This is Again")
            #        break

            while True:
                try:
                    self.latest_rfpacket = self.rf_socket.recv_pyobj(flags=zmq.NOBLOCK)
                except zmq.Again:
                    #print("This is Again2")
                    break
            #print("new try")
            #print(self.latest_packet) # this works
            #print(self.latest_rfpacket) # Not this.

            if self.latest_rfpacket is not None:
                #print(self.latest_rfpacket)
                channels, frame_lost, failsafe = parsePacket(self.latest_rfpacket)
                #print("Trasmit input")
                if failsafe:
                    #print("Transmitter Connection LOST - Failsafe Activated!")
                    self.A = 0
                    self.B = 0
                elif frame_lost:
                    #print("Transmitter Connection unstable - Frame Lost detected!")
                    self.A = 0
                    self.B = 0
                else:
                    #print("Transmitter Connected")
                    if channels[0] != 0:
                        self.channels = channels_2_dir(channels)
                        #print(self.channels[2])
                        if self.channels[2] > 0:
                            #self.A = self.latest_packet[0]
                            #self.B = self.latest_packet[1]
                            self.A, self.B = self.shared.get_AB()
                        else:
                            self.A = self.channels[0]/20
                            self.B = self.channels[1]/20
                        
                        #print(f"Updated value to A: {self.A}, B:{self.B}")
            
            sleep(0.01)

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def sin_values(self, phase_lag):
        angles = (2 * pi * i / self.period for i in range(self.period))
        
        for angle in cycle(angles):
            yield max(-0.99, min(self.A*sin(angle - phase_lag) - self.B, 0.99))


##### End #####
target = ServoTarget()
ros = ROS_Subscriber_Bringup(target)

servo = Servo(target)
servo.start()

try:
    while True:
        sleep(1)

except KeyboardInterrupt:
    servo.stop()
    
finally:
    servo.stop()

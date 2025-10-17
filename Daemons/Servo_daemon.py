# Execute source /opt/ros/noetic/setup.bash before running this code

# Set Pin Factory to pigpio https://gpiozero.readthedocs.io/en/stable/api_pins.html#changing-the-pin-factory
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Device, OutputDevice, AngularServo

Device.pin_factory = PiGPIOFactory()

##### Taken from gpiozero source #####
from time import sleep, monotonic
from itertools import cycle
from math import sin, cos, pi

import rospy
from std_msgs.msg import Float64MultiArray

import zmq
import threading


SERVOS_POWER_PIN = 27   # Power switch pin
SERVO_GPIO_PIN_0 = 17   # Front servo pin
SERVO_GPIO_PIN_1 = 12   # Servo 1 PWM pin
SERVO_GPIO_PIN_2 = 13   # Servo 2 PWM pin

SERVO_UPDATE_RATE = 0.1 # How frequent does the servo angle is updated (seconds)
RF_UPDATE_RATE = 0.01

BLADDER_NEUTRAL_POSITION = 0.0 # Between -10 and 10
BLADDER_CONTROL_GAIN = [0.2, 0.0, 0.1] # P, I, D gain
TARGET_PITCH = 0.0

TAIL_SERVO_PERIOD = 5
FISH_STATE_TIMEOUT_SEC = 0.2  # /Fish_state가 이 시간 이상 끊기면 /Fish_data 중단

Servos_power_switch = OutputDevice(SERVOS_POWER_PIN)


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
    RightSwitch = channels[4]
    RightDial = channels[5]
    return LeftVert, RightHori, LeftSwitch, RightSwitch, RightDial
    
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
        self.pitch = 0.0
        self.state = [0.0]*6
        self.mode = None
        self.pub_data = None
        self._last_state_walltime = None

    def set_AB(self, arr):
        with self._lock:
            if len(arr) >= 1: self.A = float(arr[0])
            if len(arr) >= 2: self.B = float(arr[1])

    def get_AB(self):
        with self._lock:
            return self.A, self.B

    def set_state(self, arr):
        # arr: [x, y, z, roll, pitch, yaw]
        if not arr or len(arr) != 6:
            return
        with self._lock:
            self.state = list(arr)
            self.pitch = float(arr[4])
            self._last_state_walltime = monotonic()
    
    def get_state(self):
        with self._lock:
            return self.state
    
    def _state_is_fresh(self, timeout=FISH_STATE_TIMEOUT_SEC):
        t = self._last_state_walltime
        return (t is not None) and ((monotonic() - t) <= timeout)

    def get_pitch(self):
        with self._lock:
            return self.pitch
    
    def set_mode(self, mode):
        with self._lock:
            self.mode = mode
    
    def get_mode(self):
        with self._lock:
            return self.mode
    
    def publish_data(self):
        if self.pub_data is None:
            return
        
        if self._state_is_fresh():
            A, B = self.get_AB()
            mode = self.get_mode()

            # RC=1.0, PC=0.0, Failsafe=-1.0
            if mode == 'RC':
                mode_val = 1.0
            elif mode == 'PC':
                mode_val = 0.0
            else:
                mode_val = -1.0
            
            msg = Float64MultiArray()
            msg.data = [rospy.get_time()] + self.get_state() + [A, B, mode_val]

            self.pub_data.publish(msg)


class ROS_Bringup:
    def __init__(self, shared_targets: ServoTarget, node_name="Servo_subscriber"):
        self.node_name = node_name
        self.shared = shared_targets
        self.ready = threading.Event()
        self._thread = threading.Thread(target=self._wait_and_init, daemon=True)
        self._thread.start()

    def _cmd_callback(self, msg: Float64MultiArray):
        if not msg.data:
            print("No Msg Data (/cmd/servo)")
            return
        
        self.shared.set_AB(msg.data)

    def _state_callback(self, msg: Float64MultiArray):
        if not msg.data:
            print("No Msg Data (/Fish_state)")
            return
        
        self.shared.set_state(msg.data)

    def _wait_and_init(self):
        backoff = 2
        while True:
            while not rospy.core.is_shutdown():
                try:
                    print("ROS initializing...")
                    rospy.init_node(self.node_name, anonymous=True, disable_signals=True)

                    if not rospy.core.is_initialized():
                        raise RuntimeError("rospy not initialized after init_node")

                    sub_cmd = rospy.Subscriber("/cmd/servo", Float64MultiArray, self._cmd_callback, queue_size=10)
                    sub_state = rospy.Subscriber("/Fish_state", Float64MultiArray, self._state_callback, queue_size=10)
                    pub_data = rospy.Publisher("/Fish_data", Float64MultiArray, queue_size=10)
                    self.shared.pub_data = pub_data
                    rospy.loginfo("ROS is up. Subscribed to /cmd/servo and /Fish_state (std_msgs/Float64MultiArray).")

                    while not rospy.is_shutdown():
                        sleep(0.1)
                    return
                except Exception as e:
                    print(f"ROS initialization failed, retrying: {e}")
            sleep(1.0)
            print("ROScore Not found")
            sleep(backoff)


class Servo:
    def __init__(self, shared_targets: ServoTarget):
        self.A = 0
        self.B = 0
        self.bladder = BLADDER_NEUTRAL_POSITION
        self.K_p, self.K_i, self.K_d = BLADDER_CONTROL_GAIN
        self.period = TAIL_SERVO_PERIOD

        self.shared = shared_targets

        # Servo 0 (Front servo) : Neutral at 1500us
        self.servo0 = AngularServo(SERVO_GPIO_PIN_0, min_angle=-50, max_angle=50, min_pulse_width=0.001, max_pulse_width=0.002)
        self.servo0_CPG = self.bladder_values()

        # Servo 1 : Neutral at 1475us, 10deg / 100us
        self.servo1 = AngularServo(SERVO_GPIO_PIN_1, min_angle=-50, max_angle=50, min_pulse_width=0.000975, max_pulse_width=0.001975)
        self.servo1_CPG = self.sin_values(phase_lag=0)

        # Servo 2 : Neutral at 1400us, 10deg / 100us
        self.servo2 = AngularServo(SERVO_GPIO_PIN_2, min_angle=-50, max_angle=50, min_pulse_width=0.0009, max_pulse_width=0.0019)
        self.servo2_CPG = self.sin_values(phase_lag=0)

        self.context = zmq.Context()
        self.rf_socket = self.context.socket(zmq.SUB)
        self.rf_socket.connect("tcp://localhost:5555")
        self.rf_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        print("Starting SBUS subscription...")
        self.latest_rfpacket = [0]*26
        sleep(0.5)

        self.running = True
        self.rf_thread = threading.Thread(target=self.update_command, daemon=True)
        self.servo_thread = threading.Thread(target=self.update_servo, daemon=True)

        self.channels = None

        self.last_pitch_error = 0.0
        self.last_time = None

    def update_command(self):
        next_tick = monotonic()
        while self.running:
            next_tick += RF_UPDATE_RATE
            while True:
                try:
                    self.latest_rfpacket = self.rf_socket.recv_pyobj(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break

            if self.latest_rfpacket is not None:
                channels, frame_lost, failsafe = parsePacket(self.latest_rfpacket)
                if failsafe and Servos_power_switch.is_active: Servos_power_switch.off()
                if failsafe or frame_lost:
                    # print("Transmitter Connection LOST - Failsafe Activated!")
                    self.A = 0
                    self.B = 0
                    self.shared.set_AB([self.A, self.B])
                    self.bladder = BLADDER_NEUTRAL_POSITION
                    self.shared.set_mode(None)
                else:
                    if channels[0] != 0:
                        if not Servos_power_switch.is_active: Servos_power_switch.on()
                        # print("Transmitter Connected")
                        self.channels = channels_2_dir(channels)
                        if self.channels[2] > 0:
                            self.A, self.B = self.shared.get_AB()
                            self.shared.set_mode('PC')
                        else:
                            self.A = self.channels[0]/20
                            self.B = self.channels[1]/20
                            self.shared.set_mode('RC')
                            self.shared.set_AB([self.A, self.B])
                        
                        if self.channels[3] > 0:
                            self.bladder = self.pitch_control()
                            # print(f'Auto control mode: Bladder {self.bladder}')
                        elif self.channels[3] == 0:
                            self.bladder = BLADDER_NEUTRAL_POSITION
                            # print(f'Neutral mode: Bladder {self.bladder}')
                        else:
                            self.bladder = self.channels[4]
                            # print(f'Manual control mode: Bladder {self.bladder}')

            sleep_time = next_tick - monotonic()
            if sleep_time > 0:
                sleep(sleep_time)
            else:
                next_tick = monotonic()

    def update_servo(self):
        next_tick = monotonic()
        while self.running:
            next_tick += SERVO_UPDATE_RATE

            self.shared.publish_data()
            self.servo0.value = next(self.servo0_CPG)
            self.servo1.value = next(self.servo1_CPG)
            self.servo2.value = next(self.servo2_CPG)

            sleep_time = next_tick - monotonic()
            if sleep_time > 0:
                sleep(sleep_time)
            else:
                next_tick = monotonic()

    def start(self):
        self.rf_thread.start()
        self.servo_thread.start()

    def stop(self):
        self.running = False
        self.rf_thread.join()
        self.servo_thread.join()

    def sin_values(self, phase_lag):
        angles = (2 * pi * i / self.period for i in range(self.period))
        for angle in cycle(angles):
            yield max(-0.99, min(self.A*sin(angle - phase_lag) - self.B, 0.99))
    
    def bladder_values(self):
        while True:
            bladder = max(-10, min(self.bladder, 10))
            yield 0.6 * bladder / 10.0

    # (Optional) keep your original sweep around for testing
    def bladder_test(self):
        while True:
            for ang in range(-100, 101, 1):
                yield 0.6 * ang / 100.0
            for ang in range(100, -101, -1):
                yield 0.6 * ang / 100.0

    def pitch_control(self):
        pitch = self.shared.get_pitch()

        # PD control to drive pitch to 0 deg
        error = -pitch
        now = monotonic()

        if self.last_time is None:
            dt = 0.0
            d_error = 0.0
        else:
            # protect against zero / negative dt
            dt = max(now - self.last_time, 0)
            d_error = (error - self.last_pitch_error) / dt if dt > 0 else 0.0

        u = BLADDER_NEUTRAL_POSITION - (self.K_p * error) - (self.K_d * d_error)

        # remember for next step
        self.last_pitch_error = error
        self.last_time = now

        return round(u, 1)
    


##### End #####
target = ServoTarget()
ros = ROS_Bringup(target)

servo = Servo(target)
servo.start()

try:
    while True:
        sleep(1)

except KeyboardInterrupt:
    servo.stop()
    
finally:
    servo.stop()

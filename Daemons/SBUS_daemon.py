# Need to install pyzmq using pip3 install pyzmq

import serial
import threading
import zmq
import time
import traceback

PORT = "/dev/ttyAMA0"  # Check the serial port

class SBUS:

    def __init__(self):
        self.thread = threading.Thread(target=self.__sbusThread)
        self.thread.daemon = True
        self.frame_lost = False
        self.failsafe = False
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")  # Binding to all interfaces on port 5555
        self.packet = [0] * 25
        self.running = True

    def start(self):
        self.thread.start()
    
    def stop(self):
        self.running = False
        self.thread.join()

    def __sbusThread(self):
        with serial.Serial(PORT, 100_000, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO) as ser:
            ser.reset_input_buffer()
            while self.running:
                try:
                    b = int.from_bytes(ser.read(), byteorder='little')
                    if b == 0x0F:
                        self.packet[0] = b
                        for i in range(24):
                            self.packet[i+1] = int.from_bytes(ser.read(), byteorder='little')
                        
                        # Publish the packet
                        self.socket.send_pyobj(self.packet)
                
                except Exception as e:
                    print("Error in RC receiver thread: ", e)
                    traceback.print_exc()
                    time.sleep(1)


if __name__ == "__main__":
    sbus = SBUS()
    sbus.start()
    while True:
        time.sleep(1)  # Keep the daemon alive

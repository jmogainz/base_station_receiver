from pymavlink import mavutil
import serial
from pyrtcm import RTCMReader
import multiprocessing
import time

def handle_rtcm_data(queue):
    # send rtcm_msg over uart to /dev/ttyUSB1
    uart_rtcm = serial.Serial('/dev/ttyTHS1', 460800, timeout=5)
    uart_rtcm.bytesize = serial.EIGHTBITS
    uart_rtcm.parity = serial.PARITY_NONE
    uart_rtcm.stopbits = serial.STOPBITS_ONE
    while True:
        # process a message and store data
        rtcm_msg = queue.get()
        rtcm_data = rtcm_msg.data
        rtcm_raw = bytes(rtcm_data[0:rtcm_msg.len])
        parsed = RTCMReader.parse(rtcm_raw)
        print(parsed)
        output = parsed.serialize()
        # print(rtcm_raw)
        print(len(rtcm_raw))
        # print(len(output))

        uart_rtcm.write(rtcm_raw)
        time.sleep(1)
        # print(uart_rtcm.out_waiting)

queue = multiprocessing.Queue()
rtcm_process = multiprocessing.Process(target=handle_rtcm_data, args=(queue,))
rtcm_process.start()

master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

while True:
    msg = master.recv_match(blocking=False)

    try:
        print(f"Received message of type {msg.get_type()}")
    except:
        continue

    if msg.get_type() != 'BAD_DATA':
        if msg.get_type() == 'GPS_RTCM_DATA':
            # handle rtcm data in separate process so that it does not block
            queue.put(msg)
            break




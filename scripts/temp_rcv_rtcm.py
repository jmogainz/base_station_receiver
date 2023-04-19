from pymavlink import mavutil
import serial
from pyrtcm import RTCMReader
import multiprocessing
import time

def handle_rtcm_data(queue):
    uart_rtcm = serial.Serial('/dev/ttyTHS1', 38400)
    uart_rtcm.bytesize = serial.EIGHTBITS
    uart_rtcm.parity = serial.PARITY_NONE
    uart_rtcm.stopbits = serial.STOPBITS_ONE
    time.sleep(1) # let the port initialize
    while True:
        rtcm_msg = queue.get()
        rtcm_data = rtcm_msg.data
        rtcm_raw = bytes(rtcm_data[0:rtcm_msg.len])
        # open rtcm text stream
        # stream = open('rtcm_sample.txt', 'rb')
        # rtr = RTCMReader(stream)
        # for (raw_data, parsed_data) in rtr:
        #     print(parsed_data)
        #     # test header of 5 bytes
        #     test_data = "hello"
        #     test_data = bytes(test_data, 'utf-8')
        #     written = uart_rtcm.write(raw_data)
        #     print("bytes", len(raw_data))
        #     print("bytes written: ", written, "\n")
        #     time.sleep(1)
        parsed = RTCMReader.parse(rtcm_raw)
        print(parsed)
        # print(rtcm_raw)
        print(len(rtcm_raw), "\n")

        uart_rtcm.write(rtcm_raw)
        time.sleep(1)
        # break
        # print(uart_rtcm.out_waiting)

queue = multiprocessing.Queue()
rtcm_process = multiprocessing.Process(target=handle_rtcm_data, args=(queue,))
rtcm_process.start()

master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

while True:
    msg = master.recv_match(blocking=False)

    try:
        type = msg.get_type()
    except:
        continue

    if msg.get_type() != 'BAD_DATA':
        if msg.get_type() == 'GPS_RTCM_DATA':
            # handle rtcm data in separate process so that it does not block
            queue.put(msg)
            



import time
import can
import os

interface = 'socketcan'
channel = 'can0'

def producer(id):
    """:param id: Spam the bus with messages including the data id."""
    os.system( 'sudo /sbin/ip link set can0 down' )
    os.system( 'sudo /sbin/ip link set can0 up type can bitrate 1000000' )
    bus = can.Bus(channel=channel, interface=interface)
    bus.flush_tx_buffer()
    for i in range(10):
        msg = can.Message(arbitration_id=0xc0ffee, data=[id, i, 0, 1, 3, 1, 4, 1], is_extended_id=False)
        bus.send(msg)

    time.sleep(1)

producer(10)
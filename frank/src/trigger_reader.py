#! /usr/bin/env python3
import rospy
#import numpy as np
from std_msgs.msg import Int32, Float64, Bool, String
from serial import Serial
from fcntl import  ioctl
from termios import (
    TIOCMIWAIT,
    TIOCM_RNG,
    TIOCM_DSR,
    TIOCM_CD,
    TIOCM_CTS
)

ser = Serial('/dev/ttyS0')


wait_signals = (TIOCM_RNG |
                TIOCM_DSR |
                TIOCM_CD  |
                TIOCM_CTS)

tim1 = 0
summ= 0
count = 0
first = 0
if __name__ == '__main__':

    try:
        rospy.init_node('trigger_reader')
        trigger_pub = rospy.Publisher("trigger_State", Float64, queue_size=1)
        print("ready to serve")
        while True:
            tim_old = tim1
            ioctl(ser.fd, TIOCMIWAIT, wait_signals)
	    if first==0:
		tim1 = rospy.get_time()
	        delta_t = 1/abs(tim1-tim_old)
	        summ+=delta_t
	        count+=1
		first =1
	    if first==1:
		first=0
            # print('RI=%-5s - DSR=%-5s - CD=%-5s - CTS=%-5s',
            #     ser.getRI(),
            #     ser.getDSR(),
            #     ser.getCD(),
            #     ser.getCTS(),
            # )
            trigger_pup.publish(tim1) #publish the timestamp of the recived trigger
            if count == 100:
                print(summ/100)
                count = 0
                summ = 0
            #print(delta_t)
        rospy.spin()
    except  rospy.ROSInterruptException:
        pass


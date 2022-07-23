#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import Image
import sys
from pathlib import Path
include_dir = str(Path(__file__).resolve().parent.parent)
sys.path.append(include_dir)
print(include_dir)
from include import FrankCamera
from pathlib import Path
utils_dir = str(Path(__file__).resolve().parent.parent.parent)
sys.path.append(utils_dir)
#from cv_bridge import CvBridge, CvBridgeError
from include.cv_bridge_my import *
import cv2
import time
from frank.srv import *
#from std_srvs.srv import Trigger

def connect_2d_camera(req):
    print("... Connecting to 2D Camera ...")
    res = camera.Connect()
    response = Connect2DCameraResponse()
    if res<0:
        print('...error connecting to camera...')
        response.res = 0
        response.error = "error connecting to camera"
        return response
    else:
        print('...2d camera connected...')
        response.res = 1
        response.error = "connected successfully to camera"
        return response

def disconnect_2d_camera(req):
    print("... Disconnecting from 2D Camera ...")
    res = camera.Disconnect()
    response = Disconnect2DCameraResponse()
    if res<0:
        print('...error disconnecting from camera...')
        response.res = 0
        response.error = "error disconnecting from camera"
        return response
    else:
        print('...2d camera disconnected...')
        response.res = 1
        response.error = "disconnected successfully from camera"
        return response
def grab_2dimage(req):
    print('...capturing 2D photo...')
    ExposureTime = 300000
    GainValue = 3000
    time.sleep(0.2)
    res = camera.CaptureFrame(ExposureTime, GainValue)
    response = Grab2DImageResponse()
    if type(res)!= int:
        print("frame capured with success")
        img_msg= cv2_to_imgmsg(res)
        response.image = img_msg
        camera_pub.publish(img_msg)
        response.error = 'image captured successfully'
        response.res = 1
        return response
    else:
        print('...error capturing frame...')
        response.error = "error capturing image"
        response.res = 0
        return response


### check better the logic of the


if __name__ == '__main__':

    try:
        camera = FrankCamera.newCamera()
        rospy.init_node("camera_2d_node")
        s_conn = rospy.Service("connect_2dcamera", Connect2DCamera, connect_2d_camera)
        s_disconn = rospy.Service("disconnect_2dcamera", Disconnect2DCamera, disconnect_2d_camera)
        camera_pub = rospy.Publisher("camera2d_image",Image, queue_size=1)
        s_grab = rospy.Service("grab_2dimage", Grab2DImage, grab_2dimage)
        print("ready to serve")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
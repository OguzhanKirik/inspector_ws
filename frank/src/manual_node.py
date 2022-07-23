#! /usr/bin/env python3
import rospy
import sys
from os.path import dirname, abspath
from frank.srv import *
from frank.msg import *
include_dir = str(dirname(dirname(abspath(__file__)))) + '/include/'
sys.path.append(include_dir)
import abb


def go_to_position(req):
    print("received GoToPose request")
    response = GoToPoseResponse()
    # coord= req.pose[0:3]
    # quat = req.pose[3:-1]
    msg = robot.set_cartesian(req.pose)
    print(msg)
    print(len(msg))
    man_req =RapidCommandRequest()
    man_req.msg = msg
    resp = man_cmd(man_req)
    print(resp)
    response.resp = str(resp)
    return response

def get_position(req):
    response = GetPoseResponse()
    msg = robot.get_cartesian()
    print(msg)
    man_req = RapidCommandRequest()
    man_req.msg = msg
    resp = man_cmd(man_req)
    print(resp)
    #response.value = 
    data = resp.resp.split("_")
    if len(data) == 9:
        r = [float(s) for s in data]
        response.pose = r
        return response
    else:
        if len(data) == 12:
            r = [0,0,0,0,0,0,0]
            r[0] = float(data[5])
            r[1] = float(data[6])
            r[2] = float(data[7])
    
            r[0] = float(data[8])
            r[1] = float(data[9])
            r[2] = float(data[10])
            r[3] = float(data[11])

            response.pose = r

            return response
    
    

if __name__ == '__main__':

    try:
        rospy.init_node('manual_node')
        robot = abb.Robot()
        s_GoTo = rospy.Service("GoToPose", GoToPose, go_to_position)
        s_GetPos = rospy.Service("GetPose", GetPose, get_position)

        man_cmd= rospy.ServiceProxy("/manual_command", RapidCommand)
        print("Manual rapid control is ready")

        rospy.spin()
    except  rospy.ROSInterruptException:
        pass




#! /usr/bin/env python3
import pyads
from pyads.structs import SAdsSymbolUploadInfo
import rospy
import sys
from std_msgs.msg import Int32
from os.path import dirname, abspath
include_dir =str(dirname(dirname(abspath(__file__)))) + '/include/'
#print(include_dir)
sys.path.append(include_dir)
#print(sys.path)
import FrankPLC
from frank.srv import *
print("ciao")

def command_plc(req):
    command = req.command
    response = CommandPLCResponse()
    value = int(req.value)
    print(value)
    if PLC.Connected == False:
        response.res = 0
        response.error = 'PLC IS NOT CONNECTED'
        print(response.error)
        return response
    else:
        try:
            if command=="GVL_ROS.PL_ProfilTrig_D":
                typo= pyads.PLCTYPE_UINT
                #PLC.plc.write_by_name("GVL_ROS.IRC5_WORD1_ST",2,pyads.PLCTYPE_WORD)
                PLC.plc.write_by_name("GVL_ROS.PL_ProfilTrig_P",PLC.TriggerFreq,typo) # for the triggering action I have to send alway 2 command, the last is the duty cycle
                PLC.plc.write_by_name(command,value,typo)
                msg = Int32()
                if value!=0:
                    bol = 1
                else: bol = 0
                msg.data = int(bol)
                pub_trig.publish(msg)
            elif command == "GVL_ROS.IRC5_WORD1_ST" or command =="GVL_ROS.IRC5_ParteProgrammaAct":
                typo = pyads.PLCTYPE_WORD
                if command =="GVL_ROS.IRC5_ParteProgrammaAct":
                    PLC.plc.write_by_name("GVL_ROS.IRC5_WORD1_ST",2,typo)
                    print("PLC SET IN AUTOMATIC MODE")  #deve mandarlo in automatico
                    PLC.plc.write_by_name(command,value,typo)
                    print("PLC STARTING AUTOMATIC PROCEDURE")
                    print("number : " + str(value))  

                else:
                    PLC.plc.write_by_name(command,value,typo)
                #lancia la
            elif command=="GVL_ROS.EM_StopRot" or command== "GVL_ROS.EM_StartRot" or command == "GVL_ROS.IRC5_Rst":
                typo = pyads.PLCTYPE_BOOL
                PLC.plc.write_by_name(command,value,typo)  
            elif command == "GVL_ROS.EM_SetpointSpeed":
                typo = pyads.PLCTYPE_REAL
                if value==0:
                    #stop rotation
                    #set to automatic
                    PLC.plc.write_by_name("GVL_ROS.IRC5_WORD1_ST",2,pyads.PLCTYPE_WORD)
                    procedure2 = "GVL_ROS.EM_StartRot"   #  
                    valore2 = 0
                    PLC.plc.write_by_name(procedure2,valore2,pyads.PLCTYPE_BOOL)
                    procedure2 = "GVL_ROS.EM_StopRot"   #  
                    valore2 = 1
                    PLC.plc.write_by_name(procedure2,valore2,pyads.PLCTYPE_BOOL)
                elif value!=0:
                    #set rotation speed
                    PLC.plc.write_by_name("GVL_ROS.IRC5_WORD1_ST",2,pyads.PLCTYPE_WORD)
                    PLC.plc.write_by_name(command,value,typo)
                    #start rotation
                    procedure2 = "GVL_ROS.EM_StopRot"   #  
                    valore2 = 0
                    PLC.plc.write_by_name(procedure2,valore2,pyads.PLCTYPE_BOOL)
                    procedure2 = "GVL_ROS.EM_StartRot"   #  
                    valore2 = 1
                    PLC.plc.write_by_name(procedure2,valore2,pyads.PLCTYPE_BOOL)
                  
            response.stamp = rospy.get_rostime()
            response.error = "command send successfully"
            response.res=1
            print(response.error)
            return response
        except:
            response.res = 0
            response.error = "ERROR COMMUNICATION"
            response.stamp = rospy.get_rostime()
            print(response.error)
            return response


def read_plc(req):
    command = req.command
    response = ReadPLCResponse()
    if PLC.Connected == False:
        response.res = 0
        response.error = 'PLC IS NOT CONNECTED'
        print(response.error)
        return response
    else:
        try:
            if command in ["GVL_ROS.MPS_ToolBlocc", "GVL_ROS.MPS_ToolSbloc", "GVL_ROS.MPS_AllStTool",
                "GVL_ROS.EM_Agganciato", "GVL_ROS.TM_Agganciato", "GVL_ROS.TSM_PresenzaTool", "GVL_ROS.TSE_PresenzaTool"] :
                typo= pyads.PLCTYPE_BOOL
                #PLC.plc.write_by_name("GVL_ROS.IRC5_WORD1_ST",2,pyads.PLCTYPE_WORD)
                result = PLC.plc.read_by_name(command,typo)
                if result==True:
                    response.value =1
                elif result ==False:
                    response.value = 0
            if command in ["GVL_ROS.EM_RotSpeed"]:
                typo= pyads.PLCTYPE_LREAL
                result = PLC.plc.read_by_name(command,typo)
                response.value = float(result)
            #response.stamp = rospy.get_rostime()
            response.error = "command send successfully"
            response.res=1
            print(response.error)
            return response
        except:
            response.res = 0
            response.error = "ERROR COMMUNICATION"
            #response.stamp = rospy.get_rostime()
            print(response.error)
            return response

    #check connection3
    
  
        
def connect_plc(req):
    response = ConnectPLCResponse()
    if PLC.Connected == True:
        response.res = 2
        response.error = "PLC is already connected"
        print(response.error)
    else:
        print ("=== add route to plc")
        pyads.add_route_to_plc (PLC.SENDER_AMS, PLC.HOSTNAME, PLC.PLC_IP, PLC.PLC_USERNAME, PLC.PLC_PASSWORD, route_name = PLC.ROUTE_NAME)
        pyads.open_port ()
        pyads.set_local_address (PLC.SENDER_AMS)
        print (pyads.get_local_address ())
        pyads.close_port ()
        print ("=== add route")
        pyads.open_port ()
        pyads.add_route (PLC.NET_ID , PLC.PLC_IP)
        pyads.close_port ()
        PLC.plc = pyads.Connection(PLC.NET_ID, pyads.PORT_TC3PLC1, PLC.PLC_IP)
        PLC.plc.open()
        try:
            print(PLC.plc.read_by_name('GVL_ROS.InfoPulsStart', pyads.PLCTYPE_BOOL)) #try to read a variable 
            PLC.Connected =True
            response.res = 1
            response.error = "connection with plc successfull"
            return response
        except:
            PLC.Connected = False
            response.res = 0
            response.error = "error connection with plc"
            return response

def disconnect_plc(req):
    try:
        print("1")
        PLC.plc.close()
        print("2")

        response = DisconnectPLCResponse()
        response.res = 1
        response.error = "plc disconnected"
        PLC.Connected = False
        return response
    except:
        response.res = 0
        response.error="error disconnecting plc"
        return response

def allarm_publish():
    typo= pyads.PLCTYPE_BOOL
    msg = Int32()
    #PLC.plc.write_by_name("GVL_ROS.IRC5_WORD1_ST",2,pyads.PLCTYPE_WORD)
    result = PLC.plc.read_by_name("GVL_ROS.PLC_All",typo)
    if result==True:
        value =1
    elif result ==False:
        value = 0
    msg.data = value
    pub_allarm.publish(msg)

def plc_tool_info(req):
    """
    return the tool currently mounted and if there are mounted any tool in the racks
    """
    response = PLCToolInfoResponse()
    try:
        typo= pyads.PLCTYPE_BOOL
        #PLC.plc.write_by_name("GVL_ROS.IRC5_WORD1_ST",2,pyads.PLCTYPE_WORD)
        response.spindle_mounted = PLC.plc.read_by_name("GVL_ROS.EM_Agganciato",typo)
        response.inspection_mounted = PLC.plc.read_by_name("GVL_ROS.TM_Agganciato",typo)
        response.rack_spindle_presence = PLC.plc.read_by_name("GVL_ROS.TSE_PresenzaTool",typo)
        response.rack_inspection_presence = PLC.plc.read_by_name("GVL_ROS.TSM_PresenzaTool",typo)
        response.error = "no error occurred"
        return response
    except:
        response.error = "error communicating with PLC"
        return response

if __name__ == '__main__':
    try:
        rospy.init_node("plc_node")
        PLC = FrankPLC.newPLC()
        s_plc_comm = rospy.Service("plc_command", CommandPLC, command_plc)
        s_plc_read = rospy.Service("plc_read", ReadPLC, read_plc)
        s_plc_conn = rospy.Service("plc_connect", ConnectPLC, connect_plc)
        s_plc_disc = rospy.Service("plc_disconnect", DisconnectPLC, disconnect_plc)
        pub_trig = rospy.Publisher("trigger_state", Int32, queue_size=1 )
        pub_allarm = rospy.Publisher("allarm_state", Int32, queue_size=1 )
        s_plc_tool = rospy.Service("plc_tool_info",PLCToolInfo,plc_tool_info)
        print("PLC is ready to serve")
        # while rospy.ok():
        #     allarm_publish()
        #     rospy.sleep(0.2)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

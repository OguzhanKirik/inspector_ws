#include "abb_librws/rws_interface.h"
#include "abb_librws/rws_state_machine_interface.h"
#include "stdio.h"
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <cstdlib>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "math.h"
#include "frank/CommandPLC.h"
#include "frank/ConnectPLC.h"
#include "frank/DisconnectPLC.h"

#include "frank/Grab3DPointcloud.h"
#include "frank/Connect3DCamera.h"
#include "frank/Disconnect3DCamera.h"
#include "frank/SaveSparseScan.h"
#include "frank/SaveCalibScan.h"
#include "frank/ConnectProfilometer.h"
#include "frank/DisconnectProfilometer.h"
#include "frank/SparseAlignment.h"
#include "frank/OperationState.h"
#include "frank/RapidCommand.h"
#include "frank/RapidCommandRequest.h"
#include "frank/RapidCommandResponse.h"
#include "frank/CallRoutine.h"
#include "frank/CallRoutine.h"
#include "frank/CallRoutine.h"
///////////////////////////DRILLING CASE/////////////////
int Drill;
int DrillOp;
std::string DrillBit;
/////////////////////////////////////////////////////////

trajectory_msgs::MultiDOFJointTrajectoryPoint point_check;
bool new__point_check_received = false;

using namespace abb;
using namespace rws;
using namespace std;

void Reset_Signals();
void Reset_egm_stop();
void Reset_egm_start();
void Reset_RUN_RAPID();

void set_egm_stop();
void set_egm_start();
void set_RUN_RAPID();
void Restart_System();
bool manual_command_robot();


//Define rws constants
#define POS_TOLERANCE 0.00001
#define COMM_TIMEOUT 1
#define RAMP_IN_TIME 0.1///originally is 0.1-> is ramp_out time in rapid!!
#define RAMP_OUT_TIME 0.3///0.38
#define COND_TIME 5
#define COMM_DELAY 0.1 //0.1
#define PORT_REAL_ROBOT 80
#define PORT_ROBOT_STUDIO 80
//#define LP 70
// #define K 1
#define MAX_SPEED_DEV 200.0 //20
#define SAMPLE_RATE 4 //4
#define COND_MIN_MAX 0.00001  //0.1

#define RWS_START_DELAY 0


//const string IP_ADDRESS_ROBOT_STUDIO = "192.168.0.209";//ufficio
//const string IP_ADDRESS_ROBOT_STUDIO = "192.168.0.226";//ufficioLAN
//const string IP_ADDRESS_ROBOT_STUDIO = "192.168.0.169";//temp
//const string IP_ADDRESS_ROBOT_STUDIO = "192.168.1.101";//casa
const string IP_ADDRESS_ROBOT_STUDIO = "192.168.125.1"; //LMA

//INIT RWS INTERFACE
RWSStateMachineInterface Frank(IP_ADDRESS_ROBOT_STUDIO, PORT_ROBOT_STUDIO);
//RWSStateMachineInterface Frank(IP_ADDRESS_REAL_ROBOT, PORT_REAL_ROBOT);

const string TASK = SystemConstants::RAPID::TASK_ROB_1;
const string ROBOT = SystemConstants::General::MECHANICAL_UNIT_ROB_1;
const string SIGNAL_EGM_STOP = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::EGM_STOP;
const string SIGNAL_EGM_START = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::EGM_START_POSE;
const string SIGNAL_EGM_STARTJ = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::EGM_START_JOINT;

const string SIGNAL_RUN_RAPID = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::RUN_RAPID_ROUTINE;
const string HIGH = SystemConstants::IOSignals::HIGH;
const string LOW = SystemConstants::IOSignals::LOW;
boost::shared_ptr<const std_msgs::Int16> Num ;
std_msgs::Int16 num ;


int counter ;
int traj;
int egm_mode;
double nump;
boost::shared_ptr<const std_msgs::Int16> Cond ;

std_msgs::Int16 cond;
trajectory_msgs::MultiDOFJointTrajectoryPoint set_point;
bool new_point_check_received = false ;
sensor_msgs::JointState j_set_point;
bool new_jset_point_received = false;


std_msgs::Bool ready;
///Variables for Rapid Routines
abb::rws::RAPIDBool Proc_exec;
abb::rws::RAPIDString receivedString;  //string recieved by rapid
abb::rws::RAPIDString sendString;    //string sent back from rapid
std_msgs::Bool RRexec_res;

abb::rws::RAPIDBool PCAquired;
std_msgs::Bool pc_aquired;

abb::rws::RAPIDNum DbIn;
abb::rws::RAPIDNum DbOut;
int DbActual;
std::string DbActualString;

std::string rapid_routines_module = "TRobSystemExample";
std::string rapid_caseRoutine_module = "TRobSystem";
std::string call_RR;

////Tool reference system
double toolx;//=0.0 ;
double tooly;//=0.0 ;
double toolz;//=919.5;
double toolq1;//=1.0;
double toolq2;//= 0.0;
double toolq3;//=0.0;
double toolq4;//=0.0;
double toolmass;//=1.0;
double toolcogx;//=0.0;
double toolcogy;//=0.0;
double toolcogz;//=1.0;
double toolaom1;//=1.0;
double toolaom2;//=0.0;
double toolaom3;//=0.0;
double toolaom4;//=0.0;
double toolix;//=0.0;
double tooliy;//=0.0;
double tooliz;//=0.0;

////Work Object reference system
double woobjufx;//=-2170.543;
double woobjufy;//= 0.0;
double woobjufz;//= -6324.43;
double woobjufq1;//= 1.0;
double woobjufq2;//= 0.0;
double woobjufq3;//= 0.0;
double woobjufq4;//= 0.0;
double woobjofx;//=0.0;
double woobjofy;//= 0.0;
double woobjofz;//= 0.0;
double woobjofq1;//= 1.0;
double woobjofq2;//= 0.0;
double woobjofq3;//= 0.0;
double woobjofq4;//= 0.0;

void num_callback(const std_msgs::Int16::ConstPtr& msg)
{
    num.data = msg->data;
    
}


void pointcheck_callback(const trajectory_msgs::MultiDOFJointTrajectoryPoint &msg)
{
    point_check.transforms.operator=(msg.transforms);
    point_check.velocities.operator=(msg.velocities);
    new_point_check_received = true;
}
void j_setpoint_callback(const sensor_msgs::JointState &msg)
{
  j_set_point.position.operator=(msg.position);
  j_set_point.velocity.operator=(msg.velocity);
  new_jset_point_received = true;
}


void Reset_egm_stop()
{
    while (Frank.getIOSignal(SIGNAL_EGM_STOP) == HIGH)
    {
        ros::Duration(COMM_DELAY).sleep();
        Frank.setIOSignal(SIGNAL_EGM_STOP, LOW);
        ros::Duration(COMM_DELAY).sleep();
    }
}

void Reset_egm_start()
{
    while (Frank.getIOSignal(SIGNAL_EGM_START) == HIGH)
    {
        ros::Duration(COMM_DELAY).sleep();
        Frank.setIOSignal(SIGNAL_EGM_START, LOW);
        ros::Duration(COMM_DELAY).sleep();
    }
}
void Reset_egmj_start()
{
    while (Frank.getIOSignal(SIGNAL_EGM_STARTJ) == HIGH)
    {
        ros::Duration(COMM_DELAY).sleep();
        Frank.setIOSignal(SIGNAL_EGM_STARTJ, LOW);
        ros::Duration(COMM_DELAY).sleep();
    }
}

void Reset_RUN_RAPID()
{
    while (Frank.getIOSignal(SIGNAL_RUN_RAPID) == HIGH)
    {
        ros::Duration(COMM_DELAY).sleep();
        Frank.setIOSignal(SIGNAL_RUN_RAPID, LOW);
        ros::Duration(COMM_DELAY).sleep();
    }
}

void Set_egm_stop()
{
    while (Frank.getIOSignal(SIGNAL_EGM_STOP) == LOW)
    {
        ros::Duration(COMM_DELAY).sleep();
        Frank.setIOSignal(SIGNAL_EGM_STOP, HIGH);
        ros::Duration(COMM_DELAY).sleep();
    }
}

void Set_egm_start()
{    
    while (Frank.getIOSignal(SIGNAL_EGM_START) == LOW)
    {
        ros::Duration(COMM_DELAY).sleep();
        Frank.setIOSignal(SIGNAL_EGM_START, HIGH);
        ros::Duration(COMM_DELAY).sleep();
       
    }
}
void Set_egmj_start()
{    
    while (Frank.getIOSignal(SIGNAL_EGM_STARTJ) == LOW)
    {
        ros::Duration(COMM_DELAY).sleep();
        Frank.setIOSignal(SIGNAL_EGM_STARTJ, HIGH);
        ros::Duration(COMM_DELAY).sleep();
       
    }
}
void Set_RUN_RAPID()
{
    while (Frank.getIOSignal(SIGNAL_RUN_RAPID) == LOW)
    {
        ros::Duration(COMM_DELAY).sleep();
        Frank.setIOSignal(SIGNAL_RUN_RAPID, HIGH);
        ros::Duration(COMM_DELAY).sleep();
    }
}

void Restart_System()
{
    if (Frank.isRAPIDRunning().isTrue())
    {
        ros::Duration(COMM_DELAY).sleep();
        Frank.stopRAPIDExecution();
        ros::Duration(COMM_DELAY).sleep();
        Frank.resetRAPIDProgramPointer();
        ros::Duration(COMM_DELAY).sleep();
        Frank.startRAPIDExecution();
        ROS_INFO("Try to start Rapid");
        
    }
    else
    {
        ros::Duration(COMM_DELAY).sleep();
        Frank.resetRAPIDProgramPointer();
        ros::Duration(COMM_DELAY).sleep();
        if (Frank.isMotorsOn().isFalse())
            ros::Duration(COMM_DELAY).sleep();
            Frank.setMotorsOn();
        ros::Duration(COMM_DELAY).sleep();
        Frank.startRAPIDExecution();
        ROS_INFO("Try to start Rapid");
    }
}
void Reset_Signals()
{
    Reset_egm_start();
    ros::Duration(COMM_DELAY).sleep();
    Reset_egmj_start();
    ros::Duration(COMM_DELAY).sleep();
    Reset_egm_stop();
    
    //Reset_RUN_RAPID();
}
void START_IRC5()
{
    Reset_Signals();
    ros::Duration(COMM_DELAY).sleep();
    Restart_System();
    ros::Duration(COMM_DELAY).sleep();
    Restart_System();
    ros::Duration(COMM_DELAY).sleep();
    while (Frank.isRAPIDRunning().isFalse()){
        ros::Duration(COMM_DELAY).sleep();
        Frank.startRAPIDExecution();
        ros::Duration(COMM_DELAY).sleep();
    }
    ROS_INFO("Rapid Started"); 
}
void Call_Rapid_routine(std::string Routine_name)
{
    Proc_exec.value = false;
    Frank.setRAPIDSymbolData(TASK,rapid_routines_module,"Proc_exec",Proc_exec);
    ros::Duration(COMM_DELAY).sleep();
    Set_egm_stop();
    ros::Duration(COMM_DELAY).sleep();
    Reset_Signals(); 
    ros::Duration(COMM_DELAY).sleep();
    Frank.services().rapid().setRoutineName(TASK,Routine_name);
    ros::Duration(COMM_DELAY).sleep();
    Frank.services().rapid().signalRunRAPIDRoutine();
    ROS_INFO_STREAM(Routine_name +" execution...");
    ros::Duration(COMM_DELAY).sleep();
    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
    RRexec_res.data=Proc_exec.value;
    ros::Duration(COMM_DELAY).sleep();
    while (RRexec_res.data == false){
        Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
        RRexec_res.data=Proc_exec.value;
        ros::Duration(COMM_DELAY).sleep();
                      
    }
    //Add the turn off of RUNRAPIDROUTINE signal!!
    ROS_INFO_STREAM(Routine_name + " execution finished");
    ros::Duration(COMM_DELAY).sleep();
    Reset_RUN_RAPID(); 
}

bool manual_command_robot(frank::RapidCommand::Request &req, frank::RapidCommand::Response &res)
    {
        std::string Routine_name;
        Routine_name = "RapidControl";

        receivedString = req.msg; /// std::to_string(req.msg);
        // prepare the routine call
        Proc_exec.value = false;
        // set variable of precedure execution
        Frank.setRAPIDSymbolData(TASK,rapid_routines_module,"Proc_exec",Proc_exec);
        ros::Duration(0.1).sleep();
        Set_egm_stop();  //stop egm
        ros::Duration(0.1).sleep();
        Reset_Signals(); 
        ros::Duration(0.1).sleep();
        Frank.services().rapid().setRoutineName(TASK,Routine_name);
        Frank.setRAPIDSymbolData(TASK,rapid_routines_module,"receivedString",receivedString);
        ros::Duration(0.1).sleep();
        Frank.services().rapid().signalRunRAPIDRoutine();
        ROS_INFO_STREAM(Routine_name +" execution...");
        // wait for routine to be finished
        RRexec_res.data=Proc_exec.value;
        ros::Duration(0.1).sleep();
        while (RRexec_res.data == false){
            Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
            RRexec_res.data=Proc_exec.value;
            ros::Duration(0.1).sleep();     
        }
        Frank.getRAPIDSymbolData(TASK,rapid_routines_module,"sendString", &sendString);
        //std::cout << sendString.parseString << "\n";
        res.resp = sendString.value; //need to be parsed by the client
        ROS_INFO_STREAM(Routine_name + " execution finished");
        ros::Duration(0.1).sleep();
        Reset_RUN_RAPID(); 

        return true;
    }
bool call_routine(frank::CallRoutine::Request &req, frank::CallRoutine::Response &res)
{
    std::string Routine_name;
    Routine_name = req.name;
    Proc_exec.value = false;
    Frank.setRAPIDSymbolData(TASK,rapid_routines_module,"Proc_exec",Proc_exec);
    ros::Duration(COMM_DELAY).sleep();
    Set_egm_stop();
    ros::Duration(COMM_DELAY).sleep();
    Reset_Signals(); 
    ros::Duration(COMM_DELAY).sleep();
    Frank.services().rapid().setRoutineName(TASK,Routine_name);
    ros::Duration(COMM_DELAY).sleep();
    Frank.services().rapid().signalRunRAPIDRoutine();
    ROS_INFO_STREAM(Routine_name +" execution...");
    ros::Duration(COMM_DELAY).sleep();
    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
    RRexec_res.data=Proc_exec.value;
    ros::Duration(COMM_DELAY).sleep();
    while (RRexec_res.data == false){
        Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
        RRexec_res.data=Proc_exec.value;
        ros::Duration(COMM_DELAY).sleep();
                      
    }
    //Add the turn off of RUNRAPIDROUTINE signal!!
    ROS_INFO_STREAM(Routine_name + " execution finished");
    ros::Duration(COMM_DELAY).sleep();
    Reset_RUN_RAPID(); 
    return true;
}

int main(int argc, char **argv)
{ //init node
    ros::init(argc, argv, "rws_node");
    ros::NodeHandle node_handle;
    ros::ServiceServer ManualServ = node_handle.advertiseService("/manual_command", manual_command_robot);
    ros::ServiceServer routineServ = node_handle.advertiseService("/call_routine", call_routine);
    //put here sub o pub declaration
   
    ros::Subscriber num_sub = node_handle.subscribe("/num_to_check", 1000, num_callback);

    ros::Publisher State_pub= node_handle.advertise<frank::OperationState>("/operation_state", 1);
    frank::OperationState OpState;
    
    ros::Publisher feedback_publisher = node_handle.advertise<std_msgs::Bool>("/rws_ready", 1); //initially 1000
    //////SERVICES FOR PLC
    ros::ServiceClient plc_client=node_handle.serviceClient<frank::CommandPLC>("plc_command");
    frank::CommandPLC plc_comm;
    ros::ServiceClient plc_conn=node_handle.serviceClient<frank::ConnectPLC>("plc_connect");
    frank::ConnectPLC plc_connect;
    ros::ServiceClient plc_disconn = node_handle.serviceClient<frank::DisconnectPLC>("plc_disconnect");
    frank::DisconnectPLC plc_disconnect;
    //////SERVICES FOR 3D CAM
    ros::ServiceClient cam3d_client = node_handle.serviceClient<frank::Grab3DPointcloud>("grab_3dpointcloud");
    frank::Grab3DPointcloud cam3d_comm;
    ros::ServiceClient cam3d_conn = node_handle.serviceClient<frank::Connect3DCamera>("connect_3dcamera");
    frank::Connect3DCamera cam3d_connect;
    ros::ServiceClient cam3d_disconn = node_handle.serviceClient<frank::Disconnect3DCamera>("disconnect_3dcamera");
    frank::Disconnect3DCamera cam3d_disconnect;
    ros::ServiceClient save_cam3d_calib = node_handle.serviceClient<frank::SaveCalibScan>("save_calib_pointcloud");
    frank::SaveCalibScan c3d_calib;
    ros::ServiceClient save_cam3d_sparse = node_handle.serviceClient<frank::SaveSparseScan>("save_sparse_pointcloud");
    frank::SaveSparseScan c3d_sparse;
    //
    //
    ////////SERVICES FOR PROFI
    ros::ServiceClient profi_conn = node_handle.serviceClient<frank::ConnectProfilometer>("connect_profilometer");
    frank::ConnectProfilometer profi_connect;
    ros::ServiceClient profi_disconn = node_handle.serviceClient<frank::DisconnectProfilometer>("disconnect_profilometer");
    frank::Disconnect3DCamera profi_disconnect;   
    /////SERVICE FOR SPARSE ALIGNMENT
    ros::ServiceClient align_sp = node_handle.serviceClient<frank::SparseAlignment>("sparse_alignment");
    frank::SparseAlignment align_sparse;
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //RAPID task and MOD
    std::string rapid_task = "T_ROB1";
    
    std::string rapid_module = "Reach_Check";

    //Set egm settings
    RWSStateMachineInterface::EGMSettings egm_settings;
    abb::rws::ToolData Curr_tool; //intialize the current tool and wobj for reach check
    abb::rws::WObjData Curr_Wobj;
    
    
    ready.data = true;
    counter=0;
    //CALL PLC
    plc_conn.waitForExistence();
    ros::Duration(0.5).sleep();
    if (plc_conn.call(plc_connect)){
        if (plc_connect.response.res!=1){
            ROS_INFO("plc not ready!");
            
        }
    }
    else{
        ROS_INFO("PLC service call failed");
           
    }
    //START Rapid execution
    START_IRC5();
    
    while (ros::ok())
    {
    
        egm_mode=num.data;
        
        switch (egm_mode)
        {
            case -3: //Clean Shutdown
                if (plc_disconn.call(plc_disconnect)){
                    if (plc_disconnect.response.res!=1){
                        ROS_INFO("plc not ready!");
            
                    }
                }
                else{
                    ROS_INFO("PLC service call failed");
           
                }
                
                Frank.stopRAPIDExecution();
                ros::waitForShutdown();

            
            case -2://Set the tool
                ros::param::get("/Procedure/case", Drill);
                ///Get Reference system from Param service///////
                if (Drill==1){ ////Change Tool
                    ros::param::get("/Procedure/DrillOp", DrillOp);
                    DrillBit = std::to_string(DrillOp);
                    ros::param::get("/DTool_ref"+DrillBit+"/x", toolx); ros::param::get("/DTool_ref"+DrillBit+"/y", tooly); ros::param::get("/DTool_ref"+DrillBit+"/z", toolz); ros::param::get("/DTool_ref"+DrillBit+"/q1", toolq1);
                    ros::param::get("/DTool_ref"+DrillBit+"/q2", toolq2); ros::param::get("/DTool_ref"+DrillBit+"/q3", toolq3); ros::param::get("/DTool_ref"+DrillBit+"/q4", toolq4); ros::param::get("/DTool_ref"+DrillBit+"/mass", toolmass);
                    ros::param::get("/DTool_ref"+DrillBit+"/cogx", toolcogx); ros::param::get("/DTool_ref"+DrillBit+"/cogy", toolcogy); ros::param::get("/DTool_ref"+DrillBit+"/cogz", toolcogz);
                    ros::param::get("/DTool_ref"+DrillBit+"/aom1", toolaom1); ros::param::get("/DTool_ref"+DrillBit+"/aom2", toolaom2); ros::param::get("/DTool_ref"+DrillBit+"/aom3", toolaom3); ros::param::get("/DTool_ref"+DrillBit+"/aom4", toolaom4);
                    ros::param::get("/DTool_ref"+DrillBit+"/ix", toolix); ros::param::get("/DTool_ref"+DrillBit+"/iy", tooliy); ros::param::get("/DTool_ref"+DrillBit+"/iz", tooliz);
                }
                else { ////Scan Tool
                    ros::param::get("/STool_ref/x", toolx); ros::param::get("/STool_ref/y", tooly); ros::param::get("/STool_ref/z", toolz);
                    ros::param::get("/STool_ref/q1", toolq1); ros::param::get("/STool_ref/q2", toolq2); ros::param::get("/STool_ref/q3", toolq3); ros::param::get("/STool_ref/q4", toolq4);
                    ros::param::get("/STool_ref/mass", toolmass); ros::param::get("/STool_ref/cogx", toolcogx); ros::param::get("/STool_ref/cogy", toolcogy); ros::param::get("/STool_ref/cogz", toolcogz);
                    ros::param::get("/STool_ref/aom1", toolaom1); ros::param::get("/STool_ref/aom2", toolaom2); ros::param::get("/STool_ref/aom3", toolaom3); ros::param::get("/STool_ref/aom4", toolaom4);
                    ros::param::get("/STool_ref/ix", toolix); ros::param::get("/STool_ref/iy", tooliy); ros::param::get("/STool_ref/iz", tooliz);
                }
                ros::param::get("/Wobj_ref/ufx", woobjufx); ros::param::get("/Wobj_ref/ufy", woobjufy); ros::param::get("/Wobj_ref/ufz", woobjufz);
                ros::param::get("/Wobj_ref/ufq1", woobjufq1); ros::param::get("/Wobj_ref/ufq2", woobjufq2); ros::param::get("/Wobj_ref/ufq3", woobjufq3); ros::param::get("/Wobj_ref/ufq4", woobjufq4);
                ros::param::get("/Wobj_ref/ofx", woobjofx); ros::param::get("/Wobj_ref/ofy", woobjofy); ros::param::get("/Wobj_ref/ofz", woobjofz);
                ros::param::get("/Wobj_ref/ofq1", woobjofq1); ros::param::get("/Wobj_ref/ofq2", woobjofq2); ros::param::get("/Wobj_ref/ofq3", woobjofq3); ros::param::get("/Wobj_ref/ofq4", woobjofq4);
        
               /////////////////////////////////////////-> send to Rapid
        
                egm_settings.activate.tool.robhold = true;
                egm_settings.activate.tool.tframe.pos.x = toolx;
                egm_settings.activate.tool.tframe.pos.y = tooly;
                egm_settings.activate.tool.tframe.pos.z = toolz;
                egm_settings.activate.tool.tframe.rot.q1 = toolq1;
                egm_settings.activate.tool.tframe.rot.q2 = toolq2;
                egm_settings.activate.tool.tframe.rot.q3 = toolq3;
                egm_settings.activate.tool.tframe.rot.q4 = toolq4;
                egm_settings.activate.tool.tload.mass = toolmass; //mettere dati reali tool
                egm_settings.activate.tool.tload.cog.x = toolcogx;
                egm_settings.activate.tool.tload.cog.y = toolcogy;
                egm_settings.activate.tool.tload.cog.z = toolcogz;
                egm_settings.activate.tool.tload.aom.q1 = toolaom1;
                egm_settings.activate.tool.tload.aom.q2 = toolaom2;
                egm_settings.activate.tool.tload.aom.q3 = toolaom3;
                egm_settings.activate.tool.tload.aom.q4 = toolaom4;
                egm_settings.activate.tool.tload.ix = toolix;
                egm_settings.activate.tool.tload.iy = tooliy;
                egm_settings.activate.tool.tload.iz = tooliz;
            
                egm_settings.activate.wobj.robhold = false;
                egm_settings.activate.wobj.ufprog = true;
                egm_settings.activate.wobj.ufmec.value = "\0";
                egm_settings.activate.wobj.uframe.pos.x = woobjufx;
                egm_settings.activate.wobj.uframe.pos.y = woobjufy;
                egm_settings.activate.wobj.uframe.pos.z = woobjufz;
                egm_settings.activate.wobj.uframe.rot.q1 = woobjufq1;
                egm_settings.activate.wobj.uframe.rot.q2 = woobjufq2;
                egm_settings.activate.wobj.uframe.rot.q3 = woobjufq3;
                egm_settings.activate.wobj.uframe.rot.q4 = woobjufq4;
                egm_settings.activate.wobj.oframe.pos.x = woobjofx;
                egm_settings.activate.wobj.oframe.pos.y = woobjofy;
                egm_settings.activate.wobj.oframe.pos.z = woobjofz;
                egm_settings.activate.wobj.oframe.rot.q1 = woobjofq1;
                egm_settings.activate.wobj.oframe.rot.q2 = woobjofq2;
                egm_settings.activate.wobj.oframe.rot.q3 = woobjofq3;
                egm_settings.activate.wobj.oframe.rot.q4 = woobjofq4;
            
                egm_settings.activate.correction_frame.operator=(egm_settings.activate.wobj.uframe);
                egm_settings.activate.sensor_frame.operator=(egm_settings.activate.wobj.uframe);
            
                //egm_settings.activate.lp_filter = LP;
                egm_settings.activate.max_speed_deviation = MAX_SPEED_DEV;
                egm_settings.activate.sample_rate = SAMPLE_RATE;
                egm_settings.activate.cond_min_max = COND_MIN_MAX;
            
                egm_settings.allow_egm_motions = true;
                egm_settings.setup_uc.comm_timeout = COMM_TIMEOUT;
                egm_settings.setup_uc.use_filtering = false ;
                egm_settings.stop.ramp_out_time = RAMP_OUT_TIME;
    
                egm_settings.run.ramp_in_time = RAMP_IN_TIME;
                egm_settings.run.cond_time = COND_TIME;
                //egm_settings.run.pos_corr_gain = K;
                egm_settings.run.offset.operator=(egm_settings.activate.tool.tframe); 
                ROS_INFO("New Tool acquired!!");
                num.data=0;
                break;
            
            case -1: //BREAK the while loop
                call_RR="GoHome";
                Call_Rapid_routine(call_RR);
                num.data=0;
                break;
            
            case 1:
                Set_egm_stop();
                ros::Duration(COMM_DELAY).sleep();
                Reset_egmj_start();
                ros::Duration(COMM_DELAY).sleep();
                Reset_egm_stop();
                ros::Duration(COMM_DELAY).sleep();
                ROS_INFO("case POSE");
                Frank.services().egm().setSettings(TASK, egm_settings);
                ros::Duration(0.85).sleep();
                //cond.data=cond.data-1;
                Set_egm_start();
                ros::param::set("/EGM/mod",0);
               
                num.data=0;
                break;
            case 2:
                Set_egm_stop();
                ros::Duration(1).sleep();
                Reset_egm_stop();
                ros::Duration(COMM_DELAY).sleep();
                Reset_egm_start();
                ros::Duration(COMM_DELAY).sleep();
                Frank.services().egm().setSettings(TASK, egm_settings);
                ros::Duration(1).sleep();
                Set_egmj_start();
                //Frank.services().egm().signalEGMStartJoint();
                ros::param::set("/EGM/mod",1);
                ROS_INFO("case Joint");
                num.data=0;
               
                break;    
            case 0:
                //Frank.services().egm().signalEGMStop();
                //ros::topic::waitForMessage<std_msgs::Int16>("/num_to_check");
                //ROS_INFO("RWS idle...");
                break;
            case 3:
                if (profi_conn.call(profi_connect)){
                    if (profi_connect.response.res!=1){
                        ROS_INFO("profi not ready!");
                        num.data=0;
                        break;
                    }
                }
                else{
                     ROS_INFO("Profi conn service call failed");
                     num.data=0;
                     break;
                }
                if (cam3d_conn.call(cam3d_connect)){
                    if (cam3d_connect.response.res!=1){
                        ROS_INFO("cam3d not ready!");
                        num.data=0;
                        break;
                    }
                }
                else{
                    ROS_INFO("cam3d conn service call failed");
                    num.data=0;
                    break;
                }
                Set_egm_stop();
                ros::Duration(COMM_DELAY).sleep();
                Reset_Signals();
                ros::Duration(COMM_DELAY).sleep();
                Frank.services().rapid().setRoutineName(TASK,"SparseScan");
                ros::Duration(COMM_DELAY).sleep();
                Frank.services().rapid().signalRunRAPIDRoutine();
                ROS_INFO("SPARSE SCAN execution...");
                ros::Duration(COMM_DELAY).sleep();
                Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                RRexec_res.data=Proc_exec.value;
                while (RRexec_res.data == false)
                {
                    ros::Duration(COMM_DELAY).sleep();
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                    RRexec_res.data=Proc_exec.value;
                    ros::Duration(COMM_DELAY).sleep();
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "PCAcquired", &PCAquired);
                    pc_aquired.data=PCAquired.value;
            
                    if(pc_aquired.data==false){
                        if (cam3d_client.call(cam3d_comm)){
                            ROS_INFO("3D photo acquired");
                        }
                        else{
                            ROS_INFO("3D camera fails...");
                        }
                       
                        PCAquired.value=true;
                        Frank.setRAPIDSymbolData(TASK,rapid_routines_module, "PCAcquired", PCAquired);
                    }
                      
                }
                //Add the turn off of RUNRAPIDROUTINE signal!!
		//ADDED CALL FOR SAVING SPARSE SCAN
		        if (save_cam3d_sparse.call(c3d_sparse)){
                        ROS_INFO("saved calibration data");
                }
                else{
                    ROS_INFO("3D camera fails...");
                }
                ROS_INFO("SPARSE SCAN execution finished");
                Reset_RUN_RAPID(); 
                OpState.procedure=num.data-1;
                OpState.state="Completed";
                OpState.error="No error";
                State_pub.publish(OpState);
                //ROS_INFO("Request of sparse alignment");
                //align_sparse.request.mode=1;
                //if (align_sp.call(align_sparse)){
                //    if (align_sparse.response.res!=1){
                //        ROS_INFO("Auto mode failed,  Manual called");
                //        align_sparse.request.mode=0;
                //        if (align_sp.call(align_sparse)){
                //            if (align_sparse.response.res!=1){
                //                ROS_INFO("Also manual mode failed");
                //            }
                //        }
                //        else{
                //            ROS_INFO("manual request failed!");
                //        }
                //    }
                //}
                //else{
                //    ROS_INFO("Sparse alignment call failed!!");
                //}
                num.data=0;
                break;    
            case 4:
                plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                plc_comm.request.value=200;
                if (plc_client.call(plc_comm)){
                    if (plc_comm.response.res!=1){
                         ROS_INFO("plc not ready!");
                         break;
                    }
                }
                else{
                     ROS_INFO("PLC service call failed");
                     break;
                }
                call_RR="LoadDrillTool";
                Call_Rapid_routine(call_RR);
                OpState.procedure=num.data-1;
                OpState.state="Completed";
                OpState.error="No error";
                State_pub.publish(OpState);
                num.data=0;
                break;    
            case 5:
                plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                plc_comm.request.value=400;
                if (plc_client.call(plc_comm)){
                    if (plc_comm.response.res!=1){
                        ROS_INFO("plc not ready!");
                        break;
                    }
                }
                else{
                    ROS_INFO("PLC service call failed");
                    break;
                }
                ros::Duration(0.2).sleep();
                call_RR="LoadInspectionTool";
                Call_Rapid_routine(call_RR);
                ros::Duration(5).sleep();
                if (profi_conn.call(profi_connect)){
                    if (profi_connect.response.res!=1){
                        ROS_INFO("profi not ready!");
                        num.data=0;
                        break;
                    }
                }
                else{
                     ROS_INFO("Profi conn service call failed");
                     num.data=0;
                     break;
                }
                if (cam3d_conn.call(cam3d_connect)){
                    if (cam3d_connect.response.res!=1){
                        ROS_INFO("cam3d not ready!");
                        num.data=0;
                        break;
                    }
                }
                else{
                    ROS_INFO("cam3d conn service call failed");
                    num.data=0;
                    break;
                }
                Reset_RUN_RAPID(); 
                
                OpState.procedure=num.data-1;
                OpState.state="Completed";
                OpState.error="No error";
                State_pub.publish(OpState);
                num.data=0;
                break;
            case 6:
                ROS_INFO("STARTING UNMOUNTING PROFILOMETER");
                if (profi_disconn.call(profi_disconnect)){
                    if (profi_disconnect.response.res!=1){
                        ROS_INFO("profi not ready!");
                        break;
                    }
                }
                else{
                     ROS_INFO("Profi conn service call failed");
                     break;
                }
                if (cam3d_disconn.call(cam3d_disconnect)){
                    if (cam3d_disconnect.response.res!=1){
                        ROS_INFO("cam3d not ready!");
                        break;
                    }
                }
                else{
                    ROS_INFO("cam3d conn service call failed");
                    break;
                }
                plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                plc_comm.request.value=300;
                if (plc_client.call(plc_comm)){
                    if (plc_comm.response.res!=1){
                        ROS_INFO("plc not ready!");
                        break;
                    }
                }
                else{
                     ROS_INFO("PLC service call failed");
                     break;
                }
                ros::Duration(0.2).sleep();
                call_RR="UnloadInspectionTool";
                Call_Rapid_routine(call_RR);
                OpState.procedure=num.data-1;
                OpState.state="Completed";
                OpState.error="No error";
                State_pub.publish(OpState);
                num.data=0;
                break;
            case 7:
                Set_egm_stop();
                ros::Duration(COMM_DELAY).sleep();
                Reset_Signals();
                ros::param::get("/DrillBit/actual", DbActual);
                if (DbActual!=0){
                    plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                    plc_comm.request.value=500;
                    if (plc_client.call(plc_comm)){
                        if (plc_comm.response.res!=1){
                            ROS_INFO("plc not ready!");
                            break;
                        }
                    }  
                    else{
                        ROS_INFO("PLC service call failed");
                        break;                
                    }
                    DbActualString=std::to_string(DbActual);
                    ros::param::get("/DrillBit/"+DbActualString+"_pos", DbOut.value);
                    Frank.setRAPIDSymbolData(TASK,rapid_caseRoutine_module, "DbOut", DbOut);
                    ros::Duration(COMM_DELAY).sleep();
                    Frank.services().rapid().setRoutineName(TASK,"UnloadDrillBit");
                    ros::Duration(COMM_DELAY).sleep();
                    Frank.services().rapid().signalRunRAPIDRoutine();
                    ros::Duration(COMM_DELAY).sleep();
                    ROS_INFO("Unloading Drillbit execution...");
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                    RRexec_res.data=Proc_exec.value;
                    while (RRexec_res.data == false){
                        ros::Duration(COMM_DELAY).sleep();
                        Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                        RRexec_res.data=Proc_exec.value;
                        ros::Duration(COMM_DELAY).sleep();
                    }
                    ROS_INFO("Unloading drillbit execution finished");
                    ros::param::set("/DrillBit/actual", 0);
                }
                plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                plc_comm.request.value=100;
                if (plc_client.call(plc_comm)){
                    if (plc_comm.response.res!=1){
                        ROS_INFO("plc not ready!");
                        break;
                    }
                }
                else{
                    ROS_INFO("PLC service call failed");
                    break;                
                }
                ros::Duration(0.2).sleep();
                call_RR="UnloadDrillTool";
                Call_Rapid_routine(call_RR);
                //ROS_INFO("RESTART");
                OpState.procedure=num.data-1;
                OpState.state="Completed";
                OpState.error="No error";
                State_pub.publish(OpState);
                num.data=0;
                break;
                num.data=0;
                break;
            case -4:
                
                Set_egm_stop();
                ros::Duration(COMM_DELAY).sleep();
                Reset_Signals();
                ros::Duration(COMM_DELAY).sleep();
                Frank.services().rapid().setRoutineName(TASK,"Camera3d_Calibration");
                ros::Duration(COMM_DELAY).sleep();
                Frank.services().rapid().signalRunRAPIDRoutine();
                ROS_INFO("CAMERA 3D CALIBRATION execution...");
                ros::Duration(COMM_DELAY).sleep();
                Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                RRexec_res.data=Proc_exec.value;
                while (RRexec_res.data == false)
                {
                    ros::Duration(COMM_DELAY).sleep();
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                    RRexec_res.data=Proc_exec.value;
                    ros::Duration(COMM_DELAY).sleep();
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "PCAcquired", &PCAquired);
                    pc_aquired.data=PCAquired.value;
            
                    if(pc_aquired.data==false){
                        if (cam3d_client.call(cam3d_comm)){
                            ROS_INFO("3D photo acquired");
                        }
                        else{
                            ROS_INFO("3D camera fails...");
                        }
                       
                        PCAquired.value=true;
                        ros::Duration(COMM_DELAY).sleep();
                        Frank.setRAPIDSymbolData(TASK,rapid_routines_module, "PCAcquired", PCAquired);
                    }
                      
                }
                //Add the turn off of RUNRAPIDROUTINE signal!!
		//ADD calling 3dcamera save serviice
                if (save_cam3d_calib.call(c3d_calib)){
                    ROS_INFO("saved calibration data");
                }
                else{
                    ROS_INFO("3D camera fails...");
                }
                ROS_INFO("Camera 3d calibration execution finished");
                Reset_RUN_RAPID(); 
                //ROS_INFO("RESTART");
                num.data=0;
                break;    
            case 8:
                Set_egm_stop();
                ros::Duration(COMM_DELAY).sleep();
                Reset_Signals();
                plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                plc_comm.request.value=600;
                if (plc_client.call(plc_comm)){
                    if (plc_comm.response.res!=1){
                        ROS_INFO("plc not ready!");
                        break;
                    }
                }
                else{
                    ROS_INFO("PLC service call failed");
                    break;                
                }
                Frank.services().rapid().setRoutineName(TASK,"LoadDrillBit");
                ros::Duration(COMM_DELAY).sleep();
                ros::param::get("/DrillBit/target",DbIn.value);
                Frank.setRAPIDSymbolData(TASK,rapid_caseRoutine_module, "DbIn", DbIn);
                ros::Duration(COMM_DELAY).sleep();
                Frank.services().rapid().signalRunRAPIDRoutine();
                ROS_INFO("Load Drill bit execution...");
                ros::Duration(COMM_DELAY).sleep();
                Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                RRexec_res.data=Proc_exec.value;
                while (RRexec_res.data == false){
                    ros::Duration(COMM_DELAY).sleep();
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                    RRexec_res.data=Proc_exec.value;
                }
                //Add the turn off of RUNRAPIDROUTINE signal!!
                ROS_INFO("Load drillbit execution finished");
                ros::param::get("/Procedure/DrillOp", DrillOp);
                ros::param::set("/DrillBit/actual", DrillOp);
                ros::Duration(COMM_DELAY).sleep();
                Reset_RUN_RAPID();
                //ROS_INFO("RESTART");
                num.data=0;
                break;
            case 9:
                Set_egm_stop();
                ros::Duration(COMM_DELAY).sleep();
                Reset_Signals();
                plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                plc_comm.request.value=700;
                if (plc_client.call(plc_comm)){
                    if (plc_comm.response.res!=1){
                        ROS_INFO("plc not ready!");
                        break;
                    }
                }
                else{
                    ROS_INFO("PLC service call failed");
                    break;                
                }
                Frank.services().rapid().setRoutineName(TASK,"SwitchDrillBit");
                ros::Duration(COMM_DELAY).sleep();
                ros::param::get("/DrillBit/target",DbIn.value);
                ros::param::get("/DrillBit/actual", DbActual);
                DbActualString = std::to_string(DbActual);
                ros::param::get("/DrillBit/"+DbActualString+"_pos", DbOut.value);
                Frank.setRAPIDSymbolData(TASK,rapid_caseRoutine_module, "DbOut", DbOut);
                ros::Duration(COMM_DELAY).sleep();
                Frank.setRAPIDSymbolData(TASK,rapid_caseRoutine_module, "DbIn", DbIn);
                ros::Duration(COMM_DELAY).sleep();
                Frank.services().rapid().signalRunRAPIDRoutine();
                ROS_INFO("Switch Drill bit execution...");
                ros::Duration(COMM_DELAY).sleep();
                Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                RRexec_res.data=Proc_exec.value;
                while (RRexec_res.data == false){
                    ros::Duration(COMM_DELAY).sleep();
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                    RRexec_res.data=Proc_exec.value;
                }
                //Add the turn off of RUNRAPIDROUTINE signal!!
                ROS_INFO("Switch drillbit execution finished");
                ros::param::get("/Procedure/DrillOp", DrillOp);
                ros::param::set("/DrillBit/actual", DrillOp);
                ros::Duration(COMM_DELAY).sleep();
                Reset_RUN_RAPID();
                //ROS_INFO("RESTART");
                num.data=0;
                break;

            case 15:
                 ROS_INFO("STARTING MOUNTING bit 5mm");
                 plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                 plc_comm.request.value=600;
                 if (plc_client.call(plc_comm)){
                    if (plc_comm.response.res!=1){
                         ROS_INFO("plc not ready!");
                         break;
                    }
                }
                else{
                     ROS_INFO("PLC service call failed");
                     break;
                }
                while (Frank.getIOSignal(SIGNAL_EGM_STOP)==LOW){
                    Frank.services().egm().signalEGMStop();

                }
                Reset_Signals();
                Frank.services().rapid().setRoutineName(TASK,"Load5mm");
                ros::Duration(0.2).sleep();
                Frank.services().rapid().signalRunRAPIDRoutine();
                ROS_INFO(" execution...");
                Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                RRexec_res.data=Proc_exec.value;
                while (RRexec_res.data == false){
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                    RRexec_res.data=Proc_exec.value;
                      
                }
                //Add the turn off of RUNRAPIDROUTINE signal!!
                ROS_INFO("STARTING MOUNTING bit 5mm finished");
                Reset_RUN_RAPID(); 
                //ROS_INFO("RESTART");
                num.data=0;
                break;
            case 10:
                 ROS_INFO("STARTING Unloading bit 5mm");
                 plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                 plc_comm.request.value=500;
                 if (plc_client.call(plc_comm)){
                    if (plc_comm.response.res!=1){
                         ROS_INFO("plc not ready!");
                         break;
                    }
                }
                else{
                     ROS_INFO("PLC service call failed");
                     break;
                }
                while (Frank.getIOSignal(SIGNAL_EGM_STOP)==LOW){
                    Frank.services().egm().signalEGMStop();

                }
                Reset_Signals();
                Frank.services().rapid().setRoutineName(TASK,"Unload5mm");
                ros::Duration(0.2).sleep();
                Frank.services().rapid().signalRunRAPIDRoutine();
                ROS_INFO(" execution...");
                Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                RRexec_res.data=Proc_exec.value;
                while (RRexec_res.data == false){
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                    RRexec_res.data=Proc_exec.value;
                      
                }
                //Add the turn off of RUNRAPIDROUTINE signal!!
                ROS_INFO("Unloading bit 5mm execution finished");
                Reset_RUN_RAPID(); 
                //ROS_INFO("RESTART");
                num.data=0;
                break;
            case 11:
                ROS_INFO("STARTING MOUNTING Marker");
                plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                plc_comm.request.value=600;
                if (plc_client.call(plc_comm)){
                    if (plc_comm.response.res!=1){
                         ROS_INFO("plc not ready!");
                         break;
                    }
                }
                else{
                     ROS_INFO("PLC service call failed");
                     break;
                }
                while (Frank.getIOSignal(SIGNAL_EGM_STOP)==LOW){
                    Frank.services().egm().signalEGMStop();

                }
                Reset_Signals();
                Frank.services().rapid().setRoutineName(TASK,"LoadMarker");
                ros::Duration(0.2).sleep();
                Frank.services().rapid().signalRunRAPIDRoutine();
                ROS_INFO(" execution...");
                Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                RRexec_res.data=Proc_exec.value;
                while (RRexec_res.data == false){
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                    RRexec_res.data=Proc_exec.value;
                      
                }
                //Add the turn off of RUNRAPIDROUTINE signal!!
                ROS_INFO(" execution finished");
                Reset_RUN_RAPID(); 
                //ROS_INFO("RESTART");
                num.data=0;
                break;
            case 12:
                ROS_INFO("STARTING unloading Marker");
                plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                plc_comm.request.value=500;
                if (plc_client.call(plc_comm)){
                    if (plc_comm.response.res!=1){
                         ROS_INFO("plc not ready!");
                         break;
                    }
                }
                else{
                     ROS_INFO("PLC service call failed");
                     break;
                }
                while (Frank.getIOSignal(SIGNAL_EGM_STOP)==LOW){
                    Frank.services().egm().signalEGMStop();

                }
                Reset_Signals();
                Frank.services().rapid().setRoutineName(TASK,"UnloadMarker");
                ros::Duration(0.2).sleep();
                Frank.services().rapid().signalRunRAPIDRoutine();
                ROS_INFO(" execution...");
                Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                RRexec_res.data=Proc_exec.value;
                while (RRexec_res.data == false){
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                    RRexec_res.data=Proc_exec.value;
                      
                }
                //Add the turn off of RUNRAPIDROUTINE signal!!
                ROS_INFO(" execution finished");
                Reset_RUN_RAPID(); 
                //ROS_INFO("RESTART");
                num.data=0;
                break; 
            case 13:
                ROS_INFO("STARTING MOUNTING bit 3mm");
                plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                plc_comm.request.value=600;
                if (plc_client.call(plc_comm)){
                    if (plc_comm.response.res!=1){
                         ROS_INFO("plc not ready!");
                         break;
                    }
                }
                else{
                     ROS_INFO("PLC service call failed");
                     break;
                }
                while (Frank.getIOSignal(SIGNAL_EGM_STOP)==LOW){
                    Frank.services().egm().signalEGMStop();

                }
                Reset_Signals();
                Frank.services().rapid().setRoutineName(TASK,"Load3mm");
                ros::Duration(0.2).sleep();
                Frank.services().rapid().signalRunRAPIDRoutine();
                ROS_INFO(" execution...");
                Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                RRexec_res.data=Proc_exec.value;
                while (RRexec_res.data == false){
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                    RRexec_res.data=Proc_exec.value;
                      
                }
                //Add the turn off of RUNRAPIDROUTINE signal!!
                ROS_INFO(" execution finished");
                Reset_RUN_RAPID(); 
                //ROS_INFO("RESTART");
                num.data=0;
                break; 
            case 14:
                ROS_INFO("STARTING unloading bit 3mm");
                plc_comm.request.command="GVL_ROS.IRC5_ParteProgrammaAct";
                plc_comm.request.value=500;
                if (plc_client.call(plc_comm)){
                    if (plc_comm.response.res!=1){
                         ROS_INFO("plc not ready!");
                         break;
                    }
                }
                else{
                     ROS_INFO("PLC service call failed");
                     break;
                }
                while (Frank.getIOSignal(SIGNAL_EGM_STOP)==LOW){
                    Frank.services().egm().signalEGMStop();

                }
                Reset_Signals();
                Frank.services().rapid().setRoutineName(TASK,"Unload3mm");
                ros::Duration(0.2).sleep();
                Frank.services().rapid().signalRunRAPIDRoutine();
                ROS_INFO(" execution...");
                Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                RRexec_res.data=Proc_exec.value;
                while (RRexec_res.data == false){
                    Frank.getRAPIDSymbolData(TASK,rapid_routines_module, "Proc_exec", &Proc_exec);
                    RRexec_res.data=Proc_exec.value;
                      
                }
                //Add the turn off of RUNRAPIDROUTINE signal!!
                ROS_INFO(" execution finished");
                Reset_RUN_RAPID(); 
                //ROS_INFO("RESTART");
                num.data=0;
                break;          
        }
            

            
        

       
        //ros::waitForShutdown();
        //Reset_Signals();
        ros::Duration(COMM_DELAY).sleep();
        
        //Frank.stopRAPIDExecution();
        //ros::Duration(0.1).sleep();
 
    }
  
  return 0;
}




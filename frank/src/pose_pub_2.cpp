#include <ros/ros.h>
#include <abb_libegm/egm_controller_interface.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>

#include <sensor_msgs/JointState.h>



//sensor_msgs::JointState set_point;
trajectory_msgs::MultiDOFJointTrajectoryPoint set_point;
bool new_set_point_received = false;
sensor_msgs::JointState j_set_point;
bool new_jset_point_received = false;
int Drill;
int egm_mod;

void setpoint_callback(const trajectory_msgs::MultiDOFJointTrajectoryPoint &msg)
{
  set_point.transforms.operator=(msg.transforms);
  set_point.velocities.operator=(msg.velocities);
  new_set_point_received = true;
}
void j_setpoint_callback(const sensor_msgs::JointState &msg)
{
  j_set_point.position.operator=(msg.position);
  j_set_point.velocity.operator=(msg.velocity);
  new_jset_point_received = true;
}


int main(int argc, char **argv)
{
  //----------------------------------------------------------
  // Preparations
  //----------------------------------------------------------
  // Initialize the node.
  ros::init(argc, argv, "pose_pub_2_node");
  //TODO: multithreading for callback
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // OPTIONS
  bool use_vel = false;
  int egm_port = 6511;

  
  ros::Publisher feedback_publisher = node_handle.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/robot_feedback", 1);
  ros::Subscriber egm_sub = node_handle.subscribe("/robot_set_point", 1, &setpoint_callback);
  ros::Subscriber egmj_sub = node_handle.subscribe("/robot_jset_point", 1, &j_setpoint_callback);
  ros::Publisher Jfeedback_publisher = node_handle.advertise<sensor_msgs::JointState>("/joints_robot_feedback", 1);

  // Boost components for managing asynchronous UDP socket(s).
  boost::asio::io_service io_service;
  boost::thread_group thread_group;

  // Create EGM configurations.
  abb::egm::BaseConfiguration configuration;
  //TODO: change with position only
  configuration.use_velocity_outputs = use_vel;

  // Create an EGM interface:
  // * Sets up an EGM server (that the robot controller's EGM client can connect to).
  // * Provides APIs to the user (for setting motion references, that are sent in reply to the EGM client's request).
  //
  // Note: It is important to set the correct port number here,
  //       as well as configuring the settings for the EGM client in thre robot controller.
  //       If using the included RobotStudio Pack&Go file, then port 6511 = ROB_1, 6512 = ROB_2, etc.
  abb::egm::EGMControllerInterface egm_interface(io_service, egm_port, configuration);

  if (!egm_interface.isInitialized())
  {
    ROS_ERROR("EGM interface failed to initialize (e.g. due to port already bound)");
    return 0;
  }

  // Spin up a thread to run the io_service.
  thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

  //----------------------------------------------------------
  // Execute a pose velocity controller loop.
  //
  // Note 1: The EGM communication session is started by the
  //         EGMRunPose RAPID instruction.
  //
  // Note 2: To get pure velocity control, then the EGM client
  //         (in the robot controller) need its position
  //         correction gain to be set to 0. This is done with
  //         the EGMRunPose RAPID instruction.
  //----------------------------------------------------------
  ROS_INFO("========== Pose velocity controller (open-loop) sample ==========");
  bool wait = true;
  bool init = true;
  abb::egm::wrapper::Input input;
  abb::egm::wrapper::CartesianVelocity initial_velocity;
  abb::egm::wrapper::CartesianPose initial_pose;
  abb::egm::wrapper::Joints initial_position;

  const int egm_rate = 250.0; // [Hz] (EGM communication rate, specified by the EGMActPose RAPID instruction).
  int sequence_number = 0;    // [-] (sequence number of a received EGM message).
  double time = 0.0;// [seconds] (elapsed time during an EGM communication session).
  double DegTorad = 0.01745329251994329576924;           
  
  int starter = 0;
  int starterC = 0;
  int starterJ= 0;
  abb::egm::wrapper::Output output;
  abb::egm::wrapper::Output outputj; // added later

  sensor_msgs::JointState Joint_feedback; //Create joint pos feedback msg
  Joint_feedback.header.seq=0;
  Joint_feedback.name.push_back("joint_1");
  Joint_feedback.name.push_back("joint_2");
  Joint_feedback.name.push_back("joint_3");
  Joint_feedback.name.push_back("joint_4");
  Joint_feedback.name.push_back("joint_5");
  Joint_feedback.name.push_back("joint_6");

  for(int k=0;k<6;k++){
    Joint_feedback.position.push_back(0);
    Joint_feedback.velocity.push_back(0);
    Joint_feedback.effort.push_back(0);

  }

  ROS_INFO("1: Wait for an EGM communication session to start...");
  while (ros::ok() && wait)
  {
    if (egm_interface.isConnected())
    {
      if (egm_interface.getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
      {
        ROS_WARN("RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.");
      }
      else
      {
        wait = egm_interface.getStatus().egm_state()!=abb::egm::wrapper::Status_EGMState_EGM_RUNNING;//egm_interface.getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
      }
    }

    ros::Duration(0.5).sleep();
  }

  while (ros::ok())
  {
    ros::param::get("/Procedure/case",Drill);
    if (Drill==1){
    // Wait for a new EGM message from the EGM client (with a timeout of 500 ms).
    if (egm_interface.waitForMessage(4))
    {
      // Read the message received from the EGM client.
      egm_interface.read(&input);
      sequence_number = input.header().sequence_number();
      //ROS_INFO_STREAM(sequence_number);

      if (sequence_number==0)
      {
        // Reset all references, if it is the first message.
        output.Clear();
        outputj.Clear();
        initial_velocity.CopyFrom(input.feedback().robot().cartesian().velocity());
        output.mutable_robot()->mutable_cartesian()->mutable_velocity()->CopyFrom(initial_velocity);
        initial_pose.CopyFrom(input.feedback().robot().cartesian().pose());
        output.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(initial_pose);
//
        initial_position.CopyFrom(input.feedback().robot().joints().position());
        outputj.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(initial_position);

        //egm_interface.write(output);
        ROS_INFO("sequence=0");
        
      }
      else
      {
        time = sequence_number / ((double)egm_rate);
        geometry_msgs::Twist feedback_vel;
        geometry_msgs::Transform feedback_pos;
        ros::Time t = ros::Time::now();

        ros::param::get("/EGM/mod", egm_mod);

        
        Joint_feedback.header.seq++;
        Joint_feedback.header.stamp=ros::Time::now();

        for(int m=0;m<6;m++){
          Joint_feedback.position[m]=DegTorad*input.feedback().robot().joints().position().values(m);
          Joint_feedback.velocity[m]=input.feedback().robot().joints().velocity().values(m);
        }
        int feedback_time;
        feedback_pos.translation.x = input.feedback().robot().cartesian().pose().position().x();
        feedback_pos.translation.y = input.feedback().robot().cartesian().pose().position().y();
        feedback_pos.translation.z = input.feedback().robot().cartesian().pose().position().z();
        //feedback_pos.rotation.x = input.feedback().robot().cartesian().pose().euler().x();
        //feedback_pos.rotation.y = input.feedback().robot().cartesian().pose().euler().y();
        //feedback_pos.rotation.z = input.feedback().robot().cartesian().pose().euler().z();
        feedback_pos.rotation.x = input.feedback().robot().cartesian().pose().quaternion().u1();
        feedback_pos.rotation.y = input.feedback().robot().cartesian().pose().quaternion().u2();
        feedback_pos.rotation.z = input.feedback().robot().cartesian().pose().quaternion().u3();
        feedback_pos.rotation.w = input.feedback().robot().cartesian().pose().quaternion().u0();

        feedback_vel.linear.x = input.feedback().robot().cartesian().velocity().linear().x();
        feedback_vel.linear.y = input.feedback().robot().cartesian().velocity().linear().y();
        feedback_vel.linear.z = input.feedback().robot().cartesian().velocity().linear().z();
        feedback_vel.angular.x = input.feedback().robot().cartesian().velocity().angular().x();
        feedback_vel.angular.y = input.feedback().robot().cartesian().velocity().angular().y();
        feedback_vel.angular.z = input.feedback().robot().cartesian().velocity().angular().z();
        feedback_time = sequence_number;
        //Publish Feedback
        trajectory_msgs::MultiDOFJointTrajectoryPoint feedback_msg;
        feedback_msg.transforms.push_back(feedback_pos);
        feedback_msg.velocities.push_back(feedback_vel);
        feedback_msg.time_from_start = ros::Duration(time) ;  
        if(egm_mod==1){  
          Jfeedback_publisher.publish(Joint_feedback);
        }
        //ROS_INFO("JointFeedbackPublished");
        else if(egm_mod==0){
          feedback_publisher.publish(feedback_msg);
        }
        else{
          Jfeedback_publisher.publish(Joint_feedback);
          feedback_publisher.publish(feedback_msg);

        }
        
        //ROS_INFO("feedbackPublished");
        
        // If new set point received from the call back function, apply it to the controller
        if (new_set_point_received)
        {
          new_set_point_received = false;
          
          
          output.Clear();
          //if (starterC==0){
          //  initial_pose.CopyFrom(input.feedback().robot().cartesian().pose());
          //  output.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(initial_pose);
          //  
          //}  
          starterC=starterC+1;
          starterJ=0;
          //ROS_INFO("set_point_received");
          //output.mutable_robot()->mutable_joints()->mutable_position()->set_values(0, set_point.position[0]);
          //output.mutable_robot()->mutable_joints()->mutable_position()->set_values(1, set_point.position[1]);
          //output.mutable_robot()->mutable_joints()->mutable_position()->set_values(2, set_point.position[2]);
          //output.mutable_robot()->mutable_joints()->mutable_position()->set_values(3, set_point.position[3]);
          //output.mutable_robot()->mutable_joints()->mutable_position()->set_values(4, set_point.position[4]);
          //output.mutable_robot()->mutable_joints()->mutable_position()->set_values(5, set_point.position[5]);}
          ////Set Quaternion
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->set_u1(set_point.transforms[0].rotation.x);
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->set_u2(set_point.transforms[0].rotation.y);
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->set_u3(set_point.transforms[0].rotation.z);
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->set_u0(set_point.transforms[0].rotation.w);
          
          //Set Cartesian Position
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_x(set_point.transforms[0].translation.x);
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_y(set_point.transforms[0].translation.y);
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_z(set_point.transforms[0].translation.z);
          //ROS_INFO("%f", set_point.transforms[0].rotation.x );
          

          egm_interface.write(output);
         
        }
        if(new_jset_point_received){
          new_jset_point_received = false;
          //outputj.Clear();//added later
          if(starterJ==0){
            initial_position.CopyFrom(input.feedback().robot().joints().position());
            outputj.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(initial_position);
          }  
          starterC=0;
          starterJ=starterJ+1;
          //ROS_INFO("joint_set_point_received");
          outputj.mutable_robot()->mutable_joints()->mutable_position()->set_values(0, j_set_point.position[0]);
          outputj.mutable_robot()->mutable_joints()->mutable_position()->set_values(1, j_set_point.position[1]);
          outputj.mutable_robot()->mutable_joints()->mutable_position()->set_values(2, j_set_point.position[2]);
          outputj.mutable_robot()->mutable_joints()->mutable_position()->set_values(3, j_set_point.position[3]);
          outputj.mutable_robot()->mutable_joints()->mutable_position()->set_values(4, j_set_point.position[4]);
          outputj.mutable_robot()->mutable_joints()->mutable_position()->set_values(5, j_set_point.position[5]);
          
          

          egm_interface.write(outputj);
         
        }

        
        if (sequence_number % egm_rate == 0)
        {
          // TODO print feedback
          ROS_INFO("Connected");
         
        }
      }
      if (egm_mod==1){
        egm_interface.write(outputj);
      }
      else if (egm_mod==0){
        egm_interface.write(output);
      }
      // Write references back to the EGM client.
      
      
    }
    }
    else{
      if (egm_interface.waitForMessage(500))
      {
        // Read the message received from the EGM client.
        egm_interface.read(&input);
        sequence_number = input.header().sequence_number();
        ros::Time t = ros::Time::now();
        //ROS_INFO_STREAM(sequence_number);
        if (sequence_number==0){
          starter=0;
        }
        if (starter == 0)
        {
          // Reset all references, if it is the first message.
          output.Clear();
          outputj.Clear();
          //initial_velocity.CopyFrom(input.feedback().robot().cartesian().velocity());
          //output.mutable_robot()->mutable_cartesian()->mutable_velocity()->CopyFrom(initial_velocity);
          initial_pose.CopyFrom(input.feedback().robot().cartesian().pose());
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(initial_pose);
//  
          initial_position.CopyFrom(input.feedback().robot().joints().position());
          outputj.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(initial_position);
  
  
          ROS_INFO("sequence=0");
          
          starter=starter+1;
        }
        else
        {
          //time = sequence_number / ((double)egm_rate);
          geometry_msgs::Twist feedback_vel;
          geometry_msgs::Transform feedback_pos;
          
          
          time=double(t.sec)+double(t.nsec)*1e-9;
          
          Joint_feedback.header.seq++;
          Joint_feedback.header.stamp=ros::Time::now();
  
          for(int m=0;m<6;m++){
            Joint_feedback.position[m]=DegTorad*input.feedback().robot().joints().position().values(m);
          }
          
          ros::param::get("/EGM/mod", egm_mod);
          //Jfeedback_publisher.publish(Joint_feedback);
  
          //ROS_INFO("JointFeedbackPublished");
  
          int feedback_time;
          feedback_pos.translation.x = input.feedback().robot().cartesian().pose().position().x();
          feedback_pos.translation.y = input.feedback().robot().cartesian().pose().position().y();
          feedback_pos.translation.z = input.feedback().robot().cartesian().pose().position().z();
          //feedback_pos.rotation.x = input.feedback().robot().cartesian().pose().euler().x();
          //feedback_pos.rotation.y = input.feedback().robot().cartesian().pose().euler().y();
          //feedback_pos.rotation.z = input.feedback().robot().cartesian().pose().euler().z();
          feedback_pos.rotation.x = input.feedback().robot().cartesian().pose().quaternion().u1();
          feedback_pos.rotation.y = input.feedback().robot().cartesian().pose().quaternion().u2();
          feedback_pos.rotation.z = input.feedback().robot().cartesian().pose().quaternion().u3();
          feedback_pos.rotation.w = input.feedback().robot().cartesian().pose().quaternion().u0();
  
          feedback_vel.linear.x = input.feedback().robot().cartesian().velocity().linear().x();
          feedback_vel.linear.y = input.feedback().robot().cartesian().velocity().linear().y();
          feedback_vel.linear.z = input.feedback().robot().cartesian().velocity().linear().z();
          feedback_vel.angular.x = input.feedback().robot().cartesian().velocity().angular().x();
          feedback_vel.angular.y = input.feedback().robot().cartesian().velocity().angular().y();
          feedback_vel.angular.z = input.feedback().robot().cartesian().velocity().angular().z();
          feedback_time = sequence_number;
          //Publish Feedback
          trajectory_msgs::MultiDOFJointTrajectoryPoint feedback_msg;
          feedback_msg.transforms.push_back(feedback_pos);
          feedback_msg.velocities.push_back(feedback_vel);
          feedback_msg.time_from_start = ros::Duration(time) ;
  
          //feedback_msg.time_from_start = ros::Duration(sequence_number);
          //feedback_msg.time_from_start.sec.push_back(ros::Time::now().sec);
          //feedback_msg.time_from_start.nsec.push_back(ros::Time::now().nsec);
          //feedback_publisher.publish(feedback_msg);
          //ROS_INFO("feedbackPublished");
          if(egm_mod==1){  
            Jfeedback_publisher.publish(Joint_feedback);
          }
          //ROS_INFO("JointFeedbackPublished");
          else if(egm_mod==0){
            feedback_publisher.publish(feedback_msg);
          }
          else{
            Jfeedback_publisher.publish(Joint_feedback);
            feedback_publisher.publish(feedback_msg);
  
          }
          // If new set point received from the call back function, apply it to the controller
         
          if(new_jset_point_received){
            new_jset_point_received = false;
            //outputj.Clear();//added later
            //initial_position.CopyFrom(input.feedback().robot().joints().position());
            //outputj.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(initial_position);
          
            //ROS_INFO("joint_set_point_received");
            outputj.mutable_robot()->mutable_joints()->mutable_position()->set_values(0, j_set_point.position[0]);
            outputj.mutable_robot()->mutable_joints()->mutable_position()->set_values(1, j_set_point.position[1]);
            outputj.mutable_robot()->mutable_joints()->mutable_position()->set_values(2, j_set_point.position[2]);
            outputj.mutable_robot()->mutable_joints()->mutable_position()->set_values(3, j_set_point.position[3]);
            outputj.mutable_robot()->mutable_joints()->mutable_position()->set_values(4, j_set_point.position[4]);
            outputj.mutable_robot()->mutable_joints()->mutable_position()->set_values(5, j_set_point.position[5]);
            
            
  
            //egm_interface.write(outputj);
           
          }
  
          
          if (sequence_number % egm_rate == 0)
          {
            // TODO print feedback
            ROS_INFO("Connected");
            // ROS_INFO_STREAM("y current   =   " << feedback_msg.transforms[0].translation.x);
            // ROS_INFO_STREAM("z current   =   " << feedback_msg.transforms[0].translation.x);
            // ROS_INFO_STREAM("q1 current  =   " << feedback_msg.transforms[0].rotation.w);
            // ROS_INFO_STREAM("q2 current  =   " << feedback_msg.transforms[0].rotation.x);
            // ROS_INFO_STREAM("q3 current  =   " << feedback_msg.transforms[0].rotation.y);
            // ROS_INFO_STREAM("q4 current  =   " << feedback_msg.transforms[0].rotation.z);
          }
        }
        
        // Write references back to the EGM client.
        egm_interface.write(outputj);
        
    }  
    }
  }


  // Perform a clean shutdown.
  io_service.stop();
  thread_group.join_all();

  return 0;
}

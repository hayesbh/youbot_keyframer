#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "brics_actuator/JointPositions.h"
#include "brics_actuator/CartesianWrench.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <boost/units/systems/si/torque.hpp>
#include <boost/units/systems/si/force.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/io.hpp>

using namespace std;


std::vector<brics_actuator::JointValue> recvd_motor_positions;
std::vector<brics_actuator::JointValue> recvd_gripper_positions;
std::vector<std::vector<brics_actuator::JointValue> > keyframes;
std::vector<double> recvd_motor_velocities;

//ros::Publisher status_publisher;
ros::Publisher position_publisher;
ros::Subscriber status_subscriber;
 
bool recvd_msg;
void init(); 
void recordKeyframe();
void playRecordedAction(); 
double distFromTarget(const std::vector<brics_actuator::JointValue> tgt);
void StatusCallback(const sensor_msgs::JointState::ConstPtr& msg);
void toggleMotors();
bool isVelocityNonZero();


ros::ServiceClient srvMotorsOn;
ros::ServiceClient srvMotorsOff;

bool motorsOn;

int main(int argc, char **argv) {
  ros::init(argc, argv, "shl_youbot_keyframe_recorder");
  ros::NodeHandle n;

  srvMotorsOn = n.serviceClient<std_srvs::Empty>("/arm_1/switchOnMotors");
  srvMotorsOff = n.serviceClient<std_srvs::Empty>("/arm_1/switchOffMotors");
  
  init();
  
  status_subscriber = n.subscribe("joint_states", 1000, StatusCallback);
  //status_publisher = n.advertise<std_msgs::String> ("shl/keyframe_recorder/status", 1);
  position_publisher = n.advertise<brics_actuator::JointPositions> ("arm_1/arm_controller/position_command", 1);
  ros::Rate rate(20);

  recvd_msg = false;
  
  while (recvd_msg == false) {
    ros::spinOnce();
    rate.sleep();
  }
  
  while (n.ok()) {
    std::string input_str = "";
    cout << "Enter command: (m)otor-toggle (k)eyframe (p)lay (c)lear (q)uit." << std::endl;
    cin >> input_str;
    if (input_str[0] == 'k') {
      recordKeyframe();
    } else if (input_str[0] == 'p') {
      playRecordedAction();
    } else if (input_str[0] == 'q') {
      return 0;
    } else if (input_str[0] == 'm') {
      toggleMotors();
    } else {
      std::vector<brics_actuator::JointValue>::iterator iter;
      cout << "Joint States: " << std::endl;
      for (iter = recvd_motor_positions.begin(); 
           iter != recvd_motor_positions.end();
           ++iter) {
        cout << "    " << iter->joint_uri << " = " << iter->value << std::endl;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
   
}

void toggleMotors() {
  std_srvs::Empty empty;
  if (motorsOn)
    srvMotorsOff.call(empty);
  else
    srvMotorsOn.call(empty);
    
  motorsOn = !motorsOn;
}

void init() {
  for (int i=0;i<5;++i) {
    recvd_motor_positions.push_back(brics_actuator::JointValue());
    recvd_motor_velocities.push_back(0.0);
  }
  motorsOn = true;
  toggleMotors();
}

void recordKeyframe() {
  std::vector<brics_actuator::JointValue> keyframe;
  for (int i = 0; i < 5; ++i) {
    keyframe.push_back(recvd_motor_positions[i]);
  }
  //keyframe[6] = recvd_gripper_positions[0];
  //keyframe[7] = recvd_gripper_positions[1];
  
  keyframes.push_back(keyframe);
  std::cout << "Recorded keyframe " << keyframes.size() << std::endl;
}

void playRecordedAction() {
  if (motorsOn == false)
    toggleMotors();
  std::vector<std::vector<brics_actuator::JointValue> >::const_iterator kf_iter;
  ros::Rate rate(4);
  
  int kf_idx = 0;
  unsigned int total_kfs = keyframes.size();
  for (kf_iter = keyframes.begin(); kf_iter != keyframes.end(); ++kf_iter) {
    ++kf_idx;
    std::vector<brics_actuator::JointValue> kf = *kf_iter;
    
    brics_actuator::JointPositions command;
    command.positions = kf;
    position_publisher.publish(command);   
    
    while (distFromTarget(kf) > 0.5) {
//    while (isVelocityNonZero()) {
      position_publisher.publish(command);   
      std::cout << "Waiting to hit keyframe " << kf_idx << " of " <<  total_kfs 
           << std::endl;
      ros::spinOnce();
      rate.sleep();
    }
  }
  
  
}


double distFromTarget(const std::vector<brics_actuator::JointValue> tgt) {
  std::vector<brics_actuator::JointValue>::const_iterator iter;
  std::vector<brics_actuator::JointValue>::const_iterator iter_rt;
  double dist = 0.;
  for (iter = tgt.begin(), iter_rt = recvd_motor_positions.begin(); 
       iter != tgt.end(), iter_rt != recvd_motor_positions.end(); 
       ++iter, ++iter_rt) {
    dist += (iter->value - iter_rt->value) * (iter->value - iter_rt->value);
  }
  
  return sqrt(dist);
}


bool isVelocityNonZero() {
  for (int i=0;i<recvd_motor_positions.size();++i) {
    if (recvd_motor_velocities[i] > 0.02) return false;    
  }
  return true;
}

void StatusCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  recvd_msg = true;
  // std::cout << "Recv'd joint state." << std::endl;
  for (int i = 0; i < 5; ++i) {
    recvd_motor_positions[i].joint_uri = msg->name[i];    
    recvd_motor_positions[i].value = msg->position[i];
    recvd_motor_velocities[i] = msg->velocity[i];
    recvd_motor_positions[i].unit = boost::units::to_string(boost::units::si::radians);
  }
/*  recvd_gripper_positions[0].joint_uri= msg->name[5];
  recvd_gripper_positions[0].value = msg->position[5];
  recvd_gripper_positions[1].joint_uri = msg->name[6];
  recvd_gripper_positions[1].value = msg->position[6];*/
}

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

// Action interface type for moving TIAGO's torso
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_control_client;
typedef boost::shared_ptr<torso_control_client> torso_control_client_Ptr;
// Action interface type for moving TIAGo's arm
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr<arm_control_client>  arm_control_client_Ptr;
// Action interface type for moving TIAGO's gripper
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  gripper_control_client;
typedef boost::shared_ptr<gripper_control_client> gripper_control_client_Ptr;

// Create a ROS action client to move TIAGO's torso
void createTorsoClient(torso_control_client_Ptr& torso_action_client, const std::string torso_controller_name)
{
  ROS_INFO("Creating action client to %s ...", torso_controller_name.c_str());

  std::string torso_action_client_name = "/" + torso_controller_name + "/follow_joint_trajectory";
  torso_action_client.reset( new torso_control_client(torso_action_client_name) );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !torso_action_client->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the torso_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createTorsoClient: torso controller action server not available");
  else
    ROS_INFO("Success creating the torso_controller_action server");
}

// Create a ROS action client to move TIAGo's arm
void createArmClient(arm_control_client_Ptr& action_client, const std::string arm_controller_name)
{
  ROS_INFO("Creating action client to %s ...", arm_controller_name.c_str());

  std::string action_client_name = "/" + arm_controller_name + "/follow_joint_trajectory";
  action_client.reset( new arm_control_client(action_client_name) );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !action_client->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
  else
    ROS_INFO("Success creating the arm_controller_action server");
}

// Create a ROS action client to move TIAGo's gripper
void createGripperClient(gripper_control_client_Ptr& gripper_action_client, const std::string gripper_controller_name)
{
  ROS_INFO("Creating action client to %s ...", gripper_controller_name.c_str());
  std::string gripper_client_name = "/" + gripper_controller_name + "/follow_joint_trajectory";
  gripper_action_client.reset(new gripper_control_client(gripper_client_name));
  int iterations = 0, max_iterations = 3;
  // Wait for gripper controller action server to come up
  while(!gripper_action_client->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations)
  {
    ROS_DEBUG("Waiting for the gripper_controller_action server to come up");
    ++iterations;
  }

  if(iterations == max_iterations)
    throw std::runtime_error("Error in createGripperClient: gripper controller action server not available");
  else
    ROS_INFO("Success creating the gripper_controller_action server");
}

// Generates a simple trajectory with two waypoints to move TIAGO's torso
void waypointsTorso(control_msgs::FollowJointTrajectoryGoal& goal) 
{
  // The torso joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("torso_lift_joint");
  int torsoJoints = 1;

  // 2 waypoints in this goal trajectory
  int numPoints = 2;
  goal.trajectory.points.resize(numPoints);

  // 1st trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(torsoJoints);
  goal.trajectory.points[index].positions[0] = 0.14;
  // Velocities
  goal.trajectory.points[index].velocities.resize(torsoJoints);
  for (int j = 0; j < torsoJoints; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // To be reached 2 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(6.0);

  // 2nd trajectory point
  // Positions
  index+= 1;
  goal.trajectory.points[index].positions.resize(torsoJoints);
  goal.trajectory.points[index].positions[0] = 0.35;
  // Velocities
  goal.trajectory.points[index].velocities.resize(torsoJoints);
  for(int j = 0; j < torsoJoints; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // To be reached 2 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(12.0);
}

// Generates a simple trajectory with seven waypoints to move TIAGo's arm 
void waypointsArmRightGoal(control_msgs::FollowJointTrajectoryGoal& goal, control_msgs::FollowJointTrajectoryGoal& goalgrip)
{
  // The arm joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_right_1_joint");
  goal.trajectory.joint_names.push_back("arm_right_2_joint");
  goal.trajectory.joint_names.push_back("arm_right_3_joint");
  goal.trajectory.joint_names.push_back("arm_right_4_joint");
  goal.trajectory.joint_names.push_back("arm_right_5_joint");
  goal.trajectory.joint_names.push_back("arm_right_6_joint");
  goal.trajectory.joint_names.push_back("arm_right_7_joint");
  int numJoints = 7;

  //  The gripper joint names
  goalgrip.trajectory.joint_names.push_back("gripper_right_left_finger_joint");
  goalgrip.trajectory.joint_names.push_back("gripper_right_right_finger_joint");
  int gripJoints = 2;  
  
  // 5 waypoints in this goal trajectory
  int numPoints = 5;
  goal.trajectory.points.resize(numPoints);
  goalgrip.trajectory.points.resize(numPoints);

  // 1st trajectory point (home)
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(numJoints);
  goal.trajectory.points[index].positions[0] = -1.1;
  goal.trajectory.points[index].positions[1] = 1.46;
  goal.trajectory.points[index].positions[2] = 2.72;
  goal.trajectory.points[index].positions[3] = 1.7;
  goal.trajectory.points[index].positions[4] = -1.57;
  goal.trajectory.points[index].positions[5] = 1.37;
  goal.trajectory.points[index].positions[6] = 0.0;
  goalgrip.trajectory.points[index].positions.resize(gripJoints);
  goalgrip.trajectory.points[index].positions[0] = 0.04;  
  goalgrip.trajectory.points[index].positions[1] = 0.04;
  // Velocities
  goal.trajectory.points[index].velocities.resize(numJoints);
  goalgrip.trajectory.points[index].velocities.resize(gripJoints);
  for (int j = 0; j < numJoints; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  for (int i = 0; i < gripJoints; i++)
  {
    goalgrip.trajectory.points[index].velocities[i] = 0.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
  goalgrip.trajectory.points[index].time_from_start = ros::Duration(2.0); 
  
  // 2nd trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(numJoints);
  goal.trajectory.points[index].positions[0] = -1.1;
  goal.trajectory.points[index].positions[1] = 1.46;
  goal.trajectory.points[index].positions[2] = 2.72;
  goal.trajectory.points[index].positions[3] = 1.7;
  goal.trajectory.points[index].positions[4] = -1.57;
  goal.trajectory.points[index].positions[5] = 0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  goalgrip.trajectory.points[index].positions.resize(gripJoints);
  goalgrip.trajectory.points[index].positions[0] = 0.00;
  goalgrip.trajectory.points[index].positions[1] = 0.00;
  // Velocities
  goal.trajectory.points[index].velocities.resize(numJoints);
  goalgrip.trajectory.points[index].velocities.resize(gripJoints);
  for (int j = 0; j < numJoints; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  for (int i = 0; i < gripJoints; i++)
  {
    goalgrip.trajectory.points[index].velocities[i] = 0.0;
  }
  goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
  goalgrip.trajectory.points[index].time_from_start = ros::Duration(4.0);
  
  // 3rd trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(numJoints);
  goal.trajectory.points[index].positions[0] = 0.74;
  goal.trajectory.points[index].positions[1] = -0.14; //0.14 in the real robot
  goal.trajectory.points[index].positions[2] = 1.39;
  goal.trajectory.points[index].positions[3] = 1.76;
  goal.trajectory.points[index].positions[4] = -1.12;
  goal.trajectory.points[index].positions[5] = 0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  goalgrip.trajectory.points[index].positions.resize(gripJoints);
  goalgrip.trajectory.points[index].positions[0] = 0.00;
  goalgrip.trajectory.points[index].positions[1] = 0.00;
  // Velocities
  goal.trajectory.points[index].velocities.resize(numJoints);
  goalgrip.trajectory.points[index].velocities.resize(gripJoints);
  for (int j = 0; j < numJoints; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  for (int i = 0; i < gripJoints; i++)
  {
    goalgrip.trajectory.points[index].velocities[i] = 0.0;
  }
  // To be reached 8 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(8.0);
  goalgrip.trajectory.points[index].time_from_start = ros::Duration(8.0);
  // 4th trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(numJoints);
  goal.trajectory.points[index].positions[0] = 0.74;
  goal.trajectory.points[index].positions[1] = -1.11; //1.11 in the real robot
  goal.trajectory.points[index].positions[2] = 1.39;
  goal.trajectory.points[index].positions[3] = 1.76;
  goal.trajectory.points[index].positions[4] = -1.12;
  goal.trajectory.points[index].positions[5] = 0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  goalgrip.trajectory.points[index].positions.resize(gripJoints);
  goalgrip.trajectory.points[index].positions[0] = 0.04;
  goalgrip.trajectory.points[index].positions[1] = 0.04;
  // Velocities
  goal.trajectory.points[index].velocities.resize(numJoints);
  goalgrip.trajectory.points[index].velocities.resize(gripJoints);
  for (int j = 0; j < numJoints; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  for (int i = 0; i < gripJoints; i++)
  {
    goalgrip.trajectory.points[index].velocities[i] = 0.0;
  }
  // To be reached 10 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(10.0);
  goalgrip.trajectory.points[index].time_from_start = ros::Duration(10.0);
  // 5th trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(numJoints);
  goal.trajectory.points[index].positions[0] = 0.74;
  goal.trajectory.points[index].positions[1] = -1.11; //1.11 in the real robot
  goal.trajectory.points[index].positions[2] = 1.39;
  goal.trajectory.points[index].positions[3] = 1.76;
  goal.trajectory.points[index].positions[4] = -1.12;
  goal.trajectory.points[index].positions[5] = 0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  goalgrip.trajectory.points[index].positions.resize(gripJoints);
  goalgrip.trajectory.points[index].positions[0] = 0.04;
  goalgrip.trajectory.points[index].positions[1] = 0.04;
  // Velocities
  goal.trajectory.points[index].velocities.resize(numJoints);
  goalgrip.trajectory.points[index].velocities.resize(gripJoints);
  for (int j = 0; j < numJoints; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  for (int i = 0; i < gripJoints; i++)
  {
    goalgrip.trajectory.points[index].velocities[i] = 0.0;
  }
  // To be reached 10 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(12.0);
  goalgrip.trajectory.points[index].time_from_start = ros::Duration(12.0);
}
void waypointsArmLeftGoal(control_msgs::FollowJointTrajectoryGoal& goal, control_msgs::FollowJointTrajectoryGoal& goalgrip)
{
  // The arm joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_left_1_joint");
  goal.trajectory.joint_names.push_back("arm_left_2_joint");
  goal.trajectory.joint_names.push_back("arm_left_3_joint");
  goal.trajectory.joint_names.push_back("arm_left_4_joint");
  goal.trajectory.joint_names.push_back("arm_left_5_joint");
  goal.trajectory.joint_names.push_back("arm_left_6_joint");
  goal.trajectory.joint_names.push_back("arm_left_7_joint");

  int numJoints = 7;

  //  The gripper joint names
  goalgrip.trajectory.joint_names.push_back("gripper_left_left_finger_joint");
  goalgrip.trajectory.joint_names.push_back("gripper_left_right_finger_joint");

  int gripJoints = 2;  
  
  // 5 waypoints in this goal trajectory
  int numPoints = 5;
  goal.trajectory.points.resize(numPoints);
  goalgrip.trajectory.points.resize(numPoints);

  // 1st trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(numJoints);
  goal.trajectory.points[index].positions[0] = -1.1;
  goal.trajectory.points[index].positions[1] = 1.46;
  goal.trajectory.points[index].positions[2] = 2.72;
  goal.trajectory.points[index].positions[3] = 1.7;
  goal.trajectory.points[index].positions[4] = -1.57;
  goal.trajectory.points[index].positions[5] = 1.37;
  goal.trajectory.points[index].positions[6] = 0.0;
  goalgrip.trajectory.points[index].positions.resize(gripJoints);
  goalgrip.trajectory.points[index].positions[0] = 0.04;
  goalgrip.trajectory.points[index].positions[1] = 0.04;
  // Velocities
  goal.trajectory.points[index].velocities.resize(numJoints);
  goalgrip.trajectory.points[index].velocities.resize(gripJoints);
  for (int j = 0; j < numJoints; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  for (int i = 0; i < gripJoints; i++)
  {
    goalgrip.trajectory.points[index].velocities[i] = 0.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
  goalgrip.trajectory.points[index].time_from_start = ros::Duration(2.0);

  // 2nd trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(numJoints);
  goal.trajectory.points[index].positions[0] = -1.1;
  goal.trajectory.points[index].positions[1] = 1.46;
  goal.trajectory.points[index].positions[2] = 2.72;
  goal.trajectory.points[index].positions[3] = 1.7;
  goal.trajectory.points[index].positions[4] = -1.57;
  goal.trajectory.points[index].positions[5] = 0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  goalgrip.trajectory.points[index].positions.resize(gripJoints);
  goalgrip.trajectory.points[index].positions[0] = 0.00;
  goalgrip.trajectory.points[index].positions[1] = 0.00;
  // Velocities
  goal.trajectory.points[index].velocities.resize(numJoints);
  goalgrip.trajectory.points[index].velocities.resize(gripJoints);
  for (int j = 0; j < numJoints; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  for (int i = 0; i < gripJoints; i++)
  {
    goalgrip.trajectory.points[index].velocities[i] = 0.0;
  }
  goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
  goalgrip.trajectory.points[index].time_from_start = ros::Duration(4.0);

  // 3rd trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(numJoints);
  goal.trajectory.points[index].positions[0] = -1.1;
  goal.trajectory.points[index].positions[1] = 0.0;
  goal.trajectory.points[index].positions[2] = 2.72;
  goal.trajectory.points[index].positions[3] = 1.7;
  goal.trajectory.points[index].positions[4] = -1.57;
  goal.trajectory.points[index].positions[5] = 0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  goalgrip.trajectory.points[index].positions.resize(gripJoints);
  goalgrip.trajectory.points[index].positions[0] = 0.00;
  goalgrip.trajectory.points[index].positions[1] = 0.00;
  // Velocities
  goal.trajectory.points[index].velocities.resize(numJoints);
  goalgrip.trajectory.points[index].velocities.resize(gripJoints);
  for (int j = 0; j < numJoints; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  for (int i = 0; i < gripJoints; i++)
  {
    goalgrip.trajectory.points[index].velocities[i] = 0.0;
  }
  goal.trajectory.points[index].time_from_start = ros::Duration(6.0);
  goalgrip.trajectory.points[index].time_from_start = ros::Duration(6.0);

  // 4th trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(numJoints);
  goal.trajectory.points[index].positions[0] = 0.9;
  goal.trajectory.points[index].positions[1] = 0.0;
  goal.trajectory.points[index].positions[2] = 1.57;
  goal.trajectory.points[index].positions[3] = 1.7;
  goal.trajectory.points[index].positions[4] = 1.57;
  goal.trajectory.points[index].positions[5] = 0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  goalgrip.trajectory.points[index].positions.resize(gripJoints);
  goalgrip.trajectory.points[index].positions[0] = 0.00;
  goalgrip.trajectory.points[index].positions[1] = 0.00;
  // Velocities
  goal.trajectory.points[index].velocities.resize(numJoints);
  goalgrip.trajectory.points[index].velocities.resize(gripJoints);
  for (int j = 0; j < numJoints; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  for (int i = 0; i < gripJoints; i++)
  {
    goalgrip.trajectory.points[index].velocities[i] = 0.0;
  }
  goal.trajectory.points[index].time_from_start = ros::Duration(8.0);
  goalgrip.trajectory.points[index].time_from_start = ros::Duration(8.0);
  // 5th trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(numJoints);
  goal.trajectory.points[index].positions[0] = 0.9;
  goal.trajectory.points[index].positions[1] = -0.04;
  goal.trajectory.points[index].positions[2] = 1.39;
  goal.trajectory.points[index].positions[3] = 1.7;
  goal.trajectory.points[index].positions[4] = 1.57;
  goal.trajectory.points[index].positions[5] = 0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  goalgrip.trajectory.points[index].positions.resize(gripJoints);
  goalgrip.trajectory.points[index].positions[0] = 0.00;
  goalgrip.trajectory.points[index].positions[1] = 0.00;
  // Velocities
  goal.trajectory.points[index].velocities.resize(numJoints);
  goalgrip.trajectory.points[index].velocities.resize(gripJoints);
  for (int j = 0; j < numJoints; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  for (int i = 0; i < gripJoints; i++)
  {
    goalgrip.trajectory.points[index].velocities[i] = 0.0;
  }
  goal.trajectory.points[index].time_from_start = ros::Duration(10.0);
  goalgrip.trajectory.points[index].time_from_start = ros::Duration(10.0);
}

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "run_dual_traj_control");

  ROS_INFO("Starting run_dual_traj_control application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Create a torso controller action client to move the TIAGO's torso
  torso_control_client_Ptr torso_client;
  createTorsoClient(torso_client, "torso_controller");

  // Create an arm left controller action client to move the TIAGo's left arm
  arm_control_client_Ptr arm_left_client;
  createArmClient(arm_left_client, "arm_left_controller");

  // Create an arm right controller action client to move the TIAGo's right arm
  arm_control_client_Ptr arm_right_client;
  createArmClient(arm_right_client, "arm_right_controller");

  // Create a gripper left controller action client to move the TIAGO's left gripper
  gripper_control_client_Ptr gripper_left_client;
  createGripperClient(gripper_left_client, "gripper_left_controller");

  // Create a gripper right controller action client to move the TIAGO's right gripper
  gripper_control_client_Ptr gripper_right_client;
  createGripperClient(gripper_right_client, "gripper_right_controller");

  // Generates all the goals for the TIAGo's parts
  control_msgs::FollowJointTrajectoryGoal torso_goal;
  waypointsTorso(torso_goal); // Sets waypoints for torso, works ok
  control_msgs::FollowJointTrajectoryGoal arm_right_goal;
  control_msgs::FollowJointTrajectoryGoal gripper_right_goal;
  waypointsArmRightGoal(arm_right_goal, gripper_right_goal); // Sets waypoints for right arm&gripper, works ok
  control_msgs::FollowJointTrajectoryGoal arm_left_goal;
  control_msgs::FollowJointTrajectoryGoal gripper_left_goal;
  waypointsArmLeftGoal(arm_left_goal, gripper_left_goal); // Sets waypoints for left arm&gripper, works ok
 
  // Setting timestamps
  ros::Duration smallPause(1); // 1s
  ros::Duration torsoTime(14); // 14s
  torso_goal.trajectory.header.stamp = ros::Time::now() + smallPause; // 1 second from now
  arm_right_goal.trajectory.header.stamp = torso_goal.trajectory.header.stamp + torsoTime; //delay arms so that the torso can move up first
  // Sincronice both arms and its grippers
  arm_left_goal.trajectory.header.stamp = arm_right_goal.trajectory.header.stamp; // Sincronice both arms
  gripper_right_goal.trajectory.header.stamp = arm_right_goal.trajectory.header.stamp; // Sincronice gripper
  gripper_left_goal.trajectory.header.stamp = arm_left_goal.trajectory.header.stamp; // Sincronice gripper
  ROS_INFO("Stamp torso: %f", torso_goal.trajectory.header.stamp.toSec()); // check if the delay is ok, tested and works
  ROS_INFO("Stamp arms: %f", arm_right_goal.trajectory.header.stamp.toSec()); // check if the delay is ok tested and works
  // Send all goals to the clients
  torso_client->sendGoal(torso_goal);
  arm_right_client->sendGoal(arm_right_goal);
  arm_left_client->sendGoal(arm_left_goal);
  gripper_right_client->sendGoal(gripper_right_goal);
  gripper_left_client->sendGoal(gripper_left_goal);
  // Wait for trajectory execution
  while(!(arm_right_client->getState().isDone()) && ros::ok() && !(arm_left_client->getState().isDone())  && !(gripper_right_client->getState().isDone()) && !(gripper_left_client->getState().isDone()) && !(torso_client->getState().isDone()))
  {
    ros::Duration(4).sleep(); // sleep for four seconds
  }

  return EXIT_SUCCESS;
}
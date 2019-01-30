#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

//usage:
// roslaunch mobilican komodo_2.launch gazebo:=true lidar:=true move_base:=true amcl:=true world_name:="/home/hila/catkin_ws/src/mobilican/mobilican_gazebo/worlds/rooms.world" have_map:=true map:="/home/hila/catkin_ws/src/mobilican/mobilican_navigation/maps/rooms.yaml"
//rviz

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal goal;


void createGoalToMoveBase(double x, double y, double Y)
{

  // 'frame_id' is the coordinate system to which we send the goal.
  // If we select "base_link" instead of "map", each time the goal will be in relative to the robot.
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // z will be 0 by default.
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  //  goal.target_pose.pose.orientation.w = 1.0;

  // Convert the Euler angle to quaternion
  double radians_ = Y * (M_PI / 180);
  tf::Quaternion quaternion;
  quaternion = tf::createQuaternionFromYaw(radians_); // Create this quaternion from yaw (in radians)
  ROS_INFO_STREAM(quaternion);

  geometry_msgs::Quaternion quaternion_msg_;
  tf::quaternionTFToMsg(quaternion, quaternion_msg_);

  goal.target_pose.pose.orientation = quaternion_msg_;
}

/* Send a goal to the robot to move to a specific location */
void publishGoal(MoveBaseClient& ac, double x, double y, double Y)
{
  createGoalToMoveBase(x, y, Y);
  ROS_INFO("Sending goal: x = %f, y = %f, Y = %f", x, y, Y);
  ac.sendGoal(goal);

  // Wait for the action to return
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("You have reached the goal!");
  } else
    {
    ROS_INFO("The base failed for some reason");
  }
}

// A client that send a goal (a name of a location) to the "move_base" node,
// which plays here as a server. 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_client");

  // create the action client that send to "move_base" server
  // true causes the client to spin its own thread
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer())
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  double x = 0.47;
  double y = 5.03;
  double Y = 0.00161; // angle parameter
  publishGoal(ac, x, y, Y);

  return 0;
}
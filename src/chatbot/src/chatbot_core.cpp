#include "chatbot_core.hpp"
#include <string>
#include <boost/asio.hpp>

using namespace std;

ChatCore::ChatCore() :
		as_(nh_, "goto", boost::bind(&ChatCore::executeGotoCB, this, _1), false)
{
	as_.start();
}


void ChatCore::executeGotoCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
{
    bool success = true;
    auto x = goal->target_pose.pose.position.x;
    auto y = goal->target_pose.pose.position.y;
    // publish info to the console for the user
    ROS_INFO("ChatCore: goto(%.2f,%.2f) received from prolog!", x, y);

    // start executing the action

    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
    	ROS_INFO("ChatCore: goto cancelled!");
        // set the action state to preempted
        as_.setPreempted();
        success = false;
    }
    ros::Duration(20).sleep();

    if(success)
    {
      ROS_INFO("ChatCore: goto completed!");
  	  move_base_msgs::MoveBaseResult res;
      as_.setSucceeded(res, "success");
    }
    else
    {
        ROS_INFO("ChatCore:goto preempted!");
    	move_base_msgs::MoveBaseResult res;
        as_.setPreempted(res, "preempted");
    }
}

ChatCore::~ChatCore()
{
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "chat_core");
	ChatCore server;
	ros::spin();
	return 0;
}


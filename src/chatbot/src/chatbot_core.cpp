#include "chatbot_core.hpp"
#include <string>
#include <boost/asio.hpp>

using namespace std;

ChatCore::ChatCore() :
	gotoAs_(nh_, "goto", boost::bind(&ChatCore::executeGotoCB, this, _1), false),
	pickAs_(nh_, "pick", boost::bind(&ChatCore::executePickCB, this, _1), false),
	placeAs_(nh_, "place", boost::bind(&ChatCore::executePlaceCB, this, _1), false),
	gotoAc_(nh_, "move_base", false)
{
	gotoAs_.start();
	pickAs_.start();
	placeAs_.start();
	respondSrv_ = nh_.advertiseService("/respond", &ChatCore::executeRespond, this);
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
    if (gotoAs_.isPreemptRequested() || !ros::ok())
    {
    	ROS_INFO("ChatCore: goto cancelled!");
        // set the action state to preempted
        gotoAs_.setPreempted();
        success = false;
    }

    while(!gotoAc_.waitForServer(ros::Duration(5.0)))
      ROS_INFO("Goto received, but no move_base server. Waiting for the move_base action server to come up");

    ROS_INFO("Sending goal");
    gotoAc_.sendGoal(*goal);
    gotoAc_.waitForResult();

    if(success && gotoAc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("ChatCore: goto completed!");
  	  move_base_msgs::MoveBaseResult res;
      gotoAs_.setSucceeded(res, "success");
    }
    else
    {
        ROS_INFO("ChatCore:goto preempted!");
    	move_base_msgs::MoveBaseResult res;
        gotoAs_.setPreempted(res, "preempted");
    }
}

void ChatCore::executePickCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
{
    bool success = true;
    auto x = goal->target_pose.pose.position.x;
    auto y = goal->target_pose.pose.position.y;
    // publish info to the console for the user
    ROS_INFO("ChatCore: pick(%.2f,%.2f) received from prolog!", x, y);

    // start executing the action

    // check that preempt has not been requested by the client
    if (pickAs_.isPreemptRequested() || !ros::ok())
    {
    	ROS_INFO("ChatCore: pick cancelled!");
        // set the action state to preempted
        pickAs_.setPreempted();
        success = false;
    }
    ros::Duration(20).sleep();

    if(success)
    {
      ROS_INFO("ChatCore: pick completed!");
  	  move_base_msgs::MoveBaseResult res;
      pickAs_.setSucceeded(res, "success");
    }
    else
    {
        ROS_INFO("ChatCore:pick preempted!");
    	move_base_msgs::MoveBaseResult res;
        pickAs_.setPreempted(res, "preempted");
    }
}

void ChatCore::executePlaceCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
{
    bool success = true;
    auto x = goal->target_pose.pose.position.x;
    auto y = goal->target_pose.pose.position.y;
    // publish info to the console for the user
    ROS_INFO("ChatCore: place(%.2f,%.2f) received from prolog!", x, y);

    // start executing the action

    // check that preempt has not been requested by the client
    if (placeAs_.isPreemptRequested() || !ros::ok())
    {
    	ROS_INFO("ChatCore: place cancelled!");
        // set the action state to preempted
        placeAs_.setPreempted();
        success = false;
    }
    ros::Duration(20).sleep();

    if(success)
    {
      ROS_INFO("ChatCore: place completed!");
  	  move_base_msgs::MoveBaseResult res;
      placeAs_.setSucceeded(res, "success");
    }
    else
    {
        ROS_INFO("ChatCore:place preempted!");
    	move_base_msgs::MoveBaseResult res;
        placeAs_.setPreempted(res, "preempted");
    }
}

bool ChatCore::executeRespond(chatbot::RespondRequest &req, chatbot::RespondResponse & res)
{
	ROS_INFO("ChatCore:responding: %s", req.str.c_str());
	return true;
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


#include "chatbot_core.hpp"
#include <string>
#include <boost/asio.hpp>

using namespace std;

ChatCore::ChatCore() :
	gotoAs_(nh_, "goto", false),
	pickAs_(nh_, "pick", false),
	placeAs_(nh_, "place", false),
	gotoAc_(nh_, "move_base", false),
	pickSub_(nh_.subscribe("picker", 1, &ChatCore::pickDoneCb, this)),
	placeSub_(nh_.subscribe("placer", 1, &ChatCore::placeDoneCb, this)),
	respondPub_(nh_.advertise<std_msgs::String>("responder", 10))
{
	gotoAs_.registerGoalCallback( boost::bind(&ChatCore::goalGotoCB, this));
	gotoAs_.registerPreemptCallback(boost::bind(&ChatCore::preemptGotoCB, this));

	pickAs_.registerGoalCallback( boost::bind(&ChatCore::goalPickCB, this));
	pickAs_.registerPreemptCallback(boost::bind(&ChatCore::preemptPickCB, this));

	placeAs_.registerGoalCallback( boost::bind(&ChatCore::goalPlaceCB, this));
	placeAs_.registerPreemptCallback(boost::bind(&ChatCore::preemptPlaceCB, this));

	gotoAs_.start();
	pickAs_.start();
	placeAs_.start();
	respondSrv_ = nh_.advertiseService("/respond", &ChatCore::executeRespond, this);
}

void ChatCore::goalGotoCB()
{
	chatbot::NamedMoveBaseGoalConstPtr goal = gotoAs_.acceptNewGoal();
	executeGoto(goal);
}

void ChatCore::preemptGotoCB()
{
	ROS_WARN("ChatCore:goto preempted!");
	ROS_WARN("ChatCore:cancelling navigation.");
	gotoAc_.cancelGoal();
}

void ChatCore::executeGoto(const chatbot::NamedMoveBaseGoalConstPtr &goal)
{
	if(!gotoAs_.isActive())
		return;

    auto x = goal->goal_move_base.target_pose.pose.position.x;
	auto y = goal->goal_move_base.target_pose.pose.position.y;
	auto name = goal->goal_name;
	// publish info to the console for the user
	ROS_INFO("ChatCore::goto: %s[%.2f,%.2f] received from prolog!", name.c_str(), x, y);

	// start executing the action
	while (!gotoAc_.waitForServer(ros::Duration(5.0)))
		ROS_WARN("ChatCore:goto: goto received, but no move_base server. Waiting for the move_base action server to come up");

	ROS_INFO("ChatCore::goto: sending goal to planner.");
	gotoAc_.sendGoal(goal->goal_move_base, boost::bind(&ChatCore::navDoneCB, this, _1, _2));
}


void ChatCore::navDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
	ROS_INFO("ChatCore:navDoneCB()");

	// check that preempt has not been requested by the client
	if (gotoAs_.isPreemptRequested() || !ros::ok()) {
		ROS_WARN("ChatCore::navDoneCB: goto was lost!");
		// set the action state to preempted
		gotoAs_.setPreempted();
		return;
	}

	if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("ChatCore::navDoneCB: goto completed!");
		gotoAs_.setSucceeded();
	} else {
		ROS_WARN("ChatCore::navDoneCB: goto failed!");
		gotoAs_.setAborted();
	}
}

void ChatCore::goalPickCB()
{
	chatbot::NamedMoveBaseGoalConstPtr goal = pickAs_.acceptNewGoal();
	executePick(goal);
}

void ChatCore::preemptPickCB()
{
	ROS_WARN("ChatCore::preemptPickCB!");
	pickAs_.setPreempted();
}

void ChatCore::executePick(const chatbot::NamedMoveBaseGoalConstPtr &goal)
{
    auto x = goal->goal_move_base.target_pose.pose.position.x;
    auto y = goal->goal_move_base.target_pose.pose.position.y;
    auto name = goal->goal_name;
    // publish info to the console for the user
    ROS_INFO("ChatCore::pick: %s[%.2f,%.2f] received from prolog!", name.c_str(), x, y);
}

void ChatCore::pickDoneCb(const std_msgs::String::ConstPtr& msg)
{
	if(!pickAs_.isActive())
	{
		ROS_WARN("ChatCore::pickDoneCb: %s but pick action was not active!", msg->data.c_str());
		return;
	}

	ROS_INFO("ChatCore:pickDoneCb(): %s", msg->data.c_str());

	// check that preempt has not been requested by the client
	if (pickAs_.isPreemptRequested() || !ros::ok()) {
		ROS_WARN("ChatCore::pickDoneCb: pick was lost!");
		// set the action state to preempted
		pickAs_.setPreempted();
		return;
	}

	if (msg->data.size()) {
		ROS_INFO("ChatCore::pickDoneCb: pick completed!");
		pickAs_.setSucceeded();
	} else {
		ROS_WARN("ChatCore::pickDoneCb: pick failed!");
		pickAs_.setAborted();
	}
}

void ChatCore::goalPlaceCB()
{
	chatbot::NamedMoveBaseGoalConstPtr goal = placeAs_.acceptNewGoal();
	executePlace(goal);
}

void ChatCore::preemptPlaceCB()
{
	ROS_WARN("ChatCore::preemptPlaceCB!");
	placeAs_.setPreempted();
}

void ChatCore::executePlace(const chatbot::NamedMoveBaseGoalConstPtr &goal)
{
    auto x = goal->goal_move_base.target_pose.pose.position.x;
    auto y = goal->goal_move_base.target_pose.pose.position.y;
    auto name = goal->goal_name;
    // publish info to the console for the user
    ROS_INFO("ChatCore::place: %s[%.2f,%.2f] received from prolog!", name.c_str(), x, y);
}

void ChatCore::placeDoneCb(const std_msgs::String::ConstPtr& msg)
{
	if(!placeAs_.isActive())
	{
		ROS_WARN("ChatCore::placeDoneCb: %s but place action was not active!", msg->data.c_str());
		return;
	}

	ROS_INFO("ChatCore:placeDoneCb(): %s", msg->data.c_str());

	// check that preempt has not been requested by the client
	if (placeAs_.isPreemptRequested() || !ros::ok()) {
		ROS_WARN("ChatCore::placeDoneCb: place was lost!");
		// set the action state to preempted
		placeAs_.setPreempted();
		return;
	}

	if (msg->data.size()) {
		ROS_INFO("ChatCore::placeDoneCb: place completed!");
		placeAs_.setSucceeded();
	} else {
		ROS_WARN("ChatCore::placeDoneCb: place failed!");
		placeAs_.setAborted();
	}
}

bool ChatCore::executeRespond(chatbot::RespondRequest &req, chatbot::RespondResponse & res)
{
	ROS_INFO("ChatCore:respond: %s", req.str.c_str());
	respondPub_.publish(req.str.c_str());
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


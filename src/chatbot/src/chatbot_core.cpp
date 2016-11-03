#include "chatbot_core.hpp"
#include <string>
#include <boost/asio.hpp>
#include <boost/format.hpp>

using namespace std;
using boost::format;
using boost::str;

ChatCore::ChatCore() :
	gotoAs_(nh_, "goto", false),
	pickAs_(nh_, "pick", false),
	placeAs_(nh_, "place", false),
	gotoAc_(nh_, "move_base", false),
	pickSub_(nh_.subscribe("picker", 1, &ChatCore::pickDoneCb, this)),
	placeSub_(nh_.subscribe("placer", 1, &ChatCore::placeDoneCb, this)),
	respondPub_(nh_.advertise<std_msgs::String>("synthesizer/text_to_speak", 10)),
	statusPub_(nh_.advertise<std_msgs::String>("chatbot_gui/status", 10)),
	logPub_(nh_.advertise<std_msgs::String>("chatbot_gui/log", 10)),
	respondSrv_(nh_.advertiseService("/respond", &ChatCore::executeRespond, this))
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
	sendLog("goto preempted, canceling navigation");
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
	sendLog(str(format("goto: %1%[%2%,%3%]") % name % x % y));

	// start executing the action
	while (!gotoAc_.waitForServer(ros::Duration(5.0)))
	{
		sendLog("waiting for move_base server to come up.");
		sendStatus("Waiting...");
		ROS_WARN("ChatCore:goto: goto received, but no move_base server. Waiting for the move_base action server to come up");
	}
	ROS_INFO("ChatCore::goto: sending goal to planner.");
	sendStatus(str(format("Navigating to %1%") % name));
	gotoAc_.sendGoal(goal->goal_move_base, boost::bind(&ChatCore::navDoneCB, this, _1, _2));
}


void ChatCore::navDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
	ROS_INFO("ChatCore:navDoneCB()");
	sendStatus("Idle");
	// check that preempt has not been requested by the client
	if (gotoAs_.isPreemptRequested() || !ros::ok()) {
		ROS_WARN("ChatCore::navDoneCB: goto was lost!");
		sendLog("goto preempted!");
		// set the action state to preempted
		gotoAs_.setPreempted();
		return;
	}

	if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("ChatCore::navDoneCB: goto completed!");
		sendLog("goto completed.");
		gotoAs_.setSucceeded();
	} else {
		ROS_WARN("ChatCore::navDoneCB: goto failed!");
		sendLog("goto failed!");
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
    sendLog(str(format("pick: %1%[%2%,%3%]") % name % x % y));
    sendStatus(str(format("Picking up %1%") % name));
    ROS_INFO("ChatCore::pick: %s[%.2f,%.2f] received from prolog!", name.c_str(), x, y);
}

void ChatCore::pickDoneCb(const std_msgs::String::ConstPtr& msg)
{
	sendStatus("Idle");
	if(!pickAs_.isActive())
	{
		ROS_WARN("ChatCore::pickDoneCb: %s but pick action was not active!", msg->data.c_str());
		return;
	}

	ROS_INFO("ChatCore:pickDoneCb(): %s", msg->data.c_str());

	// check that preempt has not been requested by the client
	if (pickAs_.isPreemptRequested() || !ros::ok()) {
		ROS_WARN("ChatCore::pickDoneCb: pick was lost!");
		sendLog("pick preempted!");
		// set the action state to preempted
		pickAs_.setPreempted();
		return;
	}

	if (msg->data.size()) {
		ROS_INFO("ChatCore::pickDoneCb: pick completed!");
		sendLog("pick completed.");
		pickAs_.setSucceeded();
	} else {
		ROS_WARN("ChatCore::pickDoneCb: pick failed!");
		sendLog("pick failed!");
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
    sendLog(str(format("place: %1%[%2%,%3%]") % name % x % y));
    sendStatus(str(format("Placing down %1%") % name));
    ROS_INFO("ChatCore::place: %s[%.2f,%.2f] received from prolog!", name.c_str(), x, y);
}

void ChatCore::placeDoneCb(const std_msgs::String::ConstPtr& msg)
{
	sendStatus("Idle");
	if(!placeAs_.isActive())
	{
		ROS_WARN("ChatCore::placeDoneCb: %s but place action was not active!", msg->data.c_str());
		return;
	}

	ROS_INFO("ChatCore:placeDoneCb(): %s", msg->data.c_str());

	// check that preempt has not been requested by the client
	if (placeAs_.isPreemptRequested() || !ros::ok()) {
		ROS_WARN("ChatCore::placeDoneCb: place was lost!");
		sendLog("place preempted!");
		// set the action state to preempted
		placeAs_.setPreempted();
		return;
	}

	if (msg->data.size()) {
		ROS_INFO("ChatCore::placeDoneCb: place completed!");
		sendLog("place completed.");
		placeAs_.setSucceeded();
	} else {
		ROS_WARN("ChatCore::placeDoneCb: place failed!");
		sendLog("place failed!");
		placeAs_.setAborted();
	}
}


bool ChatCore::executeRespond(chatbot::RespondRequest &req, chatbot::RespondResponse & res)
{
	ROS_INFO("ChatCore:respond: %s", req.str.c_str());
	sendLog(str(format("response:%1%") % req.str));
	std_msgs::String msg;
	msg.data = req.str;
	respondPub_.publish(msg);
	return true;
}

void ChatCore::sendLog(const std::string& line)
{
	std_msgs::String msg;
	msg.data = line;
	logPub_.publish(msg);
}

void ChatCore::sendStatus(const std::string& status)
{
	std_msgs::String msg;
	msg.data = status;
	statusPub_.publish(msg);
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


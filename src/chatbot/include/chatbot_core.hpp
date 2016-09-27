using namespace std;
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include "chatbot/Respond.h"
#include "chatbot/NamedMoveBaseAction.h"
class ChatCore
{
public:
	ChatCore();
	~ChatCore();
protected:

	void goalGotoCB();
	void preemptGotoCB();
	void executeGoto(const chatbot::NamedMoveBaseGoalConstPtr &goal);
	void navDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr &result);

	void goalPickCB();
	void preemptPickCB();
	void executePick(const chatbot::NamedMoveBaseGoalConstPtr &goal);
	void pickDoneCb(const std_msgs::String::ConstPtr& item);

	void goalPlaceCB();
	void preemptPlaceCB();
	void executePlace(const chatbot::NamedMoveBaseGoalConstPtr &goal);
	void placeDoneCb(const std_msgs::String::ConstPtr& item);

	bool executeRespond(chatbot::RespondRequest &req, chatbot::RespondResponse & res);


	ros::NodeHandle nh_;

	actionlib::SimpleActionServer<chatbot::NamedMoveBaseAction> gotoAs_;
	actionlib::SimpleActionServer<chatbot::NamedMoveBaseAction> pickAs_;
	actionlib::SimpleActionServer<chatbot::NamedMoveBaseAction> placeAs_;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> gotoAc_;
	ros::Subscriber pickSub_;
	ros::Subscriber placeSub_;
	ros::Publisher respondPub_;

	ros::ServiceServer respondSrv_;
};

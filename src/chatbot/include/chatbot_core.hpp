using namespace std;
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "chatbot/Respond.h"
#include "chatbot/NamedMoveBaseAction.h"
class ChatCore
{
public:
	ChatCore();
	~ChatCore();
protected:
	void executeGotoCB(const chatbot::NamedMoveBaseGoalConstPtr &goal);
	void executePickCB(const chatbot::NamedMoveBaseGoalConstPtr &goal);
	void executePlaceCB(const chatbot::NamedMoveBaseGoalConstPtr &goal);
	bool executeRespond(chatbot::RespondRequest &req, chatbot::RespondResponse & res);

	ros::NodeHandle nh_;

	actionlib::SimpleActionServer<chatbot::NamedMoveBaseAction> gotoAs_;
	actionlib::SimpleActionServer<chatbot::NamedMoveBaseAction> pickAs_;
	actionlib::SimpleActionServer<chatbot::NamedMoveBaseAction> placeAs_;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> gotoAc_;
	ros::ServiceServer respondSrv_;
};

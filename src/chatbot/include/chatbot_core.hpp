using namespace std;
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "chatbot/Respond.h"

class ChatCore
{
public:
	ChatCore();
	~ChatCore();
protected:
	void executeGotoCB(const move_base_msgs::MoveBaseGoalConstPtr &goal);
	void executePickCB(const move_base_msgs::MoveBaseGoalConstPtr &goal);
	void executePlaceCB(const move_base_msgs::MoveBaseGoalConstPtr &goal);
	bool executeRespond(chatbot::RespondRequest &req, chatbot::RespondResponse & res);

	ros::NodeHandle nh_;

	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> gotoAs_;
	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> pickAs_;
	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> placeAs_;
	ros::ServiceServer respondSrv_;
};

using namespace std;
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

class ChatCore
{
public:
	ChatCore();
	~ChatCore();
protected:
	void executeGotoCB(const move_base_msgs::MoveBaseGoalConstPtr &goal);

	ros::NodeHandle nh_;

	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;
};

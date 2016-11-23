#pragma once
#include <string>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <chatbot/NamedMoveBaseAction.h>

class RobotPrologConnectionInterface
{
public:
	virtual ~RobotPrologConnectionInterface() {}
	virtual std::string getOutput() = 0;
	virtual void cleanOutput() = 0;
	virtual std::string getTask() = 0;
	virtual void cleanTask() = 0;
	virtual bool doQuery(const std::string& queryString) = 0;
	virtual void currentTaskComplete() = 0;
	virtual void abortAllTasks() = 0;

	virtual ros::ServiceClient& getRespondClient() = 0;
	virtual actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& getGotoActionClient() = 0;
	virtual actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& getPickActionClient() = 0;
	virtual actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& getPlaceActionClient() = 0;
};

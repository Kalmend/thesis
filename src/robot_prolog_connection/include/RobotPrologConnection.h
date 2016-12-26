#pragma once
#include "RobotPrologConnectionInterface.h"
#include "PrioritizedTask.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <chatbot/NamedMoveBaseAction.h>
#include <string>
#include <vector>
#include <queue>
#include <memory>

#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>

#include <Client.h>
#include <std_msgs/String.h>

class TaskCompare
{
public:
    bool operator() (std::shared_ptr<PrioritizedTask>& a, std::shared_ptr<PrioritizedTask>& b)
    {
        return (a->priority) > (b->priority);
    }
};

class RobotPrologConnection: public prolog::client::Client, public RobotPrologConnectionInterface
{
public:

	RobotPrologConnection();
	virtual ~RobotPrologConnection();


	virtual std::string getOutput();
	virtual void cleanOutput();
	virtual std::string getTask();
	virtual void cleanTask();

	virtual bool doQuery(const std::string& queryString);
	virtual void currentTaskComplete();
	virtual void abortAllTasks();

	virtual ros::ServiceClient& getRespondClient();
	virtual actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& getGotoActionClient();
	virtual actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& getPickActionClient();
	virtual actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& getPlaceActionClient();
protected:

	void onRawSpeech(const std_msgs::String &msg);

private:
	void init();
	void initChatCore();
	void initRobotCore();
	void initInputOutput();
	void cleanup();

	void startInput();
	void handleInput(const boost::system::error_code& error, size_t length);

	//calls to prolog
	bool queryReaction(const std::string& input);

	//calls to robot

	//helpers and file handling
	std::string getCommaSeparatedString(std::string input) const;
	std::string trimGarbage(std::string  raw) const;
	void toInput(const std::string& input);

	void scheduleProcessAsyncQueue();
	void processAsyncQueue();
	void handleTask();

	std::string getFile(const std::string& file);
	void cleanFile(const std::string& file);

	prolog::client::ServiceClient serviceClient_;
	boost::asio::io_service ioService_;
	boost::thread ioThread_;
	boost::asio::posix::stream_descriptor inputStream_;
	boost::asio::streambuf inputBuffer_;
	size_t inputColumn_;

	ros::NodeHandle nh_;
	ros::Subscriber sub_;

	unsigned int queryNumber_;
	std::string inputFile_;
	std::string outputFile_;
	std::string taskFile_;
	std::priority_queue<std::shared_ptr<PrioritizedTask>, std::vector<std::shared_ptr<PrioritizedTask>>, TaskCompare> queuedTasks_;
	std::shared_ptr<PrioritizedTask> currentTask_;

	actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction> gotoAc_;
	actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction> pickAc_;
	actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction> placeAc_;
	ros::ServiceClient respondClient_;
};

#pragma once
#include "prolog_client/PrioritizedTask.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <chatbot/NamedMoveBaseAction.h>
#include <string>
#include <vector>
#include <queue>

#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>

#include <prolog_client/Client.h>
#include <std_msgs/String.h>

namespace prolog
{
namespace client
{

class TaskCompare
{
public:
    bool operator() (PrioritizedTask a, PrioritizedTask b)
    {
        return a.priority > b.priority;
    }
};

class InteractiveClient: public Client
{
public:

	InteractiveClient();
	virtual ~InteractiveClient();

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
	bool executeReaction(const std::string& rawTask, bool async = true);
	bool doQuery(const std::string& queryString);
	void signalDone();
	void cleanupThreadIfDone();
	void cancelCurrentTask();

	//calls to robot
	void stubCb(const ros::TimerEvent& event);
	void gotoDoneCb(const actionlib::SimpleClientGoalState& state,
			const chatbot::NamedMoveBaseResultConstPtr& goal);
	void gotoSend(const std::string& dest, float x, float y);

	void pickDoneCb(const actionlib::SimpleClientGoalState& state,
			const chatbot::NamedMoveBaseResultConstPtr& goal);
	void pickSend(const std::string& object, float x, float y);

	void placeDoneCb(const actionlib::SimpleClientGoalState& state,
			const chatbot::NamedMoveBaseResultConstPtr& goal);
	void placeSend(const std::string& object, float x, float y);
	void respondSend(const std::string & response);
	void respondDoneCb();
	void scheduleRespondDoneCb();

	//helpers and file handling
	std::string getCommaSeparatedString(std::string input) const;
	std::string trimGarbage(std::string  raw) const;
	std::string convertToSentence(const std::vector<std::string>& words) const;
	void toInput(const std::string& input);

	void scheduleProcessQueue();
	void processQueue();

	void handleTask();
	void parseTask(const std::string& taskString);
	std::string getTask();
	void cleanTask();

	void handleOutput(bool async = true);
	void parseOutput(const std::string& output);
	std::vector<std::string> parseArguments(const std::string& arguments) const;
	std::string getOutput();
	void cleanOutput();


	std::string getFile(const std::string& file);
	void cleanFile(const std::string& file);

	ServiceClient serviceClient_;
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
	std::priority_queue<PrioritizedTask, std::vector<PrioritizedTask>, TaskCompare> queuedTasks_;
	PrioritizedTask currentTask_;

	actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction> gotoAc_;
	actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction> pickAc_;
	actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction> placeAc_;
	ros::ServiceClient respondClient_;
	ros::Timer stubTimer_;
	bool actionInProgress_;
	bool taskInProgress_;

};
}
;
}
;

#pragma once
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <chatbot/NamedMoveBaseAction.h>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>

#include <prolog_client/Client.h>
#include <prolog_client/QueryProxy.h>
#include <std_msgs/String.h>

namespace prolog
{
namespace client
{

class InteractiveClient: public Client
{
public:

	enum SolutionMode
	{
		FirstSolution, AllSolutions, IncrementalSolutions
	};

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
	std::string execute(const std::string& input);
	std::string doQuery(const std::string& queryString);
	void signalDone();
	void cleanupThreadIfDone();

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

	//helpers and file handling
	std::string getCommaSeparatedString(std::string input) const;
	std::string trimGarbage(std::string  raw) const;
	std::string convertToSentence(const std::vector<std::string>& words) const;
	void toInput(const std::string& input);
	std::string getOutput();
	void parseOutput(const std::string& output);
	std::vector<std::string> parseArguments(const std::string& arguments) const;
	void cleanOutput();
	void handleOutput();
	bool isResultSuccess(const std::string& res);

	SolutionMode solutionMode_;
	ServiceClient serviceClient_;
	boost::asio::io_service ioService_;
	boost::thread ioThread_;
	boost::asio::posix::stream_descriptor inputStream_;
	boost::asio::streambuf inputBuffer_;
	size_t inputColumn_;
	Query query_;
	QueryProxy queryProxy_;
	QueryProxy::Iterator iterator_;

	ros::NodeHandle nh_;
	ros::Subscriber sub_;

	std::string inputFile_;
	std::string outputFile_;

	actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction> gotoAc_;
	actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction> pickAc_;
	actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction> placeAc_;
	ros::ServiceClient respondSrv_;
	ros::Timer stubTimer_;
	bool actionInProgress_;

};
}
;
}
;

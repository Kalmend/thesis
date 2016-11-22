#include "prolog_client/InteractiveClient.h"
#include <prolog_serialization/PrologSerializer.h>
#include <prolog_client/Query.h>
#include <roscpp_nodewrap/worker/WorkerQueueCallback.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <sstream>

#include <chatbot/Respond.h>


static inline std::string &rtrim(std::string &s)
{
	s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
	return s;
}

static const std::string input_prefix = "raw:";

namespace prolog
{
namespace client
{

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

InteractiveClient::InteractiveClient() :
		inputStream_(ioService_, dup(STDIN_FILENO)),
		inputColumn_(0),
		queryNumber_(0),
		inputFile_(""),
		outputFile_(""),
		gotoAc_("goto", false),
		pickAc_("pick", false),
		placeAc_("place", false),
		respondClient_(nh_.serviceClient<chatbot::Respond>("respond")),
		stubTimer_(nh_.createTimer(ros::Duration(0.1), &InteractiveClient::stubCb, this, true, false)),
		actionInProgress_(false),
		taskInProgress_(false)
{
}

InteractiveClient::~InteractiveClient()
{
	inputStream_.close();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void InteractiveClient::init()
{
	bool connected = false;

	std::cout << "Welcome to the ROS interactive Prolog client.\n";
	std::cout << "\n";

	serviceClient_ = prologServiceClient("prolog", "/prolog_server");
	connected = serviceClient_.exists();

	if (!connected)
	{
		std::cout << "Establishing connection to the Prolog server...\n";
		std::cout << "\n";

		connected = serviceClient_.waitForExistence();

		std::cout << "\n";
	}

	if (connected)
	{

		initChatCore();
		initInputOutput();
		sub_ = nh_.subscribe("recognition/raw_result", 10, &InteractiveClient::onRawSpeech, this);
		std::cout << "Subscribed to /recognition/raw_result for input." << std::endl;
		std::cout << "Type prolog queries here." << std::endl;
		std::cout << "Or use the prefix \"raw:\" to simulate ROS msg." << std::endl;
		std::cout << "Use 'halt.' to exit the client." << std::endl;
		std::cout << "\n";
		startInput();
	}
	else
	{
		ROS_ERROR("PrologRobotInterface::init: Failure to contact the Prolog server.");
		ROS_ERROR("PrologRobotInterface::init: Has the Prolog server been launched?");
		ros::shutdown();
	}
}

void InteractiveClient::initChatCore()
{
	std::string coreFile = getParam(ros::names::append("prolog", "chat_core"), std::string(CHAT_CORE));
	if (!doQuery("ensure_loaded('" + coreFile + "')."))
	{
		ROS_ERROR("PrologRobotInterface::initChatCore: Failure to load chat core.");
		ROS_ERROR("PrologRobotInterface::initChatCore: Is swi-prolog up to date, supports \"->\"?");
		ros::shutdown();
	}

}

void InteractiveClient::initInputOutput()
{
	inputFile_ = getParam(ros::names::append("prolog", "input_file"), std::string(INPUT_FILE));
	outputFile_ = getParam(ros::names::append("prolog", "output_file"), std::string(OUTPUT_FILE));
	taskFile_ = getParam(ros::names::append("prolog", "task_file"), std::string(TASK_FILE));

	doQuery("kill_current_task.");
	if (!doQuery("change_input_filename('" + inputFile_ + "').")
			|| !doQuery("change_output_filename('" + outputFile_ + "').")
			|| !doQuery("change_task_filename('" + taskFile_ + "').")
			)
	{
		ROS_ERROR("PrologRobotInterface::initInputOutput: Could not set prolog files.");
		ros::shutdown();
	}
	cleanOutput();
}

void InteractiveClient::onRawSpeech(const std_msgs::String &msg)
{
	ROS_INFO("PrologRobotInterface::onRawSpeech: %s", msg.data.c_str());
	ROS_INFO("PrologRobotInterface::onRawSpeech: query result: %u", queryReaction(msg.data.c_str()));
}

void InteractiveClient::cleanup()
{
	ioService_.stop();
}

void InteractiveClient::startInput()
{
	ioService_.reset();

	std::cout << "?-" << std::flush;
	boost::asio::async_read_until(inputStream_, inputBuffer_, '\n',
			boost::bind(&InteractiveClient::handleInput, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
	std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
	ioThread_ = boost::thread(boost::bind(run, &ioService_));
}

void InteractiveClient::handleInput(const boost::system::error_code& error, size_t length)
{
	if (!error)
	{
		boost::asio::streambuf::const_buffers_type buffer = inputBuffer_.data();

		std::string queryString;
		queryString += std::string(boost::asio::buffers_begin(buffer), boost::asio::buffers_begin(buffer) + length - 1);

		inputBuffer_.consume(length);

		if (queryString.compare(0, input_prefix.length(), input_prefix) == 0)
		{
			std_msgs::String rosString;
			rosString.data = queryString.substr(input_prefix.length());
			onRawSpeech(rosString);
		}
		else
		{
			std::cout << "prolog:" << (doQuery(queryString) ? "1" : "0") << std::endl;
			std::string fileOutput = getOutput();
			if (fileOutput.length())
			{
				std::cout << "output_file:" << std::endl;
				std::cout << fileOutput << std::endl;
			}
		}
		startInput();
	}
}

bool InteractiveClient::queryReaction(const std::string& rawInput)
{
	toInput(trimGarbage(rawInput));
	bool result = doQuery("t(" + std::to_string(queryNumber_++) + ").");
	handleTask();
	scheduleProcessQueue();
	return result;
}

bool InteractiveClient::executeReaction(const std::string& rawTask, bool async)
{
	std::string prefix = "";
	if(async)
	{
		if (taskInProgress_)
		{
			ROS_ERROR("PrologRobotInterface::executeReaction: reaction task already in progress. aborting!");
			ros::shutdown();
		}
		taskInProgress_ = true;
		prefix = "async_";
	}
	bool result = doQuery(prefix.append(rawTask));
	handleOutput(async);
	return result;
}


bool InteractiveClient::doQuery(const std::string& queryString)
{

	if (!queryString.empty() && (queryString[queryString.length() - 1] == '.'))
	{
		if (queryString == "halt.")
		{
			cleanup();
			ros::shutdown();
			return "";
		}
		else
		{
			Query query(std::string(queryString.begin(), --queryString.end()));
			try
			{
				Solution solution = query.once(serviceClient_);
				return solution.isValid();
			} catch (const ros::Exception& exception)
			{
				ROS_ERROR("PrologRobotInterface::doQuery: exception:%s", exception.what());
				query.close();
				ros::shutdown();
				throw exception;
			}
		}
	}
}


void InteractiveClient::signalDone()
{
	doQuery("signal_done.");
}

void InteractiveClient::cleanupThreadIfDone()
{
	doQuery("cleanup_if_done.");
}

void InteractiveClient::cancelCurrentTask()
{
	//cleanup task prolog part
	while(doQuery("is_task_in_progress."))
		signalDone();
	taskInProgress_ = false;
	cleanOutput();
	cleanupThreadIfDone();

	//cleanup in progress actions goto pick place
	pickAc_.cancelAllGoals();
	placeAc_.cancelAllGoals();
	gotoAc_.cancelAllGoals();
}

//calls to robot
void InteractiveClient::stubCb(const ros::TimerEvent& event)
{
	if (!actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::stubCb: action not in progress. aborting!");
		ros::shutdown();
	}
	actionInProgress_ = false;
	ROS_WARN("PrologRobotInterface::stubCb() fired!");
	signalDone();
	handleOutput(false);
}

void InteractiveClient::gotoDoneCb(const actionlib::SimpleClientGoalState& state, const chatbot::NamedMoveBaseResultConstPtr &result)
{
	if (!actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::gotoDoneCb: action not in progress. aborting!");
		ros::shutdown();
	}
	actionInProgress_ = false;
	ROS_INFO("PrologRobotInterface::gotoDoneCb: %s", state.toString().c_str());
	if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		signalDone();
		handleOutput();
	}
}

void InteractiveClient::gotoSend(const std::string& dest, float x, float y)
{
	if (actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::gotoSend: another action was in progress. aborting !");
		ros::shutdown();
	}
	actionInProgress_ = true;
	if (!gotoAc_.isServerConnected())
	{
		ROS_WARN("PrologRobotInterface::gotoSend: server not connected, scheduling stub.");
		stubTimer_.stop();
		stubTimer_.setPeriod(ros::Duration(0));
		stubTimer_.start();
		return;
	}
	chatbot::NamedMoveBaseGoal goal;
	auto& pos = goal.goal_move_base.target_pose.pose.position;
	pos.x = x;
	pos.y = y;
	goal.goal_move_base.target_pose.pose.orientation.w = 1.0;
	goal.goal_move_base.target_pose.header.frame_id = "map";
	goal.goal_name = dest;
	ROS_INFO("PrologRobotInterface:gotoSend: %s[%.2f,%.2f] sending.", dest.c_str(), pos.x, pos.y);
	gotoAc_.sendGoal(goal, boost::bind(&InteractiveClient::gotoDoneCb, this, _1, _2));
}

void InteractiveClient::pickDoneCb(const actionlib::SimpleClientGoalState& state, const chatbot::NamedMoveBaseResultConstPtr &result)
{
	if (!actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::pickDoneCb: action not in progress. aborting!");
		ros::shutdown();
	}
	actionInProgress_ = false;
	ROS_INFO("PrologRobotInterface::pickDoneCb: %s", state.toString().c_str());
	if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		signalDone();
		handleOutput();
	}
}

void InteractiveClient::pickSend(const std::string& object, float x, float y)
{
	if (actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::pickSend: another action was in progress. aborting !");
		ros::shutdown();
	}
	actionInProgress_ = true;
	if (!pickAc_.isServerConnected())
	{
		ROS_WARN("PrologRobotInterface::pickSend: action server not connected, scheduling stub.");
		stubTimer_.stop();
		stubTimer_.setPeriod(ros::Duration(0));
		stubTimer_.start();
		return;
	}
	chatbot::NamedMoveBaseGoal goal;
	auto& pos = goal.goal_move_base.target_pose.pose.position;
	pos.x = x;
	pos.y = y;
	goal.goal_name = object;
	ROS_INFO("PrologRobotInterface::pickSend: %s[%.2f,%.2f] sending.", object.c_str(), pos.x, pos.y);
	pickAc_.sendGoal(goal, boost::bind(&InteractiveClient::pickDoneCb, this, _1, _2));
}

void InteractiveClient::placeDoneCb(const actionlib::SimpleClientGoalState& state, const chatbot::NamedMoveBaseResultConstPtr &result)
{
	if (!actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::placeDoneCb: action not in progress. aborting!");
		ros::shutdown();
	}
	actionInProgress_ = false;
	ROS_INFO("PrologRobotInterface::placeDoneCb: %s", state.toString().c_str());
	if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		signalDone();
		handleOutput();
	}
}

void InteractiveClient::placeSend(const std::string& object, float x, float y)
{
	if (actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::placeSend: another action was in progress. aborting !");
		ros::shutdown();
	}
	actionInProgress_ = true;
	if (!gotoAc_.isServerConnected())
	{
		ROS_WARN("PrologRobotInterface::placeSend: action server not connected, scheduling stub.");
		stubTimer_.stop();
		stubTimer_.setPeriod(ros::Duration(0));
		stubTimer_.start();
		return;
	}
	chatbot::NamedMoveBaseGoal goal;
	auto& pos = goal.goal_move_base.target_pose.pose.position;
	pos.x = x;
	pos.y = y;
	goal.goal_name = object;
	ROS_INFO("PrologRobotInterface::placeSend: %s[%.2f,%.2f] sending.", object.c_str(), pos.x, pos.y);
	placeAc_.sendGoal(goal, boost::bind(&InteractiveClient::placeDoneCb, this, _1, _2));
}

void InteractiveClient::respondSend(const std::string & response)
{
	if (!respondClient_.exists())
	{
		ROS_WARN("PrologRobotInterface::respondSend: chatbot respond service not found, returning.");
		return;
	}

	chatbot::Respond res;
	res.request.str = response;
	ROS_INFO("PrologRobotInterface::respondSend: %s.", response.c_str());
	respondClient_.call(res);
	scheduleRespondDoneCb();

}

void InteractiveClient::respondDoneCb()
{
	//unused funciton, in case we need to schedule respond
	ROS_INFO("PrologRobotInterface::respondSend: done!");
	handleOutput(false);
}

void InteractiveClient::scheduleRespondDoneCb()
{
	ros::VoidConstPtr trackedObject;
	ros::CallbackInterfacePtr callback(
	new nodewrap::WorkerQueueCallback(boost::bind(&InteractiveClient::respondDoneCb, this), trackedObject,false)
		);

	nh_.getCallbackQueue()->addCallback(callback);
}


std::string InteractiveClient::getCommaSeparatedString(std::string input) const
{
	rtrim(input);
	input = trimGarbage(input);
	std::transform(input.begin(), input.end(), input.begin(), ::tolower);

	for (int i = 0; i < input.length(); i++)
	{
		if (isspace(input[i]))
			input[i] = ',';
	}
	return input;
}

std::string InteractiveClient::trimGarbage(std::string raw) const
{
	rtrim(raw);
	std::string garbage("++garbage++");
	auto gSize = garbage.length();
	auto pos = raw.find(garbage);
	while (pos != std::string::npos)
	{
		//also check if next character is whitespace.
		uint modifier = 0;
		if (pos + gSize < raw.length() && std::isspace(raw[pos + gSize]))
			modifier++;
		raw.erase(pos, gSize + modifier);
		pos = raw.find(garbage);
	}
	return raw;
}

std::string InteractiveClient::convertToSentence(const std::vector<std::string>& words) const
{

	std::ostringstream oss;
	if (!words.empty())
	{
		// Convert all but the last element to avoid a trailing ","
		std::copy(words.begin(), words.end() - 1, std::ostream_iterator<std::string>(oss, " "));

		// Now add the last element with no delimiter
		oss << words.back();
	}
	return oss.str();
}

void InteractiveClient::toInput(const std::string& input)
{
	std::ofstream inputFile;
	inputFile.open(inputFile_.c_str(), std::ios::trunc);
	inputFile << input;
	inputFile.close();
}

std::string InteractiveClient::getTask()
{
	std::string task = getFile(taskFile_);
	task.erase(std::find_if(task.rbegin(), task.rend(),
	            std::not1(std::ptr_fun<int, int>(std::isspace))).base(), task.end());
	return task;
}

void InteractiveClient::handleTask()
{
	std::string task = getTask();
	if (task.length())
	{
		ROS_INFO("PrologRobotInterface::parseTask: task_file:");
		ROS_INFO("\n%s", task.c_str());
		parseTask(task);
	}
	cleanTask();
}

void InteractiveClient::parseTask(const std::string& taskString)
{
	queuedTasks_.emplace(taskString);
}

void InteractiveClient::scheduleProcessQueue()
{
	ros::VoidConstPtr trackedObject;
	ros::CallbackInterfacePtr callback(
	new nodewrap::WorkerQueueCallback(boost::bind(&InteractiveClient::processQueue, this), trackedObject,false)
		);

	nh_.getCallbackQueue()->addCallback(callback);
}

void InteractiveClient::processQueue()
{
	 while( !queuedTasks_.empty() )
	 {

		 if(queuedTasks_.top().synchronous)
		 {
			 executeReaction(queuedTasks_.top().task, false);
			 queuedTasks_.pop();
			 continue;
		 }

		 if(queuedTasks_.top().priority <= currentTask_.priority)
		 {
			 if(taskInProgress_ || actionInProgress_)
			 {
				 cancelCurrentTask();
				 scheduleProcessQueue();
				 return;
			 }
			 currentTask_ = queuedTasks_.top();
			 executeReaction(queuedTasks_.top().task);
			 queuedTasks_.pop();
		 }
		 break;
	 }
}

void InteractiveClient::cleanTask()
{
	cleanFile(taskFile_);
}

void InteractiveClient::handleOutput(bool async)
{
	cleanupThreadIfDone();
	std::string output = getOutput();
	if (output.length())
	{
		ROS_INFO("PrologRobotInterface::handleOutput: output_file:");
		ROS_INFO("\n%s", output.c_str());
		parseOutput(output);
	} else if (async && taskInProgress_)
	{
		taskInProgress_ = false;
		scheduleProcessQueue();
	}
	cleanOutput();
}

void InteractiveClient::parseOutput(const std::string& output)
{
	std::istringstream iss(output);
	std::string line;
	while (std::getline(iss, line))
	{
		std::string command = line.substr(0, line.find_first_of(" \t") - 1); // throw away ':'
		std::string arguments = line.substr(line.find_first_of(" \t") + 1);
		std::vector<std::string> vecArgs = parseArguments(arguments);
		if (command == "GOTO")
		{
			gotoSend(vecArgs[0], std::strtof(vecArgs[3].c_str(), NULL), std::strtof(vecArgs[4].c_str(), NULL));
		}
		else if (command == "PICK")
		{
			pickSend(vecArgs[0], std::strtof(vecArgs[2].c_str(), NULL), std::strtof(vecArgs[3].c_str(), NULL));
		}
		else if (command == "PLACE")
		{
			placeSend(vecArgs[0], std::strtof(vecArgs[2].c_str(), NULL), std::strtof(vecArgs[3].c_str(), NULL));
		}
		else if (command == "RESPOND")
		{
			respondSend(convertToSentence(vecArgs));
		}
		else
		{
			ROS_ERROR("PrologRobotInterface::parseOutput: unknown command from output file: %s", line.c_str());
		}
	}
}


std::vector<std::string> InteractiveClient::parseArguments(const std::string& arguments) const
{
	std::vector<std::string> result;
	std::stringstream ss(arguments);

	while (ss.good())
	{
		std::string word;
		std::getline(ss, word, ',');
		word.erase(std::remove_if(word.begin(), word.end(), [](char c)
		{	return c == '(' || c == ')' || c == ' ' || c == '[' || c == ']';}), word.end());
		result.push_back(word);
	}
	return result;
}

std::string InteractiveClient::getOutput()
{
	return getFile(outputFile_);
}

std::string InteractiveClient::getFile(const std::string& file)
{

	doQuery("are_files_free.");
	std::ifstream f(file);
	std::stringstream ss;
	if (f)
	{
		ss << f.rdbuf();
		f.close();
	}
	return ss.str();
}

void InteractiveClient::cleanOutput()
{
	cleanFile(outputFile_);
}

void InteractiveClient::cleanFile(const std::string& file)
{
	std::ofstream fileStream;
	fileStream.open(file.c_str(), std::ios::trunc);
	fileStream.close();
}

}
}

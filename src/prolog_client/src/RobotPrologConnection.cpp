#include "prolog_client/RobotPrologConnection.h"
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

using namespace prolog::client;

RobotPrologConnection::RobotPrologConnection() :
		inputStream_(ioService_, dup(STDIN_FILENO)),
		inputColumn_(0),
		queryNumber_(0),
		inputFile_(""),
		outputFile_(""),
		currentTask_(nullptr),
		gotoAc_("goto", false),
		pickAc_("pick", false),
		placeAc_("place", false),
		respondClient_(nh_.serviceClient<chatbot::Respond>("respond"))
{
}

RobotPrologConnection::~RobotPrologConnection()
{
	inputStream_.close();
}


//interface methods
std::string RobotPrologConnection::getOutput() { return getFile(outputFile_); }
void RobotPrologConnection::cleanOutput() { cleanFile(outputFile_); }

std::string RobotPrologConnection::getTask()
{
	std::string task = getFile(taskFile_);
	task.erase(std::find_if(task.rbegin(), task.rend(),
	            std::not1(std::ptr_fun<int, int>(std::isspace))).base(), task.end());
	return task;
}

void RobotPrologConnection::cleanTask() { cleanFile(taskFile_); }

bool RobotPrologConnection::doQuery(const std::string& queryString)
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
				prolog::Solution solution = query.once(serviceClient_);
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

void RobotPrologConnection::currentTaskComplete()
{
	ROS_INFO("RobotPrologConnection::currentTaskComplete()");
	if(currentTask_->running)
	{
		ROS_INFO("RobotPrologConnection::currentTaskComplete: invalid state!");
		ros::shutdown();
	}
	currentTask_ = nullptr;
	scheduleProcessAsyncQueue();
}

void RobotPrologConnection::abortAllTasks()
{
	ROS_INFO("RobotPrologConnection::abortAllTasks()");
	if(currentTask_ && currentTask_->running)
	{
		currentTask_->cancel();
	} else
	{
		currentTask_ = nullptr;
	}
	queuedTasks_= std::priority_queue<std::shared_ptr<PrioritizedTask>, std::vector<std::shared_ptr<PrioritizedTask>>, TaskCompare>();


}

ros::ServiceClient& RobotPrologConnection::getRespondClient() { return respondClient_;}
actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& RobotPrologConnection::getGotoActionClient() { return gotoAc_;}
actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& RobotPrologConnection::getPickActionClient() { return pickAc_;}
actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& RobotPrologConnection::getPlaceActionClient() { return placeAc_;}

//other methods
void RobotPrologConnection::init()
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
		sub_ = nh_.subscribe("recognition/raw_result", 10, &RobotPrologConnection::onRawSpeech, this);
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

void RobotPrologConnection::initChatCore()
{
	std::string coreFile = getParam(ros::names::append("prolog", "chat_core"), std::string(CHAT_CORE));
	if (!doQuery("ensure_loaded('" + coreFile + "')."))
	{
		ROS_ERROR("PrologRobotInterface::initChatCore: Failure to load chat core.");
		ROS_ERROR("PrologRobotInterface::initChatCore: Is swi-prolog up to date, supports \"->\"?");
		ros::shutdown();
	}

}

void RobotPrologConnection::initInputOutput()
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

void RobotPrologConnection::onRawSpeech(const std_msgs::String &msg)
{
	ROS_INFO("PrologRobotInterface::onRawSpeech: %s", msg.data.c_str());
	ROS_INFO("PrologRobotInterface::onRawSpeech: query result: %u", queryReaction(msg.data.c_str()));
}

void RobotPrologConnection::cleanup()
{
	ioService_.stop();
}

void RobotPrologConnection::startInput()
{
	ioService_.reset();

	std::cout << "?-" << std::flush;
	boost::asio::async_read_until(inputStream_, inputBuffer_, '\n',
			boost::bind(&RobotPrologConnection::handleInput, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
	std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
	ioThread_ = boost::thread(boost::bind(run, &ioService_));
}

void RobotPrologConnection::handleInput(const boost::system::error_code& error, size_t length)
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

bool RobotPrologConnection::queryReaction(const std::string& rawInput)
{
	toInput(trimGarbage(rawInput));
	bool result = doQuery("t(" + std::to_string(queryNumber_++) + ").");
	handleTask();
	return result;
}

std::string RobotPrologConnection::getCommaSeparatedString(std::string input) const
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

std::string RobotPrologConnection::trimGarbage(std::string raw) const
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


void RobotPrologConnection::toInput(const std::string& input)
{
	std::ofstream inputFile;
	inputFile.open(inputFile_.c_str(), std::ios::trunc);
	inputFile << input;
	inputFile.close();
}

void RobotPrologConnection::handleTask()
{
	std::shared_ptr<PrioritizedTask> task = ReactionFactory::buildTask(this);
	if(task == nullptr)
	{
		ROS_WARN("PrologRobotInterface::handleTask: could not build task");
		return;
	}

	if(task->synchronous)
	{
		task->run();
	}
	else
	{
		queuedTasks_.push(std::move(task));
		scheduleProcessAsyncQueue();
	}

}

void RobotPrologConnection::scheduleProcessAsyncQueue()
{
	ros::VoidConstPtr trackedObject;
	ros::CallbackInterfacePtr callback(
	new nodewrap::WorkerQueueCallback(boost::bind(&RobotPrologConnection::processAsyncQueue, this), trackedObject,false)
		);

	nh_.getCallbackQueue()->addCallback(callback);
}

void RobotPrologConnection::processAsyncQueue()
{
	if(queuedTasks_.empty())
		return;

	if(currentTask_ == nullptr || (queuedTasks_.top()->priority <= currentTask_->priority))
	{
		if(currentTask_ && currentTask_->running)
		{
			currentTask_->cancel();
			return;
		}

		currentTask_ = queuedTasks_.top();
		currentTask_->run();
		queuedTasks_.pop();
	}
}

std::string RobotPrologConnection::getFile(const std::string& file)
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

void RobotPrologConnection::cleanFile(const std::string& file)
{
	std::ofstream fileStream;
	fileStream.open(file.c_str(), std::ios::trunc);
	fileStream.close();
}

#include "prolog_client/InteractiveClient.h"
#include <prolog_serialization/PrologSerializer.h>
#include <prolog_client/Query.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <sstream>

#include <chatbot/Respond.h>

static inline std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}

static const std::string input_prefix = "raw:";

namespace prolog { namespace client {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

InteractiveClient::InteractiveClient() :
  solutionMode_(AllSolutions),
  inputStream_(ioService_, dup(STDIN_FILENO)),
  inputColumn_(0),
  inputFile_(""),
  outputFile_(""),
  gotoAc_("goto", false),
  pickAc_("pick", false),
  placeAc_("place", false),
  respondSrv_(nh_.serviceClient<chatbot::Respond>("respond")),
  stubTimer_(nh_.createTimer(ros::Duration(0.1), &InteractiveClient::stubCb, this, true, false)),
  actionInProgress_(false)
{
}

InteractiveClient::~InteractiveClient() {
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

	solutionMode_ =
			static_cast<SolutionMode>(getParam(
					ros::names::append("prolog", "solution_mode"),
					(int) solutionMode_));

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
		sub_ = getNodeHandle().subscribe("/recognition/raw_result", 10, &InteractiveClient::onRawSpeech, this);
		std::cout << "Subscribed to /recognition/raw_result for input." << std::endl;
		std::cout << "Type prolog queries here." << std::endl;
		std::cout << "Or use the prefix \"raw:\" to simulate ROS msg." << std::endl;;
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
	std::string coreFile = getParam(ros::names::append("prolog", "chat_core"),  std::string(CHAT_CORE));
	std::string result = doQuery("ensure_loaded('" + coreFile + "').");
	if(!isResultSuccess(result))
	{
		ROS_ERROR("PrologRobotInterface::initChatCore: Failure to load chat core.");
		ROS_ERROR("PrologRobotInterface::initChatCore: Is swi-prolog up to date, supports \"->\"?");
		ros::shutdown();
	}

}

void InteractiveClient::initInputOutput()
{
	inputFile_ = getParam(ros::names::append("prolog", "input_file"),  std::string(INPUT_FILE));
	outputFile_ = getParam(ros::names::append("prolog", "output_file"),  std::string(OUTPUT_FILE));
	doQuery("kill_current_task.");
    std::string resInput = doQuery("change_input_filename('" + inputFile_ + "').");
    std::string resOutput = doQuery("change_output_filename('" + outputFile_ + "').");
    if(!isResultSuccess(resInput) || !isResultSuccess(resOutput))
   	{
    	ROS_ERROR("PrologRobotInterface::initInputOutput: Could not set either input or output file location for swi-prolog server.");
   		ros::shutdown();
   	}
    cleanOutput();
}

void InteractiveClient::onRawSpeech(const std_msgs::String &msg)
{
	ROS_INFO("PrologRobotInterface::onRawSpeech: %s", msg.data.c_str());
	std::string unparsed_result = execute(msg.data.c_str());
	ROS_INFO("PrologRobotInterface::onRawSpeech: query result: %s", unparsed_result.c_str());
}

void InteractiveClient::cleanup() {
  ioService_.stop();
  
  query_.close();
}

void InteractiveClient::startInput() {
  ioService_.reset();
  
  std::cout << "?-" << std::flush;
  boost::asio::async_read_until(inputStream_, inputBuffer_, '\n',
	boost::bind(&InteractiveClient::handleInput, this,
	boost::asio::placeholders::error,
	boost::asio::placeholders::bytes_transferred));
  
  ioThread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &ioService_));
}

void InteractiveClient::handleInput(const boost::system::error_code& error,
    size_t length) {
  if (!error) {
    boost::asio::streambuf::const_buffers_type buffer = inputBuffer_.data();
    
    std::string queryString;
    queryString += std::string(boost::asio::buffers_begin(buffer),
      boost::asio::buffers_begin(buffer)+length-1);
    
    inputBuffer_.consume(length);

	if(queryString.compare(0, input_prefix.length(), input_prefix) == 0)
	{
		std_msgs::String rosString;
		rosString.data = queryString.substr(input_prefix.length());
		onRawSpeech(rosString);
	}
	else
	{
		std::cout << "prolog:" << doQuery(queryString) << std::endl;
		std::string fileOutput = getOutput();
		if(fileOutput.length())
		{
			std::cout << "output_file:" << std::endl;
			std::cout << fileOutput << std::endl;
		}
	}
    startInput();
  }
}

std::string InteractiveClient::execute(const std::string& rawInput)
{
	if(isResultSuccess(doQuery("is_task_in_progress.")))
		return "job already in progress, ignoring!"; //we don't want to do anything if task already in progress

	toInput(trimGarbage(rawInput));
	std::string result = doQuery("t.");
	handleOutput();
	return result;
}

std::string InteractiveClient::doQuery(const std::string& queryString)
{
	std::stringstream ss;
	if (query_.isOpen())
	{
		ROS_ERROR("PrologRobotInterface::doQuery: Previous query still open!");
		ros::shutdown();
	}
	else if (!queryString.empty() && (queryString[queryString.length() - 1] == '.'))
	{
		if (queryString == "halt.")
		{
			cleanup();
			ros::shutdown();
			return "";
		}
		else
		{
			query_ = Query(
					std::string(queryString.begin(), --queryString.end()));

			try
			{
				if (solutionMode_ == FirstSolution)
				{
					Solution solution = query_.once(serviceClient_);

					if (!solution.isEmpty())
					{
						serialization::PrologSerializer serializer;
						serializer.serializeBindings(ss,
								solution.getBindings());
					}
					else if (solution.isValid())
						ss << "true";
					else
						ss << "false";
					query_.close();
				}
				else if (solutionMode_ == AllSolutions)
				{
					std::list<Solution> solutions = query_.all(serviceClient_);

					if (!solutions.empty())
					{
						for (std::list<Solution>::const_iterator it =
								solutions.begin(); it != solutions.end(); ++it)
						{
							if (it != solutions.begin())
								ss << ";\n";

							if (!it->isEmpty())
							{
								serialization::PrologSerializer serializer;
								serializer.serializeBindings(ss,
										it->getBindings());
							}
							else if (it->isValid())
								ss << "true";
						}
					}
					else
						ss << "false";
				}
				else
				{
					queryProxy_ = query_.incremental(serviceClient_);

					iterator_ = queryProxy_.begin();

					if (iterator_ != queryProxy_.end())
					{
						if (!iterator_->isEmpty())
						{
							std::ostringstream stream;

							serialization::PrologSerializer serializer;
							serializer.serializeBindings(stream,
									iterator_->getBindings());

							size_t pos = stream.str().rfind('\n');

							if (pos != std::string::npos)
								inputColumn_ = stream.str().length() - pos - 1;
							else
								inputColumn_ = stream.str().length();

							ss << stream.str() << std::flush;
						}
						else
						{
							inputColumn_ = 4;
							ss << "true" << std::flush;
						}

						++iterator_;

						if (iterator_ == queryProxy_.end())
						{
							queryProxy_ = QueryProxy();
							iterator_ = QueryProxy::Iterator();

							ss << ".\n?- " << std::flush;
						}
						else
							ss << " " << std::flush;
					}
					else
					{
						queryProxy_ = QueryProxy();
						iterator_ = QueryProxy::Iterator();

						ss << "false.\n?- " << std::flush;
					}
				}
			} catch (const ros::Exception& exception)
			{
				query_.close();

				queryProxy_ = QueryProxy();
				iterator_ = QueryProxy::Iterator();

				ss << exception.what() << "\n?- " << std::flush;
			}
		}
	}
	else
		ss << "|    " << std::flush;

	return ss.str();
}

void InteractiveClient::cleanupThreadIfDone()
{
	doQuery("cleanup_if_done.");
}
void InteractiveClient::signalDone()
{
	doQuery("signal_done.");
}

//calls to robot
void InteractiveClient::stubCb(const ros::TimerEvent& event)
{
	if(!actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::stubCb: action not in progress. aborting!");
		ros::shutdown();
	}
	actionInProgress_ = false;
	ROS_WARN("PrologRobotInterface::stubCb() fired!");
	signalDone();
	handleOutput();
}

void InteractiveClient::gotoDoneCb(
		const actionlib::SimpleClientGoalState& state,
		const move_base_msgs::MoveBaseResultConstPtr &result)
{
	if(!actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::gotoDoneCb: action not in progress. aborting!");
		ros::shutdown();
	}
	actionInProgress_ = false;
	ROS_INFO("PrologRobotInterface::gotoDoneCb: %s", state.toString().c_str());
	signalDone();
	handleOutput();
}

void InteractiveClient::gotoSend(float x, float y)
{
	if(actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::gotoSend: another action was in progress. aborting !");
		ros::shutdown();
	}
	actionInProgress_ = true;
	if(!gotoAc_.isServerConnected())
	{
		ROS_WARN("PrologRobotInterface::gotoSend: server not connected, scheduling stub.");
		stubTimer_.stop();
		stubTimer_.setPeriod(ros::Duration(10));
		stubTimer_.start();
		return;
	}
	move_base_msgs::MoveBaseGoal goal;
	auto& pos = goal.target_pose.pose.position;
	pos.x = x;
	pos.y = y;
	goal.target_pose.pose.orientation.w = 1.0;
	goal.target_pose.header.frame_id = "map";
	ROS_INFO("PrologRobotInterface:gotoSend: [%.2f,%.2f] sending.", pos.x, pos.y);
	gotoAc_.sendGoal(goal, boost::bind(&InteractiveClient::gotoDoneCb, this, _1, _2));
}

void InteractiveClient::pickDoneCb(
		const actionlib::SimpleClientGoalState& state,
		const move_base_msgs::MoveBaseResultConstPtr &result)
{
	if(!actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::pickDoneCb: action not in progress. aborting!");
		ros::shutdown();
	}
	actionInProgress_ = false;
	ROS_INFO("PrologRobotInterface::pickDoneCb: %s", state.toString().c_str());
	signalDone();
	handleOutput();
}

void InteractiveClient::pickSend(float x, float y)
{
	if(actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::pickSend: another action was in progress. aborting !");
		ros::shutdown();
	}
	actionInProgress_ = true;
	if(!pickAc_.isServerConnected())
	{
		ROS_WARN("PrologRobotInterface::pickSend: action server not connected, scheduling stub.");
		stubTimer_.stop();
		stubTimer_.setPeriod(ros::Duration(5));
		stubTimer_.start();
		return;
	}
	move_base_msgs::MoveBaseGoal goal;
	auto& pos = goal.target_pose.pose.position;
	pos.x = x;
	pos.y = y;
	ROS_INFO("PrologRobotInterface::pickSend: [%.2f,%.2f] sending.", pos.x, pos.y);
	pickAc_.sendGoal(goal, boost::bind(&InteractiveClient::pickDoneCb, this, _1, _2));
}

void InteractiveClient::placeDoneCb(
		const actionlib::SimpleClientGoalState& state,
		const move_base_msgs::MoveBaseResultConstPtr &result)
{
	if(!actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::placeDoneCb: action not in progress. aborting!");
		ros::shutdown();
	}
	actionInProgress_ = false;
	ROS_INFO("PrologRobotInterface::placeDoneCb: %s", state.toString().c_str());
	signalDone();
	handleOutput();
}

void InteractiveClient::placeSend(float x, float y)
{
	if(actionInProgress_)
	{
		ROS_ERROR("PrologRobotInterface::placeSend: another action was in progress. aborting !");
		ros::shutdown();
	}
	actionInProgress_ = true;
	if(!gotoAc_.isServerConnected())
	{
		ROS_WARN("PrologRobotInterface::placeSend: action server not connected, scheduling stub.");
		stubTimer_.stop();
		stubTimer_.setPeriod(ros::Duration(5));
		stubTimer_.start();
		return;
	}
	move_base_msgs::MoveBaseGoal goal;
	auto& pos = goal.target_pose.pose.position;
	pos.x = x;
	pos.y = y;
	ROS_INFO("PrologRobotInterface::placeSend: [%.2f,%.2f] sending.", pos.x, pos.y);
	placeAc_.sendGoal(goal, boost::bind(&InteractiveClient::placeDoneCb, this, _1, _2));
}

void InteractiveClient::respondSend(const std::string & response)
{
	if(!respondSrv_.exists())
	{
		ROS_WARN("PrologRobotInterface::respondSend: chatbot respond service not found, returning.");
		return;
	}

	chatbot::Respond res;
	res.request.str = response;
	ROS_INFO("PrologRobotInterface::respondSend: %s.", response.c_str());
	respondSrv_.call(res);
	ROS_INFO("PrologRobotInterface::respondSend: done!");
}

std::string InteractiveClient::getCommaSeparatedString(std::string input) const
{
	rtrim(input);
	input = trimGarbage(input);
	std::transform(input.begin(), input.end(), input.begin(), ::tolower);

    for(int i = 0; i < input.length(); i++)
    {
           if( isspace(input[i]) )
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
	while(pos != std::string::npos)
	{
		//also check if next character is whitespace.
		uint modifier = 0;
		if(pos + gSize < raw.length() && std::isspace(raw[pos+gSize]))
			modifier++;
		raw.erase(pos,gSize + modifier);
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
		std::copy(words.begin(), words.end()-1, std::ostream_iterator<std::string>(oss, " "));

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

std::string InteractiveClient::getOutput()
{

	doQuery("are_files_free.");
	std::ifstream f(outputFile_);
	std::stringstream ss;
	if (f)
	{
		ss << f.rdbuf();
		f.close();
	}
	return ss.str();
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
			gotoSend(std::strtof(vecArgs[3].c_str(), NULL),
					std::strtof(vecArgs[4].c_str(), NULL));
		}
		else if (command == "PICK")
		{
			pickSend(std::strtof(vecArgs[2].c_str(), NULL),
					std::strtof(vecArgs[3].c_str(), NULL));
		}
		else if (command == "PLACE")
		{
			placeSend(std::strtof(vecArgs[2].c_str(), NULL),
					std::strtof(vecArgs[3].c_str(), NULL));
		}
		else if (command == "RESPOND")
		{
			respondSend(convertToSentence(vecArgs));
		}
		else
		{
			ROS_ERROR(
					"PrologRobotInterface::parseOutput: unknown command from output file: %s",
					line.c_str());
		}
	}
}

std::vector<std::string> InteractiveClient::parseArguments(const std::string& arguments) const
{
	std::vector<std::string> result;
	std::stringstream ss(arguments);

	while( ss.good() )
	{
	    std::string word;
	    std::getline( ss, word, ',' );
	    word.erase(std::remove_if(word.begin(), word.end(), [](char c) {return c == '(' || c == ')' || c == ' ' || c == '[' || c == ']';}), word.end());
		result.push_back(word);
	}
	return result;
}

void InteractiveClient::cleanOutput()
{
	std::ofstream outputFile;
	outputFile.open(outputFile_.c_str(), std::ios::trunc);
	outputFile.close();
}

void InteractiveClient::handleOutput()
{
	cleanupThreadIfDone();
	std::string output = getOutput();
	if (output.length())
	{
		ROS_INFO("PrologRobotInterface::handleOutput: output_file:");
		ROS_INFO("\n%s", output.c_str());
		parseOutput(output);
	}
	cleanOutput();
}

bool InteractiveClient::isResultSuccess(const std::string& res)
{
	if (res.find("true") != std::string::npos)
		return true;
	else
		return false;
}

}}

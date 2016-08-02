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
  respondSrv_(nh_.serviceClient<chatbot::Respond>("respond"))
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
		initRobotCore();
		initInputOutput();
		sub_ = getNodeHandle().subscribe("/recognition/raw_result", 10, &InteractiveClient::onRawSpeech, this);
		std::cout << "Please enter your query after the prompt,\n";
		std::cout << "or type 'halt.' to exit the client.\n";
		std::cout << "\n";
		std::cout << "?- " << std::flush;

		startInput();
	}
	else
	{
		std::cout << "Failure to contact the Prolog server.\n";
		std::cout << "Has the Prolog server been launched?\n";
		ros::shutdown();
	}
}

void InteractiveClient::initChatCore()
{
	std::string coreFile = getParam(ros::names::append("prolog", "chat_core"),  std::string(CHAT_CORE));
	std::string result = doQuery("ensure_loaded('" + coreFile + "').");
	if(!isResultSuccess(result))
	{
		std::cout << "Failure to load chat core.\n";
		std::cout << "Is swi-prolog up to date, supports \"->\"?\n";
		ros::shutdown();
	}

}

void InteractiveClient::initRobotCore()
{

	ROS_INFO("PrologRobotInterface: waiting for chatbot.");
	respondSrv_.waitForExistence();
	ROS_INFO("PrologRobotInterface: done waiting for chatbot.");
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
   		std::cout << "Could not set either input or output file location for swi-prolog server.\n";
   		ros::shutdown();
   	}
    cleanOutput();
}

void InteractiveClient::onRawSpeech(const std_msgs::String &msg)
{
	ROS_INFO("PrologRobotInterface: raw speech: %s", msg.data.c_str());
	std::string unparsed_result = execute(msg.data.c_str());
	ROS_INFO("PrologRobotInterface: query result: %s", unparsed_result.c_str());
}

void InteractiveClient::cleanup() {
  ioService_.stop();
  
  query_.close();
}

void InteractiveClient::startInput() {
  ioService_.reset();
  
  boost::asio::async_read_until(inputStream_, inputBuffer_, '\n',
    boost::bind(&InteractiveClient::handleInput, this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
  
  ioThread_ = boost::thread(boost::bind(&boost::asio::io_service::run,
    &ioService_));
}

void InteractiveClient::handleInput(const boost::system::error_code& error,
    size_t length) {
  if (!error) {
    boost::asio::streambuf::const_buffers_type buffer = inputBuffer_.data();
    
    std::string queryString;
    queryString += std::string(boost::asio::buffers_begin(buffer),
      boost::asio::buffers_begin(buffer)+length-1);
    
    inputBuffer_.consume(length);

    std::cout << doQuery(queryString);
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
		if (queryString == ".")
		{
			queryProxy_ = QueryProxy();
			iterator_ = QueryProxy::Iterator();

			ss << "?- " << std::flush;
		}
		else if (iterator_ != queryProxy_.end())
		{
			ss << "\x1B[A";
			if (inputColumn_)
				ss << "\x1B[" << inputColumn_ << "C";
			ss << ";\x1B[K\n";

			if (!iterator_->isEmpty())
			{
				std::ostringstream stream;

				serialization::PrologSerializer serializer;
				serializer.serializeBindings(stream, iterator_->getBindings());

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

			ss << ".\n?- " << std::flush;
		}
	}
	else if (!queryString.empty()
			&& (queryString[queryString.length() - 1] == '.'))
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

					ss << ".\n?- " << std::flush;
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

					ss << ".\n?- " << std::flush;
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
void InteractiveClient::gotoDoneCb(
		const actionlib::SimpleClientGoalState& state,
		const move_base_msgs::MoveBaseResultConstPtr &result)
{
	 ROS_INFO("PrologRobotInterface: goto finished: %s", state.toString().c_str());
	 signalDone();
	 handleOutput();
}

void InteractiveClient::gotoSend(float x, float y)
{
	  move_base_msgs::MoveBaseGoal goal;
	  auto& pos = goal.target_pose.pose.position;
	  pos.x = x;
	  pos.y = y;
	  ROS_INFO("PrologRobotInterface: goto(%.2f,%.2f) sending.",pos.x, pos.y);
	  gotoAc_.sendGoal(goal, boost::bind(&InteractiveClient::gotoDoneCb, this, _1, _2));
}

void InteractiveClient::pickDoneCb(
		const actionlib::SimpleClientGoalState& state,
		const move_base_msgs::MoveBaseResultConstPtr &result)
{
	 ROS_INFO("PrologRobotInterface: pick finished: %s", state.toString().c_str());
	 signalDone();
	 handleOutput();
}

void InteractiveClient::pickSend(float x, float y)
{
	  move_base_msgs::MoveBaseGoal goal;
	  auto& pos = goal.target_pose.pose.position;
	  pos.x = x;
	  pos.y = y;
	  ROS_INFO("PrologRobotInterface: pick(%.2f,%.2f) sending.",pos.x, pos.y);
	  pickAc_.sendGoal(goal, boost::bind(&InteractiveClient::pickDoneCb, this, _1, _2));
}

void InteractiveClient::placeDoneCb(
		const actionlib::SimpleClientGoalState& state,
		const move_base_msgs::MoveBaseResultConstPtr &result)
{
	 ROS_INFO("PrologRobotInterface: place finished: %s", state.toString().c_str());
	 signalDone();
	 handleOutput();
}

void InteractiveClient::placeSend(float x, float y)
{
	  move_base_msgs::MoveBaseGoal goal;
	  auto& pos = goal.target_pose.pose.position;
	  pos.x = x;
	  pos.y = y;
	  ROS_INFO("PrologRobotInterface: place(%.2f,%.2f) sending.",pos.x, pos.y);
	  placeAc_.sendGoal(goal, boost::bind(&InteractiveClient::placeDoneCb, this, _1, _2));
}

void InteractiveClient::respondSend(const std::string & response)
{
	chatbot::Respond res;
	res.request.str = response;
	ROS_INFO("PrologRobotInterface: respond sending: %s.", response.c_str());
	respondSrv_.call(res);
	ROS_INFO("PrologRobotInterface: respond done.");
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

void InteractiveClient::parseOutput()
{
	  std::ifstream f(outputFile_);
	  std::string line;
	  while (std::getline(f, line))
	  {
		  std::string command = line.substr(0, line.find_first_of(" \t")-1); // throw away ':'
		  std::string arguments = line.substr(line.find_first_of(" \t")+1);
		  std::vector<std::string> vecArgs = parseArguments(arguments);
		  if(command == "GOTO")
		  {
			  gotoSend(std::strtof(vecArgs[3].c_str(),NULL), std::strtof(vecArgs[4].c_str(),NULL));
		  } else if(command == "PICK")
		  {
			  pickSend(std::strtof(vecArgs[2].c_str(),NULL), std::strtof(vecArgs[3].c_str(),NULL));
		  } else if(command == "PLACE")
		  {
			  placeSend(std::strtof(vecArgs[2].c_str(),NULL), std::strtof(vecArgs[3].c_str(),NULL));
		  } else if(command == "RESPOND")
		  {
			  respondSend(convertToSentence(vecArgs));
		  }
		  else
		  {
			  ROS_ERROR("PrologRobotInterface: Unknown command from prolog: %s", line.c_str());
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
	parseOutput();
	cleanOutput();
}

bool InteractiveClient::isResultSuccess(const std::string& res)
{
	if(res.find("true") != std::string::npos)
		return true;
	else
		return false;
}

}}

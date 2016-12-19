#pragma once
#include "RobotPrologConnectionInterface.h"

#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <memory>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <chatbot/NamedMoveBaseAction.h>
#include <functional>


class Command
{
public:
	Command() : synchronous(true), running(false) {}
	Command(bool synchronous, bool running) : synchronous(synchronous), running(running) {}
	virtual ~Command() {}
	virtual void execute() { ROS_INFO("Command::execute: NOOP");}
	virtual void cancel() { ROS_INFO("Command::cancel: NOOP"); }
	virtual std::string name() {return "Command";}

	bool synchronous;
	bool running;
};

class RespondCommand : public Command
{
public:
	RespondCommand(ros::ServiceClient& respondClient, const std::string& sentence);
	virtual ~RespondCommand() {}
	virtual void execute();
	virtual void cancel() { ROS_ERROR("RespondCommand::cancel: NOOP"); }
	virtual std::string name() {return "RespondCommand";}
private:
	ros::ServiceClient& respondClient_;
	std::string sentence_;
};

class StopCommand : public Command
{
public:
	StopCommand(RobotPrologConnectionInterface * rpConnection);
	virtual ~StopCommand() {}
	virtual void execute();
	virtual void cancel() { ROS_ERROR("StopCommand::cancel: NOOP"); }
	virtual std::string name() {return "StopCommand";}
private:
	RobotPrologConnectionInterface * rpConnection_;
};

class GotoCommand : public Command
{
public:
	GotoCommand(actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& gotoAc, std::function<void(bool)> completeCb,  const std::string& place, float x, float y);
	virtual ~GotoCommand() {}
	virtual void execute();
	virtual void cancel();
	virtual std::string name() {return "GotoCommand";}
protected:
	void gotoDoneCb(const actionlib::SimpleClientGoalState& state, const chatbot::NamedMoveBaseResultConstPtr &result);
private:
	actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& gotoAc_;
	std::function<void(bool)> completeCb_;
	std::string place_;
	float x_;
	float y_;
};

class PickCommand : public Command
{
public:
	PickCommand(actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& pickAc, std::function<void(bool)> completeCb,  const std::string& item, float x, float y);
	virtual ~PickCommand() {}
	virtual void execute();
	virtual void cancel();
	virtual std::string name() {return "PickCommand";}
protected:
	void pickDoneCb(const actionlib::SimpleClientGoalState& state, const chatbot::NamedMoveBaseResultConstPtr &result);
private:
	actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& pickAc_;
	std::function<void(bool)> completeCb_;
	std::string item_;
	float x_;
	float y_;
};

class PlaceCommand : public Command
{
public:
	PlaceCommand(actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& placeAc, std::function<void(bool)> completeCb,  const std::string& item, float x, float y);
	virtual ~PlaceCommand() {}
	virtual void execute();
	virtual void cancel();
	virtual std::string name() {return "PlaceCommand";}
protected:
	void placeDoneCb(const actionlib::SimpleClientGoalState& state, const chatbot::NamedMoveBaseResultConstPtr &result);
private:
	actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& placeAc_;
	std::function<void(bool)> completeCb_;
	std::string item_;
	float x_;
	float y_;
};

class PrioritizedTask
{
public:

	PrioritizedTask() : priority(100),synchronous(true), running(false), task("reaction(2,'viga','/home/oliver/thesis/catkin_ws/prolog_files/input.txt',respond).") {}
	PrioritizedTask(int priority, RobotPrologConnectionInterface* rpConnection, const std::string& taskString, bool synchronous) : rpConnection(rpConnection), priority(priority),synchronous(synchronous), running(false), task(taskString) {}
	virtual ~PrioritizedTask() {}
	
	virtual void run() = 0;
	virtual void cancel() = 0;

	RobotPrologConnectionInterface * rpConnection;
	int priority;
	bool synchronous;
	bool running;
	std::string task;
protected:
	void prologCancelReaction();
	bool prologIsReactionInProgress();
	void prologSignalDone();
	void prologCleanupThreadIfDone();

};

class SyncTask : public PrioritizedTask
{
public:
	SyncTask(int priority, RobotPrologConnectionInterface* rpConnection, const std::string& taskString) : PrioritizedTask(priority, rpConnection, taskString, true) {}
	virtual ~SyncTask() {}

	virtual void run();
	virtual void cancel();
};

class AsyncTask : public PrioritizedTask
{
public:
	AsyncTask(int priority, RobotPrologConnectionInterface* rpConnection, const std::string& taskString)
: PrioritizedTask(priority, rpConnection, std::string("async_").append(taskString), false), currentCommand_(nullptr), cancelled_(false)
{}
	virtual ~AsyncTask() {}

	virtual void run();
	virtual void cancel();
protected:
	void commandFinished(bool success);
private:
	void continueAsyncTask();
	void taskFinished();

	std::unique_ptr<Command> currentCommand_;
	bool cancelled_;
};


struct ReactionFactory
{

	static std::vector<std::string> parseArguments(const std::string& arguments)
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

	static std::string convertToSentence(const std::vector<std::string>& words)
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


	static std::shared_ptr<PrioritizedTask> buildTask(RobotPrologConnectionInterface* rpConnection)
	{
		std::string taskString = rpConnection->getTask();
		rpConnection->cleanTask();
		if(taskString.empty())
			return nullptr;


		std::stringstream ss(taskString);

		std::vector<std::string> result;

		std::size_t found = taskString.find_last_of(",");
		std::string commandType = taskString.substr(found + 1);
		commandType.erase(
				std::remove_if(commandType.begin(), commandType.end(),
						[](char c)
						{	return c == '(' || c == ')' || c == ' ' || c == '[' || c == ']' || c == '\n' || c == '.';}),
				commandType.end());

		if (commandType == "stop")
		{
			return std::make_shared<SyncTask>(1, rpConnection, taskString);
		}
		else if (commandType == "respond")
		{
			return std::make_shared<SyncTask>(2, rpConnection, taskString);
		}
		else if (commandType == "liigu_mine")
		{
			return std::make_shared<AsyncTask>(3, rpConnection, taskString);
		}
		else
		{
			return std::make_shared<AsyncTask>(4, rpConnection, taskString);
		}

	}

	static std::vector<std::unique_ptr<Command>> buildCommands(RobotPrologConnectionInterface* rpConnection, std::function<void(bool)> completeCb)
	{
		std::string output = rpConnection->getOutput();
		rpConnection->cleanOutput();
		std::vector<std::unique_ptr<Command>> resultVector;

		if(output.empty())
			return resultVector;

		std::istringstream iss(output);
		std::string line;
		while (std::getline(iss, line))
		{
			std::string command = line.substr(0, line.find_first_of(" \t") - 1); // throw away ':'
			std::string arguments = line.substr(line.find_first_of(" \t") + 1);
			std::vector<std::string> vecArgs = parseArguments(arguments);

			if (command == "GOTO")
			{
				resultVector.push_back(std::move(std::unique_ptr<GotoCommand>(new GotoCommand(rpConnection->getGotoActionClient(), completeCb, vecArgs[0], std::strtof(vecArgs[3].c_str(), NULL), std::strtof(vecArgs[4].c_str(), NULL)))));
			}
			else if (command == "PICK")
			{
				resultVector.push_back(std::move(std::unique_ptr<PickCommand>(new PickCommand(rpConnection->getPickActionClient(), completeCb, vecArgs[0], std::strtof(vecArgs[3].c_str(), NULL), std::strtof(vecArgs[4].c_str(), NULL)))));
			}
			else if (command == "PLACE")
			{
				resultVector.push_back(std::move(std::unique_ptr<PlaceCommand>(new PlaceCommand(rpConnection->getPlaceActionClient(), completeCb, vecArgs[0], std::strtof(vecArgs[3].c_str(), NULL), std::strtof(vecArgs[4].c_str(), NULL)))));
			}
			else if (command == "RESPOND")
			{
				resultVector.push_back(std::move(std::unique_ptr<RespondCommand>(new RespondCommand(rpConnection->getRespondClient(), convertToSentence(vecArgs)))));
			}
			else if (command == "STOP")
			{
				resultVector.push_back(std::move(std::unique_ptr<StopCommand>(new StopCommand(rpConnection))));
			}
			else
			{
				ROS_ERROR("ReactionFactory::buildCommand: unknown command from output file: %s", line.c_str());
				resultVector.push_back(std::move(std::unique_ptr<RespondCommand>(new RespondCommand(rpConnection->getRespondClient(), "esines viga"))));
			}
		}
		return resultVector;

	}
};


#include "prolog_client/PrioritizedTask.h"
#include <chatbot/Respond.h>

//commands

//STOP:
StopCommand::StopCommand(RobotPrologConnectionInterface * rpConnection) : Command(true, false), rpConnection_(rpConnection)
{
}

void StopCommand::execute()
{
	running = true; //for show, doesnt matter since its synchronous
	rpConnection_->abortAllTasks();
	running = false;
}

//RESPOND:
RespondCommand::RespondCommand(ros::ServiceClient& respondClient, const std::string& sentence) : Command(true, false), respondClient_(respondClient), sentence_(sentence)
{
}

void RespondCommand::execute()
{
	running = true; //for show, doesnt matter since its synchronous
	if (!respondClient_.exists())
	{
		ROS_WARN("RespondCommand::execute: chatbot respond service not found, returning.");
		return;
	}

	chatbot::Respond res;
	res.request.str = sentence_;
	ROS_INFO("RespondCommand::execute: %s.", sentence_.c_str());
	respondClient_.call(res);
	running = false;
}

//GOTO:
GotoCommand::GotoCommand(actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& gotoAc, std::function<void(bool)> completeCb, const std::string& place, float x, float y)
: Command(false,false), gotoAc_(gotoAc), completeCb_(completeCb), place_(place), x_(x), y_(y)
{

}

void GotoCommand::execute()
{
	if (running)
	{
		ROS_ERROR("GotoCommand::execute: command double started. aborting !");
		ros::shutdown();
	}
	running = true;
	if (!gotoAc_.isServerConnected())
	{
		ROS_WARN("GotoCommand::execute: server not connected, queueing stub completion");
		ros::shutdown();
		/*
		stubTimer_.stop();
		stubTimer_.setPeriod(ros::Duration(0));
		stubTimer_.start();
		return;*/
	}
	chatbot::NamedMoveBaseGoal goal;
	auto& pos = goal.goal_move_base.target_pose.pose.position;
	pos.x = x_;
	pos.y = y_;
	goal.goal_move_base.target_pose.pose.orientation.w = 1.0;
	goal.goal_move_base.target_pose.header.frame_id = "map";
	goal.goal_name = place_;
	ROS_INFO("GotoCommand::execute: %s[%.2f,%.2f] sending.", place_.c_str(), pos.x, pos.y);
	gotoAc_.sendGoal(goal, boost::bind(&GotoCommand::gotoDoneCb, this, _1, _2));
}

void GotoCommand::cancel()
{
	ROS_INFO("GotoCommand::cancel()");
	gotoAc_.cancelAllGoals();
}

void GotoCommand::gotoDoneCb(const actionlib::SimpleClientGoalState& state, const chatbot::NamedMoveBaseResultConstPtr &result)
{
	if (!running)
	{
		ROS_ERROR("GotoCommand::gotoDoneCb: action not in progress. aborting!");
		ros::shutdown();
	}
	running = false;
	ROS_INFO("GotoCommand::gotoDoneCb: %s", state.toString().c_str());
	if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		completeCb_(true);
	else
		completeCb_(false);
}

//PICK:
PickCommand::PickCommand(actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& pickAc, std::function<void(bool)> completeCb, const std::string& item, float x, float y)
: Command(false,false), pickAc_(pickAc), completeCb_(completeCb), item_(item), x_(x), y_(y)
{

}

void PickCommand::execute()
{
	if (running)
	{
		ROS_ERROR("PickCommand::execute: command double started. aborting !");
		ros::shutdown();
	}
	running = true;
	if (!pickAc_.isServerConnected())
	{
		ROS_WARN("PickCommand::execute: server not connected, queueing stub completion");
		ros::shutdown();
		/*
		stubTimer_.stop();
		stubTimer_.setPeriod(ros::Duration(0));
		stubTimer_.start();
		return;*/
	}
	chatbot::NamedMoveBaseGoal goal;
	auto& pos = goal.goal_move_base.target_pose.pose.position;
	pos.x = x_;
	pos.y = y_;
	goal.goal_move_base.target_pose.pose.orientation.w = 1.0;
	goal.goal_move_base.target_pose.header.frame_id = "map";
	goal.goal_name = item_;
	ROS_INFO("PickCommand::execute: %s[%.2f,%.2f] sending.", item_.c_str(), pos.x, pos.y);
	pickAc_.sendGoal(goal, boost::bind(&PickCommand::pickDoneCb, this, _1, _2));
}

void PickCommand::cancel()
{
	ROS_INFO("PickCommand::cancel()");
	pickAc_.cancelAllGoals();
}

void PickCommand::pickDoneCb(const actionlib::SimpleClientGoalState& state, const chatbot::NamedMoveBaseResultConstPtr &result)
{
	if (!running)
	{
		ROS_ERROR("PickCommand::pickDoneCb: action not in progress. aborting!");
		ros::shutdown();
	}
	running = false;
	ROS_INFO("PickCommand::pickDoneCb: %s", state.toString().c_str());
	if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		completeCb_(true);
	else
		completeCb_(false);
}

//PLACE:
PlaceCommand::PlaceCommand(actionlib::SimpleActionClient<chatbot::NamedMoveBaseAction>& placeAc, std::function<void(bool)> completeCb, const std::string& item, float x, float y)
: Command(false,false), placeAc_(placeAc), completeCb_(completeCb), item_(item), x_(x), y_(y)
{

}

void PlaceCommand::execute()
{
	if (running)
	{
		ROS_ERROR("PlaceCommand::execute: command double started. aborting !");
		ros::shutdown();
	}
	running = true;
	if (!placeAc_.isServerConnected())
	{
		ROS_WARN("PlaceCommand::execute: server not connected, queueing stub completion");
		ros::shutdown();
		/*
		stubTimer_.stop();
		stubTimer_.setPeriod(ros::Duration(0));
		stubTimer_.start();
		return;*/
	}
	chatbot::NamedMoveBaseGoal goal;
	auto& pos = goal.goal_move_base.target_pose.pose.position;
	pos.x = x_;
	pos.y = y_;
	goal.goal_move_base.target_pose.pose.orientation.w = 1.0;
	goal.goal_move_base.target_pose.header.frame_id = "map";
	goal.goal_name = item_;
	ROS_INFO("PlaceCommand::execute: %s[%.2f,%.2f] sending.", item_.c_str(), pos.x, pos.y);
	placeAc_.sendGoal(goal, boost::bind(&PlaceCommand::placeDoneCb, this, _1, _2));
}

void PlaceCommand::cancel()
{
	ROS_INFO("PlaceCommand::cancel()");
	placeAc_.cancelAllGoals();
}

void PlaceCommand::placeDoneCb(const actionlib::SimpleClientGoalState& state, const chatbot::NamedMoveBaseResultConstPtr &result)
{
	if (!running)
	{
		ROS_ERROR("PlaceCommand::placeDoneCb: action not in progress. aborting!");
		ros::shutdown();
	}
	running = false;
	ROS_INFO("PlaceCommand::placeDoneCb: %s", state.toString().c_str());
	if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		completeCb_(true);
	else
		completeCb_(false);
}

//base task implementation
void PrioritizedTask::prologCancelReaction()
{
	while(prologIsReactionInProgress())
		prologSignalDone();
	prologCleanupThreadIfDone();
	rpConnection->cleanOutput();
}

bool PrioritizedTask::prologIsReactionInProgress()
{
	return rpConnection->doQuery("is_task_in_progress.");
}

void PrioritizedTask::prologSignalDone()
{
	rpConnection->doQuery("signal_done.");
}

void PrioritizedTask::prologCleanupThreadIfDone()
{
	rpConnection->doQuery("cleanup_if_done.");
}
//sync implementation
void SyncTask::run()
{
	ROS_INFO("SyncTask::run: %s", task.c_str());
	bool result = rpConnection->doQuery(task);
	if(!result)
	{
		ROS_ERROR("SyncTask::run: query failed. Aborting!");
		ros::shutdown();
	}

	auto cmds = ReactionFactory::buildCommands(rpConnection, nullptr);

	for(auto const& cmd: cmds)
	{
		if(!cmd->synchronous)
		{
			ROS_ERROR("SyncTask::run: async command in synchronous task. Aborting!");
			ros::shutdown();
		}

		if(cmd)
		{
			cmd->execute();
		}
	}

}

void SyncTask::cancel()
{
	ROS_INFO("SyncTask::cancel: called on sync task. Aborting!");
	ros::shutdown();
}

//async implementation
void AsyncTask::run()
{
	ROS_INFO("AsyncTask::run: %s", task.c_str());
	bool result = rpConnection->doQuery(task);
	if (!result || running)
	{
		ROS_ERROR("AsyncTask::run: invalid state, aborting!");
		ros::shutdown();
		return;
	}
	running = true;
	auto cmds = ReactionFactory::buildCommands(rpConnection, std::bind(&AsyncTask::commandFinished, this, std::placeholders::_1));


	if (cmds.empty())
	{
		ROS_ERROR("AsyncTask::run: no commands in reaction, NOOP.");
		running = false;
		return;
	}
	bool gotAsyncCommand = false;
	for(auto& cmd: cmds)
	{
		ROS_INFO("AsyncTask::running: %s", cmd->name().c_str());
		currentCommand_ = std::move(cmd);
		currentCommand_->execute();
		if(!currentCommand_->synchronous)
		{
			gotAsyncCommand = true;
			break;
		}
	}

	if(!gotAsyncCommand)
	{
		ROS_ERROR("AsyncTask::run: no async command in async task, ending task.");
		taskFinished();
	}

}

void AsyncTask::cancel()
{
	if(cancelled_ || !running)
	{
		ROS_WARN("AsyncTask::cancel: task was already cancelled");
		return;
	}
	ROS_INFO("AsyncTask::cancel: %s", task.c_str());


	cancelled_ = true;
	//cleanup prolog side
	prologCancelReaction();

	//cleanup action side
	if(currentCommand_->running)
	{
		currentCommand_->cancel();
	} else
	{
		taskFinished();
	}
}

void AsyncTask::commandFinished(bool success)
{
	if(cancelled_ && !success)
	{
		taskFinished();
	}
	else if (!cancelled_ && !success)
	{
		auto respond = std::move(std::unique_ptr<RespondCommand>(new RespondCommand(rpConnection->getRespondClient(), "käsu täitmine ebaõnnestus")));
		respond->execute();
		prologCancelReaction();
		taskFinished();
	}
	else
	{
		continueAsyncTask();
	}
}

void AsyncTask::continueAsyncTask()
{
	ROS_INFO("AsyncTask::continueAsyncTask()");
	prologSignalDone();
	if (!running)
	{
		ROS_ERROR("AsyncTask::continueAsyncTask: invalid state, aborting!");
		ros::shutdown();
		return;
	}
	auto cmds = ReactionFactory::buildCommands(rpConnection, std::bind(&AsyncTask::commandFinished, this, std::placeholders::_1));
	if (cmds.empty())
	{
		ROS_INFO("AsyncTask::taskComplete: %s", task.c_str());
		prologCleanupThreadIfDone();
		taskFinished();
		return;
	}
	bool startedNewAsyncTask = false;
	for(auto& cmd: cmds)
	{
		ROS_INFO("AsyncTask::running: %s", cmd->name().c_str());
		currentCommand_ = std::move(cmd);
		currentCommand_->execute();
		if(!currentCommand_->synchronous)
		{
			startedNewAsyncTask = true;
			break;
		}
	}
	if(!startedNewAsyncTask)
	{
		ROS_INFO("AsyncTask::taskComplete: %s", task.c_str());
		prologCleanupThreadIfDone();
		taskFinished();
	}

}

void AsyncTask::taskFinished()
{
	running = false;
	rpConnection->currentTaskComplete();
}




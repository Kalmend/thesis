/******************************************************************************
 * Copyright (C) 2016 by Ralf Kaestner                                        *
 * ralf.kaestner@gmail.com                                                    *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file InteractiveClient.h
 * \brief Header file providing the InteractiveClient class interface
 */

#ifndef ROS_PROLOG_CLIENT_INTERACTIVE_CLIENT_H
#define ROS_PROLOG_CLIENT_INTERACTIVE_CLIENT_H
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>

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

	//calls to robot
	void gotoDoneCb(const actionlib::SimpleClientGoalState& state,
			const move_base_msgs::MoveBaseResultConstPtr &goal);
	void gotoSend();

	//helpers
	std::string getCommaSeparatedString(std::string input) const;
	std::string trimGarbage(std::string raw) const;
	void toInput(const std::string& input);
	void processOutput();
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

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

};
}
;
}
;

#endif

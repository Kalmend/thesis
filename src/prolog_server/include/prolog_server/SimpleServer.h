#ifndef ROS_PROLOG_SERVER_SIMPLE_SERVER_H
#define ROS_PROLOG_SERVER_SIMPLE_SERVER_H

#include <roscpp_nodewrap/worker/Worker.h>
#include <prolog_swi/Engine.h>
#include <prolog_server/Server.h>
#include <prolog_server/ThreadedQuery.h>

namespace prolog
{
namespace server
{

class SimpleServer: public Server
{
public:

	SimpleServer();
	virtual ~SimpleServer();

protected:
	void init();
	void cleanup();
	virtual bool openQueryCallback(prolog_msgs::OpenQuery::Request& request,
			prolog_msgs::OpenQuery::Response& response);
	virtual bool hasSolutionCallback(prolog_msgs::HasSolution::Request& request,
			prolog_msgs::HasSolution::Response& response);
	virtual bool getAllSolutionsCallback(
			prolog_msgs::GetAllSolutions::Request& request,
			prolog_msgs::GetAllSolutions::Response& response);
	virtual bool getNextSolutionCallback(
			prolog_msgs::GetNextSolution::Request& request,
			prolog_msgs::GetNextSolution::Response& response);
	virtual bool closeQueryCallback(prolog_msgs::CloseQuery::Request& request,
			prolog_msgs::CloseQuery::Response& response);
private:
	ServiceServer serviceServer_;
	swi::Engine engine_;
	ThreadedQuery query_;
	nodewrap::Worker worker_;
};

}
}

#endif

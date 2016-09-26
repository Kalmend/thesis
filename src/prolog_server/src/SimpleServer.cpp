#include "prolog_server/SimpleServer.h"
#include <list>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <prolog_common/Bindings.h>
#include <prolog_serialization/JSONDeserializer.h>
#include <prolog_serialization/JSONSerializer.h>
#include <roscpp_nodewrap/Nodelet.h>

NODEWRAP_EXPORT_CLASS(prolog_server, prolog::server::SimpleServer)

namespace prolog
{
namespace server
{

SimpleServer::SimpleServer() {}
SimpleServer::~SimpleServer() {}

void SimpleServer::init()
{
	Server::init();
	if (isPrologInitialized())
	{
		serviceServer_ = advertisePrologService("prolog");
		engine_ = createPrologEngine("synchronous_prolog_engine");
	}
}

void SimpleServer::cleanup()
{
	serviceServer_.shutdown();
	Server::cleanup();
}

bool SimpleServer::openQueryCallback(prolog_msgs::OpenQuery::Request& request,
		prolog_msgs::OpenQuery::Response& response)
{
	if (request.query.empty())
	{
		response.ok = false;
		response.error = "Query is empty.";
		NODEWRAP_ERROR_STREAM(response.error);
		return true;
	}
	std::string queryIdentifier = prologQueryIdentifier();
	try
	{
		query_.impl_.reset(new ThreadedQuery::Impl(request.query, engine_, ThreadedQuery::IncrementalMode));
	}
	catch (const ros::Exception& exception)
	{
		response.ok = false;
		response.error = std::string("Failure to create query: ") + exception.what();
		NODEWRAP_ERROR_STREAM(response.error);
		return true;
	}

	nodewrap::WorkerOptions workerOptions;
	workerOptions.frequency = 0.0;
	workerOptions.callback = boost::bind(&ThreadedQuery::Impl::execute, query_.impl_, _1);
	workerOptions.autostart = true;
	workerOptions.synchronous = true;
	workerOptions.privateCallbackQueue = true;
	try
	{
		worker_ = addWorker("query_" + queryIdentifier, workerOptions);
		worker_.wake();
	}
	catch (const ros::Exception& exception)
	{
		response.ok = false;
		response.error = std::string("Failure to create worker: ") + exception.what();
		return true;
	}
	NODEWRAP_INFO_STREAM("Prolog query [" << queryIdentifier << "] has been opened.");
	response.ok = true;
	response.id = queryIdentifier;

	return true;
}

bool SimpleServer::hasSolutionCallback(
		prolog_msgs::HasSolution::Request& request,
		prolog_msgs::HasSolution::Response& response)
{
	std::string error;
	response.result = query_.hasSolution(error, true);
	if (!response.result && !error.empty())
	{
		response.status = prolog_msgs::GetNextSolution::Response::STATUS_QUERY_FAILED;
		response.error = error;
		return true;
	}
	response.status = prolog_msgs::GetNextSolution::Response::STATUS_OK;
	return true;
}

bool SimpleServer::getAllSolutionsCallback(
		prolog_msgs::GetAllSolutions::Request& request,
		prolog_msgs::GetAllSolutions::Response& response)
{

	NODEWRAP_INFO_STREAM("Prolog query [" << request.id << "] SimpleServer, treating as single solution query.");
	prolog_msgs::GetNextSolution::Request singleRequest;
	prolog_msgs::GetNextSolution::Response singleResponse;

	singleRequest.close = true;
	singleRequest.id = request.id;
	getNextSolutionCallback(singleRequest, singleResponse);
    response.status = singleResponse.status;
    response.error = singleResponse.error;
    response.solutions.push_back(singleResponse.solution);
}

bool SimpleServer::getNextSolutionCallback(
		prolog_msgs::GetNextSolution::Request& request,
		prolog_msgs::GetNextSolution::Response& response)
{
	Bindings bindings;
	std::string error;
	if (!query_.getNextSolution(bindings, error, true))
	{
		if (!error.empty())
		{
			response.status = prolog_msgs::GetNextSolution::Response::STATUS_QUERY_FAILED;
			response.error = error;
		}
		else
		{
			response.status = prolog_msgs::GetNextSolution::Response::STATUS_NO_SOLUTIONS;
		}
		return true;
	}

	if (request.close)
	{
		worker_.cancel(true);
		NODEWRAP_INFO_STREAM("Prolog query [" << request.id << "] has been closed.");
	}
	query_.impl_.reset();
	std::ostringstream stream;
	serialization::JSONSerializer serializer;
	serializer.serializeBindings(stream, bindings);
	response.solution = stream.str();
	response.status = prolog_msgs::GetNextSolution::Response::STATUS_OK;

	return true;
}

bool SimpleServer::closeQueryCallback(prolog_msgs::CloseQuery::Request& request,
		prolog_msgs::CloseQuery::Response& response)
{
	worker_.cancel(true);
	query_.impl_.reset();
	NODEWRAP_INFO_STREAM("Prolog query [" << request.id << "] has been closed.");
	response.status = prolog_msgs::CloseQuery::Response::STATUS_OK;
	return true;
}

}
}

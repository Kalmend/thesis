#pragma once
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

class PrioritizedTask
{
public:

	PrioritizedTask() : priority(100),synchronous(true), task("reaction(2,'viga','/home/oliver/thesis/catkin_ws/prolog_files/input.txt',respond).")
	{
	}
	
	PrioritizedTask(const std::string& taskString) : task(taskString)
	{
		std::stringstream ss(taskString);
		std::vector<std::string> result;

		std::size_t found = taskString.find_last_of(",");
		std::string commandType = taskString.substr(found+1);
		commandType.erase(std::remove_if(
				commandType.begin(), commandType.end(), [](char c)
		{	return c == '(' || c == ')' || c == ' ' || c == '[' || c == ']' || c == '\n' || c == '.';}), commandType.end());

	    
	    if(commandType == "stop")
	    {
	    	priority = 1;
	    	synchronous = true;
	    }    	
	    else if (commandType == "respond")
	    {
	    	priority = 2;
	    	synchronous = true;
	    }
	    else 
	    {
	    	priority = 3;
	    	synchronous = false;
	    }
	}
	
	~PrioritizedTask() {}
	
	
	int priority;
	bool synchronous;
	std::string task;


};

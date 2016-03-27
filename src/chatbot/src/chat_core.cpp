#include "chat_core.hpp"
#include <SWI-Prolog.h>
#include <ros/ros.h>
#include <vector>
#include <string>
using namespace std;

ChatCore::ChatCore() :
		argv_(NULL)
{
	initializeProlog();
}

void ChatCore::initializeProlog()
{
	vector<string> args;
	args.push_back(SWIPL_EXECUTABLE);

	argv_ = new char*[args.size()];

	for (size_t index = 0; index < args.size(); ++index)
		argv_[index] = const_cast<char*>(args[index].c_str());

	if (!PL_initialise(args.size(), argv_))
		PL_halt(1);

}


ChatCore::~ChatCore()
{
	delete argv_;
	PL_halt(0);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "chat_core");
	ChatCore server;
	ros::spin();
	return 0;
}


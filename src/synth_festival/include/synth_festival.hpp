#include <ros/ros.h>
#include <festival.h>
#include <std_msgs/String.h>

class SynthesizerFestival
{
public:
	SynthesizerFestival();
	~SynthesizerFestival();

private:
	void onSpeech(const std_msgs::String &msg);

	ros::NodeHandle nh_;
	ros::Subscriber respondSub_;
	ros::ServiceClient pauseClient_;
	std::string voice_;
};

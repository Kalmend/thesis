#include "synth_festival.hpp"
#include <boost/locale.hpp>
#include <std_srvs/SetBool.h>

using namespace boost::locale::conv;
using namespace std;

SynthesizerFestival::SynthesizerFestival():
		respondSub_(nh_.subscribe("synth_festival/text_to_speak", 10, &SynthesizerFestival::onSpeech, this)),
		pauseClient_(nh_.serviceClient<std_srvs::SetBool>("recognition/set_pause"))
{
	ros::param::param<std::string>("~voice", voice_, "voice_eki_et_riina_clunits");
	int heap_size = 2500000;
	int load_init_files = 1; // we want the festival init files loaded
	festival_initialize(load_init_files,heap_size);
	festival_eval_command("(Parameter.set 'Audio_Method 'Audio_Command)");
	festival_eval_command("(Parameter.set 'Audio_Command \"aplay -Dplug:default -f S16_LE -r $SR $FILE\")");
	std::string voice_command = "(";
	voice_command.append(voice_);
	voice_command.append(")");
	festival_eval_command(voice_command.c_str());
}

SynthesizerFestival::~SynthesizerFestival()
{
}

void SynthesizerFestival::onSpeech(const std_msgs::String &msg)
{
	if (!pauseClient_.exists())
		ROS_WARN("SynthesizerFestival::onSpeech: cant pause recognition, pause client missing.");

	std_srvs::SetBool req;
	req.request.data = true;
	if (pauseClient_.exists())
		pauseClient_.call(req);

	ROS_INFO("SynthesizerFestival:respond: %s", msg.data.c_str());
	festival_say_text(from_utf(msg.data.c_str(),"Latin9").c_str());

	req.request.data = false;
	if (pauseClient_.exists())
		pauseClient_.call(req);

	return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "synth_festival", ros::init_options::NoSigintHandler);
	SynthesizerFestival synth;
	ros::spin();
	return 0;





}


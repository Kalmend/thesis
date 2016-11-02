#include "synth_festival.hpp"
#include <boost/locale.hpp>
using namespace boost::locale::conv;
using namespace std;

SynthesizerFestival::SynthesizerFestival()
{

}

SynthesizerFestival::~SynthesizerFestival()
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "synthesizer", ros::init_options::NoSigintHandler);

	SynthesizerFestival client;
    	int heap_size = 2100000;
    	int load_init_files = 1; // we want the festival init files loaded

    	festival_initialize(load_init_files,heap_size);
    	festival_eval_command("(voice_eki_et_riina_clunits)");
    	festival_say_text(from_utf(u8"jäärapäine ööbik õun","Latin9").c_str());

	ros::spin();
	return 0;
}


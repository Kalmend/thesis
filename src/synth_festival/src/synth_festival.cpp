#include "synth_festival.hpp"

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
    	// Say some text;

	FILE *f = fopen("/tmp/test.txt", "rb");
	fseek(f, 0, SEEK_END);
	long fsize = ftell(f);
	fseek(f, 0, SEEK_SET);
	std::string arp = "jäärapäine ööbik õun";
	const char* uarp = "jäärapäine ööbik õun";
	
	char *string = (char *)malloc(fsize + 1);
	fread(string, fsize, 1, f);
	fclose(f);

	string[fsize] = 0;
	std::cout << std::to_string((uint)string[0]) << "|" << std::to_string((uint)string[1]) << "|" << std::to_string((uint)string[2]) << string[3] << " x " << std::to_string((uint)uarp[0]) << "|" << std::to_string((uint)uarp[1]) << "|" << std::to_string((uint)uarp[2]) << "|" << std::to_string((uint)uarp[3]) << "|" << std::to_string((uint)uarp[4])  << " x "  << std::endl;
    	festival_say_text(string);
	//festival_say_text(arp.c_str());
	festival_say_text(uarp);
	ros::spin();
	return 0;
}


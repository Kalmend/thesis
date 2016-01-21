#include <gst/gst.h>
#include <ros/ros.h>
#include <signal.h>
#include <boost/thread.hpp>

using namespace std;

class SpeechRecognitionSpeex
{
public:
	SpeechRecognitionSpeex();
	~SpeechRecognitionSpeex();


private:

	static gboolean onBusMessage(GstBus* bus, GstMessage* msg, gpointer userData);
	void exitOnMainThread(const std::string & message, int code);
	void parseArguments();
	void setupPipeline();
	GstElement* getSpeechRecognition();

	//arguments
	struct SRArguments
	{
		bool threaded_decoder;
		bool do_endpointing;
		std::string model_dir;
		std::string model;
		std::string fst;
		std::string word_syms;
		std::string feature_type;
		std::string mfcc_config;
		std::string ivector_extraction_config;
		std::string ep_silence_phones;
		int max_active;
		float beam;
		float lattice_beam;
		float chunk_length_s;
	} sr_arguments_;
	//ROS stuff
	ros::ServiceClient serviceClient_;

	//pipeline elements
	GstElement *pipeline_, *source_, *sink_, *resample_, *convert_, *recog_;

	boost::thread gst_thread_;
	GMainLoop *loop_;
};

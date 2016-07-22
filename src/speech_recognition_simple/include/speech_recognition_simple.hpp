#include <gst/gst.h>
#include <ros/ros.h>
#include <signal.h>
#include <boost/thread.hpp>
#include <gst/app/gstappsink.h>
#include <std_msgs/String.h>
using namespace std;


class SpeechRecognitionSimple
{
public:
	SpeechRecognitionSimple();
	~SpeechRecognitionSimple();


private:

	static gboolean onBusMessage(GstBus* bus, GstMessage* msg, gpointer userData);
	static GstFlowReturn onNewRecognition (GstAppSink *appsink, gpointer userData);

	void publish( const std_msgs::String &msg );

	void exitOnMainThread(const std::string & message, int code);
	void parseArguments();
	void setupPipeline();
	GstElement* getSpeechRecognition();

	std::string destination_type_;
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
		float acoustic_scale;
		float traceback_period_in_secs;
	} sr_arguments_;
	//ROS stuff
	ros::ServiceClient serviceClient_;
	ros::NodeHandle nh_;
	ros::Publisher pub_;

	//pipeline elements
	GstElement *pipeline_, *source_, *sink_, *resample_, *convert_, *recog_;

	boost::thread gst_thread_;
	GMainLoop *loop_;
};

#include "speech_recognition_simple.hpp"

using namespace std;

SpeechRecognitionSimple::SpeechRecognitionSimple()
{
	parseArguments();
	pub_ = nh_.advertise<std_msgs::String>("raw_result", 10, true);
	loop_ = g_main_loop_new(NULL, false);
	setupPipeline();
	gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_PLAYING);
	gst_thread_ = boost::thread(boost::bind(g_main_loop_run, loop_));
}

SpeechRecognitionSimple::~SpeechRecognitionSimple()
{
	g_main_loop_quit(loop_);
	gst_element_set_state(pipeline_, GST_STATE_NULL);
	gst_object_unref(pipeline_);
	g_main_loop_unref(loop_);

}

gboolean SpeechRecognitionSimple::onBusMessage(GstBus* bus, GstMessage* msg, gpointer userData)
{
	SpeechRecognitionSimple *server = reinterpret_cast<SpeechRecognitionSimple*>(userData);

	switch (GST_MESSAGE_TYPE(msg))
	{
	case GST_MESSAGE_EOS:
	{
		ROS_INFO("End-of-stream");
	}
	case GST_MESSAGE_ERROR:
	{
		gchar* debug;
		GError* err;

		gst_message_parse_error(msg, &err, &debug);
		g_free(debug);
		g_error_free(err);
		server->exitOnMainThread(err->message, err->code);
		break;
	}
	default:
		break;
	}
	return true;
}

GstFlowReturn SpeechRecognitionSimple::onNewRecognition(
		GstAppSink *appsink, gpointer userData)
{
	SpeechRecognitionSimple *server = reinterpret_cast<SpeechRecognitionSimple*>(userData);
	GstMapInfo map;
	GstSample *sample;
	g_signal_emit_by_name(appsink, "pull-sample", &sample);

	GstBuffer *buffer = gst_sample_get_buffer(sample);

	std_msgs::String msg;
	gst_buffer_map(buffer, &map, GST_MAP_READ);
	msg.data.resize(map.size);

	memcpy(&msg.data[0], map.data, map.size);

	server->publish(msg);
	return GST_FLOW_OK;
}

void SpeechRecognitionSimple::publish( const std_msgs::String &msg )
{
	pub_.publish(msg);
}


void SpeechRecognitionSimple::exitOnMainThread(const std::string &message, int code)
{
	ROS_ERROR("%s\n", message.c_str());
	exit(code);
}

void SpeechRecognitionSimple::parseArguments()
{
	ros::param::param<string>("~dst", destination_type_, "appsink");
	ros::param::param<bool>("~threaded_decoder", sr_arguments_.threaded_decoder, true);
	ros::param::param<bool>("~do_endpointing", sr_arguments_.do_endpointing, true);
	ros::param::param<string>("~model_dir", sr_arguments_.model_dir, "error");

	if ( sr_arguments_.model_dir == "error")
	{
		exitOnMainThread("Missing model_dir parameter",-2);
	}
	ros::param::param<string>("~model", sr_arguments_.model, sr_arguments_.model_dir + "/final.mdl");
	ros::param::param<string>("~fst", sr_arguments_.fst, sr_arguments_.model_dir + "/HCLG.fst");
	ros::param::param<string>("~word_syms", sr_arguments_.word_syms, sr_arguments_.model_dir + "/words.txt");
	ros::param::param<string>("~feature_type", sr_arguments_.feature_type, "mfcc");
	ros::param::param<string>("~mfcc_config", sr_arguments_.mfcc_config, sr_arguments_.model_dir + "/conf/mfcc.conf");
	ros::param::param<string>("~ivector_extraction_config", sr_arguments_.ivector_extraction_config, sr_arguments_.model_dir + "/conf/ivector_extractor.fixed.conf");
	ros::param::param<string>("~ep_silence_phones", sr_arguments_.ep_silence_phones, "1:2:3:4:5:6:7:8:9:10");

	ros::param::param<int>("~max_active", sr_arguments_.max_active, 7000);
	ros::param::param<float>("~beam", sr_arguments_.beam, 11.0);
	ros::param::param<float>("~lattice_beam", sr_arguments_.lattice_beam, 5.0);
	ros::param::param<float>("~chunk_length_s", sr_arguments_.chunk_length_s, 0.2);
}

GstElement* SpeechRecognitionSimple::getSpeechRecognition()
{
	GstElement* recog = gst_element_factory_make("kaldinnet2onlinedecoder", "recog");
	if (!recog)
		exitOnMainThread("Failed to locate kaldinnet2onlinedecoder\n", -3);
	g_object_set(G_OBJECT(recog), "use-threaded-decoder", sr_arguments_.threaded_decoder, NULL);
	g_object_set(G_OBJECT(recog), "model", sr_arguments_.model.c_str(), NULL);
	g_object_set(G_OBJECT(recog), "fst", sr_arguments_.fst.c_str(), NULL);
	g_object_set(G_OBJECT(recog), "word-syms", sr_arguments_.word_syms.c_str(), NULL);
	g_object_set(G_OBJECT(recog), "feature-type", sr_arguments_.feature_type.c_str(), NULL);
	g_object_set(G_OBJECT(recog), "mfcc-config", sr_arguments_.mfcc_config.c_str(), NULL);
	g_object_set(G_OBJECT(recog), "ivector-extraction-config", sr_arguments_.ivector_extraction_config.c_str(), NULL);
	g_object_set(G_OBJECT(recog), "max-active", sr_arguments_.max_active, NULL);
	g_object_set(G_OBJECT(recog), "beam", sr_arguments_.beam, NULL);
	g_object_set(G_OBJECT(recog), "lattice-beam", sr_arguments_.lattice_beam, NULL);
	g_object_set(G_OBJECT(recog), "do-endpointing", sr_arguments_.do_endpointing, NULL);
	g_object_set(G_OBJECT(recog), "endpoint-silence-phones", sr_arguments_.ep_silence_phones.c_str(), NULL);
	g_object_set(G_OBJECT(recog), "chunk-length-in-secs", sr_arguments_.chunk_length_s, NULL);
	return recog;
}

void SpeechRecognitionSimple::setupPipeline()
{

	pipeline_ = gst_pipeline_new("ros_pipeline");

	source_ = gst_element_factory_make("alsasrc", "alsa_source");
	convert_ = gst_element_factory_make("audioconvert", "convert");
	resample_ = gst_element_factory_make("audioresample", "resample");
	recog_ = getSpeechRecognition();
	if (destination_type_ == "appsink")
		{
			sink_ = gst_element_factory_make("appsink", "sink");
			g_object_set(G_OBJECT(sink_), "emit-signals", true, NULL);
			g_object_set(G_OBJECT(sink_), "max-buffers", 100, NULL);
			g_signal_connect( G_OBJECT(sink_), "new-sample", G_CALLBACK(onNewRecognition), this);
		} else {
			sink_ = gst_element_factory_make("filesink", "sink");
			g_object_set(G_OBJECT(sink_), "location", "/dev/stdout", NULL);
			g_object_set(G_OBJECT(sink_), "buffer-mode", 2, NULL);
		}

	gst_bin_add_many(GST_BIN(pipeline_), source_, convert_, resample_, recog_, sink_, NULL);

	if (!gst_element_link_many(source_, convert_, resample_, recog_, sink_, NULL))
	{
		exitOnMainThread("Failed to link pipeline", -1);
	}

	GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
	gst_bus_add_watch(bus, onBusMessage, this);
	gst_object_unref(bus);
}

void SigIntHandler(int sig)
{
	ROS_INFO("play_audio_speex sigint handler");
	ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "audio_play", ros::init_options::NoSigintHandler);
	gst_init(&argc, &argv);

	SpeechRecognitionSimple client;
	ros::spin();
	return 0;
}


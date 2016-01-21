#include "speech_recognition_speex.hpp"

using namespace std;

SpeechRecognitionSpeex::SpeechRecognitionSpeex()
{
	parseArguments();
	sub_ = nh_.subscribe("audio", 10, &SpeechRecognitionSpeex::onAudio, this);
	loop_ = g_main_loop_new(NULL, false);
	setupPipeline();
	gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_PLAYING);
	gst_thread_ = boost::thread(boost::bind(g_main_loop_run, loop_));
	paused_ = false;
}

SpeechRecognitionSpeex::~SpeechRecognitionSpeex()
{
	g_main_loop_quit(loop_);
	gst_element_set_state(pipeline_, GST_STATE_NULL);
	gst_object_unref(pipeline_);
	g_main_loop_unref(loop_);

}

void SpeechRecognitionSpeex::onNeedData(GstElement *appsrc, guint unused_size, gpointer user_data)
{
	ROS_WARN("need-data signal emitted! Pausing the pipeline");
	SpeechRecognitionSpeex *client = reinterpret_cast<SpeechRecognitionSpeex*>(user_data);
	gst_element_set_state(GST_ELEMENT(client->pipeline_), GST_STATE_PAUSED);
	client->paused_ = true;
}

void SpeechRecognitionSpeex::onAudio(const audio_common_msgs::AudioDataConstPtr &msg)
{
	if (paused_)
	{
		ROS_INFO("Got first audio after pause, resuming.");
		gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_PLAYING);
		paused_ = false;
	}
	GstBuffer *buffer = gst_buffer_new_and_alloc(msg->data.size());
	gst_buffer_fill(buffer, 0, &msg->data[0], msg->data.size());
	GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(source_), buffer);
}

gboolean SpeechRecognitionSpeex::onBusMessage(GstBus* bus, GstMessage* msg, gpointer userData)
{
	SpeechRecognitionSpeex *server = reinterpret_cast<SpeechRecognitionSpeex*>(userData);

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

void SpeechRecognitionSpeex::exitOnMainThread(const std::string &message, int code)
{
	ROS_ERROR("%s\n", message.c_str());
	exit(code);
}

void SpeechRecognitionSpeex::parseArguments()
{
	ros::param::param<string>("~dst", destination_type_, "appsink");
	ros::param::param<bool>("~enh", perceptual_enhancement_, false);

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

	ROS_INFO("Perceptual enhancement: %d.", perceptual_enhancement_);
}

GstCaps*
SpeechRecognitionSpeex::getSinkCapabilities()
{
	serviceClient_ = nh_.serviceClient<capture_vad_speex::GetSinkCapabilities>("/audio_capture/get_sink_capabilities");
	serviceClient_.waitForExistence();
	capture_vad_speex::GetSinkCapabilitiesRequest req;
	capture_vad_speex::GetSinkCapabilitiesResponse resp;
	if (!serviceClient_.call(req, resp))
		exitOnMainThread("Failed GetSinkCapabilities call!",-1);
	ROS_INFO("Got caps for appsrc!");
	return gst_caps_from_string(resp.capabilties.c_str());
}

GstElement*
SpeechRecognitionSpeex::getSpeechRecognition()
{
	GstElement* recog = gst_element_factory_make("kaldinnet2onlinedecoder", "recog");
	if(!recog)
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

void SpeechRecognitionSpeex::setupPipeline()
{
	pipeline_ = gst_pipeline_new("app_pipeline");
	source_ = gst_element_factory_make("appsrc", "app_source");
	g_object_set (G_OBJECT (source_),
			"stream-type", 0,
			"format", GST_FORMAT_TIME, NULL);
	gst_bin_add(GST_BIN(pipeline_), source_);
	g_signal_connect(source_, "need-data", G_CALLBACK(onNeedData), this);

	//tell appsrc speexenc capabilities
	GstCaps* caps = getSinkCapabilities();
	gst_app_src_set_caps(GST_APP_SRC(source_), caps);
	gst_caps_unref(caps);
	decoder_ = gst_element_factory_make("speexdec", "speex_decoder");
	g_object_set(G_OBJECT(decoder_), "enh", perceptual_enhancement_, NULL);
	convert_ = gst_element_factory_make("audioconvert", "convert");

	if (destination_type_ == "appsink")
	{
		assert("NOT YET IMPLEMENTED\n");
	}
	else
	{
		sink_ = gst_element_factory_make("filesink", "sink");
		g_object_set(G_OBJECT(sink_), "location", "/dev/stdout", NULL);
		g_object_set(G_OBJECT(sink_), "buffer-mode", 2, NULL);
		resample_ = gst_element_factory_make("audioresample", "resample");
		recog_ = getSpeechRecognition();
		gst_bin_add_many(GST_BIN(pipeline_), decoder_, convert_, resample_, recog_, sink_, NULL);

		if (!gst_element_link_many(source_, decoder_, convert_, resample_, recog_, sink_,
		NULL))
		{
			exitOnMainThread("Failed to link pipeline", -1);
		}

	}

	GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
	gst_bus_add_watch(bus, onBusMessage, this);
	gst_object_unref(bus);

}

void SigIntHandler(int sig)
{
// Do some custom action.
// For example, publish a stop message to some other nodes.

// All the default sigint handler does is call shutdown()
	ROS_INFO("play_audio_speex sigint handler");
	ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "audio_play", ros::init_options::NoSigintHandler);
	gst_init(&argc, &argv);

	SpeechRecognitionSpeex client;
	ros::spin();
	return 0;
}


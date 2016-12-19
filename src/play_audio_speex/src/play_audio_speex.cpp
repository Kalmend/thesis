#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <ros/ros.h>
#include <signal.h>
#include <boost/thread.hpp>

#include "audio_common_msgs/AudioData.h"
#include "capture_vad_speex/GetSinkCapabilities.h"

using namespace std;

class SpeechRecognitionSimple
{
public:
	SpeechRecognitionSimple()
	{
		parseArguments();
		sub_ = nh_.subscribe("audio", 10, &SpeechRecognitionSimple::onAudio, this);
		loop_ = g_main_loop_new(NULL, false);
		setupPipeline();
		gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_PLAYING);
		gst_thread_ = boost::thread( boost::bind(g_main_loop_run, loop_) );
		paused_ = false;
	}


	~SpeechRecognitionSimple()
	{
		g_main_loop_quit(loop_);
		gst_element_set_state(pipeline_, GST_STATE_NULL);
		gst_object_unref(pipeline_);
		g_main_loop_unref(loop_);

	}


private:

	static void onNeedData (GstElement *appsrc,
			guint       unused_size,
			gpointer    user_data)
	{
		ROS_WARN("need-data signal emitted! Pausing the pipeline");
		SpeechRecognitionSimple *client = reinterpret_cast<SpeechRecognitionSimple*>(user_data);
		gst_element_set_state(GST_ELEMENT(client->pipeline_), GST_STATE_PAUSED);
		client->paused_ = true;
	}

	void onAudio(const audio_common_msgs::AudioDataConstPtr &msg)
	{
		if(paused_)
		{
			ROS_INFO("Got first audio after pause, resuming.");
			gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_PLAYING);
			paused_ = false;
		}
		GstBuffer *buffer = gst_buffer_new_and_alloc(msg->data.size());
		gst_buffer_fill(buffer, 0, &msg->data[0], msg->data.size());
		GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(source_),buffer);
	}

	static gboolean onBusMessage(GstBus* bus, GstMessage* msg, gpointer userData)
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

				ROS_ERROR("ERROR: %s", err->message);
				g_error_free(err);
				server->exitOnMainThread(err->code);
				break;
			}
			default:
				break;
			}
			return true;
		}

	void exitOnMainThread(int code)
	{
		exit(code);
	}

	void parseArguments()
	{
		ros::param::param<string>("~dst", destination_type_, "alsasink");
		ros::param::param<bool>("~enh", perceptual_enhancement_, false);
		ROS_INFO("Perceptual enhancement: %d.", perceptual_enhancement_);
	}

	GstCaps* getSinkCapabilities()
	{
		serviceClient_ = nh_.serviceClient<capture_vad_speex::GetSinkCapabilities>("/capture_vad_speex/get_sink_capabilities");
		serviceClient_.waitForExistence();
		capture_vad_speex::GetSinkCapabilitiesRequest req;
		capture_vad_speex::GetSinkCapabilitiesResponse resp;
		if(!serviceClient_.call(req, resp))
		{
			ROS_ERROR("Failed GetSinkCapabilities call!");
			exitOnMainThread(-1);
		}
		ROS_INFO("Got caps for appsrc!");
		return gst_caps_from_string(resp.capabilties.c_str());
	}

	void setupPipeline()
	{
		pipeline_ = gst_pipeline_new("app_pipeline");
		source_ = gst_element_factory_make("appsrc", "app_source");
		g_object_set (G_OBJECT (source_),
				"stream-type", 0,
				"format", GST_FORMAT_TIME, NULL);
		gst_bin_add( GST_BIN(pipeline_), source_);
		g_signal_connect(source_, "need-data", G_CALLBACK(onNeedData),this);

		if (destination_type_ == "alsasink")
		{
			//tell appsrc speexenc capabilities
			GstCaps* caps = getSinkCapabilities();
			gst_app_src_set_caps(GST_APP_SRC(source_),caps);
			gst_caps_unref(caps);
			decoder_ = gst_element_factory_make("speexdec", "speex_decoder");
			g_object_set( G_OBJECT(decoder_), "enh", perceptual_enhancement_, NULL);
			convert_ = gst_element_factory_make("audioconvert", "convert");
			sink_ = gst_element_factory_make("autoaudiosink", "sink");
			gst_bin_add_many(GST_BIN(pipeline_), decoder_, convert_, sink_, NULL);
			if(!gst_element_link_many(source_, decoder_, convert_, sink_, NULL))
			{
				ROS_ERROR("Failed to link pipeline");
				exitOnMainThread(-1);
			}

		} else {
			sink_ = gst_element_factory_make("filesink", "sink");
			g_object_set( G_OBJECT(sink_), "location", destination_type_.c_str(), NULL);
			gst_bin_add(GST_BIN(pipeline_), sink_);
			gst_element_link(source_, sink_);
		}

		GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
		gst_bus_add_watch(bus, onBusMessage, this);
		gst_object_unref(bus);

	}

	//arguments
	string destination_type_;
	bool perceptual_enhancement_;
	//ROS stuff
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::ServiceClient serviceClient_;

	//pipeline elements
	GstElement *pipeline_, *source_, *sink_, *decoder_, *resample_, *convert_;

	boost::thread gst_thread_;
	GMainLoop *loop_;
	bool paused_;
};

void SigIntHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.

  // All the default sigint handler does is call shutdown()
  ROS_INFO("play_audio_speex sigint handler");
  ros::shutdown();
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "play_audio_speex", ros::init_options::NoSigintHandler);
	gst_init(&argc, &argv);

	SpeechRecognitionSimple client;
	ros::spin();
	return 0;
}


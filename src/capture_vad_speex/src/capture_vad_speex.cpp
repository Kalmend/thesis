#include <stdio.h>
#include <string>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include "audio_common_msgs/AudioData.h"
#include "capture_vad_speex/GetSinkCapabilities.h"

using namespace std;

class RosGstSpeexCapture
{
public:
	RosGstSpeexCapture() : destination_type_("appsink"), bitrate_(0)/*, publishBackoff_(0)*/
{
		parseArguments();
		// The source of the audio
		pub_ = nh_.advertise<audio_common_msgs::AudioData>("audio", 10, true);
		loop_ = g_main_loop_new(NULL, false);
		setupPipeline();
		gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_PLAYING);
		gst_thread_ = boost::thread( boost::bind(g_main_loop_run, loop_) );
}

	~RosGstSpeexCapture()
	{
		g_main_loop_quit(loop_);
		gst_element_set_state(pipeline_, GST_STATE_NULL);
		gst_object_unref(pipeline_);
		g_main_loop_unref(loop_);
	}

private:

	static GstFlowReturn onNewBuffer (GstAppSink *appsink, gpointer userData)
	{
		RosGstSpeexCapture *server = reinterpret_cast<RosGstSpeexCapture*>(userData);
		GstMapInfo map;

		GstSample *sample;
		g_signal_emit_by_name(appsink, "pull-sample", &sample);

		GstBuffer *buffer = gst_sample_get_buffer(sample);

		audio_common_msgs::AudioData msg;
		gst_buffer_map(buffer, &map, GST_MAP_READ);
		msg.data.resize( map.size );

		memcpy( &msg.data[0], map.data, map.size );

		server->publish(msg);

		return GST_FLOW_OK;
	}

	static gboolean onMessage (GstBus *bus, GstMessage *message, gpointer userData)
	{
		RosGstSpeexCapture *server = reinterpret_cast<RosGstSpeexCapture*>(userData);
		GError *err;
		gchar *debug;

		gst_message_parse_error(message, &err, &debug);
		ROS_ERROR_STREAM("gstreamer: " << err->message);
		g_error_free(err);
		g_free(debug);
		g_main_loop_quit(server->loop_);
		server->exitOnMainThread(1);
		return FALSE;
	}


	static GstPadProbeReturn onEncoderEvent(GstPad *pad,GstPadProbeInfo *info,gpointer user_data)
	{
		GstEvent* event = gst_pad_probe_info_get_event(info);
		if(GST_EVENT_TYPE(event) == GST_EVENT_CAPS)
		{
			ROS_INFO("up and running pad added to speexenc element, setting up service...\n");
			RosGstSpeexCapture *server = reinterpret_cast<RosGstSpeexCapture*>(user_data);
			server->setupGetSinkCapabilitiesService();
			gst_pad_remove_probe(pad,info->id);
		}
		return GST_PAD_PROBE_OK;
	}

	void exitOnMainThread(int code)
	{
		exit(code);
	}

	void parseArguments()
	{
		ros::param::param<int>("~bitrate", bitrate_, 0);
		ros::param::param<string>("~dst", destination_type_, "appsink");
		ROS_INFO("speexenc bitrate: %d.", bitrate_);
	}

	void setupPipeline()
	{
		pipeline_ = gst_pipeline_new("ros_pipeline");
		GstBus * bus_  = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
		gst_bus_add_signal_watch(bus_);
		g_signal_connect(bus_ , "message::error", G_CALLBACK(onMessage), this);
		g_object_unref(bus_);

		// We create the sink first, just for convenience
		if (destination_type_ == "appsink")
		{
			sink_ = gst_element_factory_make("appsink", "sink");
			g_object_set(G_OBJECT(sink_), "emit-signals", true, NULL);
			g_object_set(G_OBJECT(sink_), "max-buffers", 100, NULL);
			g_signal_connect( G_OBJECT(sink_), "new-sample", G_CALLBACK(onNewBuffer), this);
		} else {
			cout << "file sink" << endl;
			sink_ = gst_element_factory_make("filesink", "sink");
			g_object_set( G_OBJECT(sink_), "location", destination_type_.c_str(), NULL);
		}

		source_ = gst_element_factory_make("alsasrc", "source");
		convert_ = gst_element_factory_make("audioconvert", "convert");

		gboolean link_ok;
		// We create the source and add encode to speex with VAD

		encode_ = gst_element_factory_make("speexenc", "encoder");
		g_object_set( G_OBJECT(encode_), "vad", true, NULL);
		g_object_set( G_OBJECT(encode_), "dtx", true, NULL);
		g_object_set( G_OBJECT(encode_), "bitrate", bitrate_, NULL);
		gst_pad_add_probe(GST_PAD_CAST(encode_->srcpads->data), GST_PAD_PROBE_TYPE_EVENT_BOTH, onEncoderEvent, this, NULL);

		gst_bin_add_many( GST_BIN(pipeline_), source_, convert_, encode_, sink_, NULL);
		link_ok = gst_element_link_many(source_, convert_, encode_, sink_, NULL);

		if (!link_ok) {
			ROS_ERROR_STREAM("Unsupported media type.");
			exitOnMainThread(1);
		}
	}

	void setupGetSinkCapabilitiesService()
	{
		storeSpeexCaps();
		serviceSrv_ = nh_.advertiseService("/audio_capture/get_sink_capabilities", &RosGstSpeexCapture::getSinkCaps, this);
	}

	bool getSinkCaps(capture_vad_speex::GetSinkCapabilitiesRequest &req,
			capture_vad_speex::GetSinkCapabilitiesResponse & res)
	{
		ROS_INFO("Capabilities requested from capture_vad_speex node.\n");
		ROS_INFO("Returning: %s\n", speexCaps_.c_str());
		res.capabilties = speexCaps_;
		return true;
	}

	void storeSpeexCaps()
	{
		GList* srcpads = encode_->srcpads;
		/* Walking through our pad list searching for the pad we want to release */
		if (!srcpads)
		{
			ROS_ERROR_STREAM("Can not locate srcpad of speex encoder.\n");
			exitOnMainThread(1);
			return;
		}
		GstPad* pad =  GST_PAD_CAST(srcpads->data);
		GstCaps* caps = gst_pad_get_current_caps(pad);
		gchar* capsGchar = gst_caps_to_string (caps);
		speexCaps_ = capsGchar;
		g_free(capsGchar);
	}

	void publish( const audio_common_msgs::AudioData &msg )
	{
		/*
		if(msg.data.size() <= 2) // we dont want to send the 2 byte comfort noise
			publishBackoff_ = 2; // dont send this and one more

		if(publishBackoff_ > 0)
		{
			publishBackoff_--;
			return;
		}*/
		pub_.publish(msg);
	}


	//params
	int bitrate_;
	string destination_type_;

	//ROS stuff
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	ros::ServiceServer serviceSrv_;
	string speexCaps_;
	//int publishBackoff_;

	//pipeline elements
	GstElement *pipeline_, *source_, *sink_, *convert_, *encode_;

	boost::thread gst_thread_;
	GMainLoop *loop_;
};


int main (int argc, char **argv)
{
	ros::init(argc, argv, "audio_capture");
	gst_init(&argc, &argv);
	RosGstSpeexCapture server;
	ros::spin();
}

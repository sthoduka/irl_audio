#include <manyears_ros/manyears_node.hpp>
#include "manyears_config.hpp"
#include "global_value.h" // ManyEars static consts.

using namespace manyears_ros;

ManyEarsNode::ManyEarsNode(ros::NodeHandle& n, ros::NodeHandle& np)
{
    manyears_context_ = createEmptyOverallContext();
    ParametersLoadDefault(manyears_context_.myParameters);

    if (!parseParams(np)) {
        ROS_ERROR("Could not parse ManyEars parameters correctly, the node "
                  "will not be initialized.");
        return;
    }

    ROS_INFO("Initializing ManyEars with %i microphones...",
             microphonesCount());

    initPipeline();

    sub_audio_ = n.subscribe("audio_stream", 10, &ManyEarsNode::audioCB, this);

    pub_sources_ = n.advertise<manyears_msgs::ManyEarsTrackedAudioSource>(
        "tracked_sources",
        10);

}

ManyEarsNode::~ManyEarsNode()
{
    preprocessorTerminate(       manyears_context_.myPreprocessor);
    beamformerTerminate(         manyears_context_.myBeamformer);
    mixtureTerminate(            manyears_context_.myMixture);
    gssTerminate(                manyears_context_.myGSS);
    postfilterTerminate(         manyears_context_.myPostfilter);
    postprocessorTerminate(      manyears_context_.myPostprocessorSeparated);
    postprocessorTerminate(      manyears_context_.myPostprocessorPostfiltered);
    potentialSourcesTerminate(   manyears_context_.myPotentialSources);
    trackedSourcesTerminate(     manyears_context_.myTrackedSources);
    separatedSourcesTerminate(   manyears_context_.mySeparatedSources);
    postfilteredSourcesTerminate(manyears_context_.myPostfilteredSources);
}

bool ManyEarsNode::parseParams(const ros::NodeHandle& np)
{
    if (!np.hasParam("config_file")) {
        ROS_ERROR("No 'config_file' parameter found.");
        return false;
    } else {
        std::string config_fn;
        np.getParam("config_file", config_fn);
        if (!manyears_ros::parseConfigFile(manyears_context_, config_fn)) {
            return false;
        }
    }

    if (!np.hasParam("microphones")) {
        ROS_ERROR("No 'microphones' parameter - cannot define array geometry.");
        return false;
    }

    using namespace XmlRpc;
    XmlRpcValue mic_n;
    np.getParam("microphones", mic_n);
    if (mic_n.getType() != XmlRpcValue::TypeArray) {
        ROS_ERROR("'microphones' is not an array.");
        return false;
    }

    for (int i = 0; i < mic_n.size(); ++i) {
        MicDef md;

        XmlRpcValue& v = mic_n[i];
        if (v.getType() != XmlRpcValue::TypeStruct) {
            ROS_ERROR("'microphones' element %i is not a struct, skipping",
                      i);
            continue;
        }

        if (!v.hasMember("gain")) {
            ROS_ERROR("No gain specified for microphone %i, using '1.0'",
                      i);
        } else {
            XmlRpcValue& gv = v["gain"];
            if (gv.getType() != XmlRpcValue::TypeDouble) {
                ROS_ERROR("Gain element of microphone %i is not a double, "
                          "using '1.0'",
                          i);
            } else {
                md.gain = gv;
            }
        }

        if (!v.hasMember("pos")) {
            ROS_ERROR("No pos specified for microphone %i, using '(0, 0, 0)'",
                      i);
        } else {
            XmlRpcValue& posv = v["pos"];
            if (posv.getType() != XmlRpcValue::TypeStruct) {
                ROS_ERROR("Pos element of microphone %i is not a struct, "
                          "using (0, 0, 0).");
            } else {
                if (posv.hasMember("x")) {
                    XmlRpcValue& xv = posv["x"];
                    if (xv.getType() == XmlRpcValue::TypeDouble) {
                        md.pos[0] = xv;
                    } else {
                        ROS_WARN("'x' element of microphone %i is not a "
                                 "double, using 0.0.",
                                 i);
                    }
                }
                if (posv.hasMember("y")) {
                    XmlRpcValue& yv = posv["y"];
                    if (yv.getType() == XmlRpcValue::TypeDouble) {
                        md.pos[1] = yv;
                    } else {
                        ROS_WARN("'y' element of microphone %i is not a "
                                 "double, using 0.0.",
                                 i);
                    }
                }
                if (posv.hasMember("z")) {
                    XmlRpcValue& zv = posv["z"];
                    if (zv.getType() == XmlRpcValue::TypeDouble) {
                        md.pos[2] = zv;
                    } else {
                        ROS_WARN("'z' element of microphone %i is not a "
                                 "double, using 0.0.",
                                 i);
                    }
                }
            }
        }

        mic_defs_.push_back(md);
    }

    if (microphonesCount() < 2) {
        ROS_ERROR("Less than two microphones were defined.");
        return false;
    }

    np.param("instant_time", instant_time_,                             true);
    np.param("frame_id",     frame_id_,     std::string("micro_center_link"));

    double gain_sep, gain_pf;
    np.param("gain_sep", gain_sep, 1.0);
    np.param("gain_pf", gain_pf, 1.0);
    gain_sep_ = gain_sep;
    gain_pf_  = gain_pf;

    return true;
}

void ManyEarsNode::initPipeline()
{
    microphonesInit(manyears_context_.myMicrophones, mic_defs_.size());
    for (int i = 0; i < mic_defs_.size(); ++i) {
        const MicDef& md = mic_defs_[i];
        microphonesAdd(manyears_context_.myMicrophones,
                      i,
                      md.pos[0],
                      md.pos[1],
                      md.pos[2],
                      md.gain);
    }

    preprocessorInit(       manyears_context_.myPreprocessor,
                            manyears_context_.myParameters,
                            manyears_context_.myMicrophones);
    beamformerInit(         manyears_context_.myBeamformer,
                            manyears_context_.myParameters,
                            manyears_context_.myMicrophones);
    mixtureInit(            manyears_context_.myMixture,
                            manyears_context_.myParameters);
    gssInit(                manyears_context_.myGSS,
                            manyears_context_.myParameters,
                            manyears_context_.myMicrophones);
    postfilterInit(         manyears_context_.myPostfilter,
                            manyears_context_.myParameters);
    postprocessorInit(      manyears_context_.myPostprocessorSeparated,
                            manyears_context_.myParameters);
    postprocessorInit(      manyears_context_.myPostprocessorPostfiltered,
                            manyears_context_.myParameters);
    potentialSourcesInit(   manyears_context_.myPotentialSources,
                            manyears_context_.myParameters);
    trackedSourcesInit(     manyears_context_.myTrackedSources,
                            manyears_context_.myParameters);
    separatedSourcesInit(   manyears_context_.mySeparatedSources,
                            manyears_context_.myParameters);
    postfilteredSourcesInit(manyears_context_.myPostfilteredSources,
                            manyears_context_.myParameters);

}

ros::Time ManyEarsNode::getTimeStamp() const
{
    if (instant_time_) {
        return ros::Time::now();
    } else {
        return processed_time_;
    }
}

void ManyEarsNode::audioCB(const rt_audio_ros::AudioStream::ConstPtr& msg)
{
    // Fixed ManyEars input buffer size, in bytes:
    static const int BUFFER_SIZE = manyears_global::samples_per_frame_s *
                                   microphonesCount() *
                                   sizeof(int16_t);

    // NOTE: Always calculate processed time, even if frames are skipped:
    if (processed_time_.isZero()) {
        processed_time_ = ros::Time::now();
    } else {
        processed_time_ += ros::Duration(
            float(manyears_global::samples_per_frame_s) / 
            float(manyears_global::sample_rate_s));
    }
   
    if (msg->channels != microphonesCount()) {
        ROS_ERROR_THROTTLE(1.0,
                           "Received an audio stream packet with %i channels, "
                           "expected %i. Packet skipped.",
                            microphonesCount(),
                            msg->channels);
        return;
    }

    if (msg->encoding != rt_audio_ros::AudioStream::SINT_16_PCM) {
        ROS_ERROR_THROTTLE(1.0,
                           "Received an audio stream packet not in "
                           "SINT_16_PCM format. Packet skipped.");
        return;
    }

    if (msg->is_bigendian != false) {
        ROS_ERROR_THROTTLE(1.0,
                           "Received an audio stream packet not in "
                           "little endian. Packet skipped.");
        return;
    }

    if (msg->sample_rate != manyears_global::sample_rate_s) {
        ROS_ERROR_THROTTLE(1.0,
                           "Received an audio stream packet sampled at "
                           "%i Hz, expecting %i. Packet skipped.",
                           msg->sample_rate,
                           manyears_global::sample_rate_s);
        return;
    }
    
    if (msg->data.size() != BUFFER_SIZE) {
        ROS_ERROR_THROTTLE(1.0,
                           "Received an audio stream packet of size %i bytes, "
                           "ManyEars expects a fixed buffer size of %i bytes. "
                           "Packet skipped.",
                           msg->data.size(),
                           BUFFER_SIZE);
        return;
    }

    // First, convert buffer to floats.
    typedef std::vector<float>    FloatVec;
    typedef std::vector<FloatVec> OutputBuffer;
    OutputBuffer buffer_out = OutputBuffer(microphonesCount());
    for (int i = 0; i < buffer_out.size(); ++i) {
        buffer_out[i].resize(manyears_global::samples_per_frame_s);
    }

    const int16_t* buffer_in = reinterpret_cast<const int16_t*>(&(msg->data[0]));
    int i = 0; // Incremented when buffer_in is read.
    for (int s = 0; s < manyears_global::samples_per_frame_s; ++s) {
        for (int c = 0; c < microphonesCount(); ++c) {
            float v = float(buffer_in[i++]) / SHRT_MAX;
            buffer_out[c][s] = v;
        }
    }

    // Push all frames to ManyEars and process.
    for (int i = 0; i < buffer_out.size(); ++i) {
        preprocessorPushFrames(manyears_context_.myPreprocessor,
                               buffer_out[i].size(),
                               i);
        preprocessorAddFrame(  manyears_context_.myPreprocessor,
                               &(buffer_out[i][0]),
                               i,
                               buffer_out[i].size());
    }
    preprocessorProcessFrame(manyears_context_.myPreprocessor);
    beamformerFindMaxima(    manyears_context_.myBeamformer,
                             manyears_context_.myPreprocessor,
                             manyears_context_.myPotentialSources);
    mixtureUpdate(           manyears_context_.myMixture,
                             manyears_context_.myTrackedSources,
                             manyears_context_.myPotentialSources);

    if (enable_sep_) {
        gssProcess(       manyears_context_.myGSS,
                          manyears_context_.myPreprocessor,
                          manyears_context_.myTrackedSources,
                          manyears_context_.mySeparatedSources);
        postfilterProcess(manyears_context_.myPostfilter,
                          manyears_context_.mySeparatedSources,
                          manyears_context_.myPreprocessor,
                          manyears_context_.myPostfilteredSources);
        postprocessorProcessFrameSeparated(
            manyears_context_.myPostprocessorSeparated,
            manyears_context_.myTrackedSources,
            manyears_context_.mySeparatedSources);
        postprocessorProcessFramePostfiltered(
            manyears_context_.myPostprocessorPostfiltered,
            manyears_context_.myTrackedSources,
            manyears_context_.myPostfilteredSources);
    }

    // Output formatting.
    manyears_msgs::ManyEarsTrackedAudioSource msg_out;
    msg_out.header.frame_id = frame_id_;
    msg_out.header.stamp    = getTimeStamp();

    objTrackedSources* sources = manyears_context_.myTrackedSources;
    for (int i = 0; i < manyears_context_.myParameters->P_GEN_DYNSOURCES; ++i) {
        int src_id = trackedSourcesGetID(sources, i);
        if (src_id != -1) {
            int last_s = msg_out.tracked_sources.size();
            msg_out.tracked_sources.resize(last_s + 1);
            manyears_msgs::SourceInfo& src = msg_out.tracked_sources[last_s];

            src.source_id = src_id; 

            double& px = src.source_pos.x;
            double& py = src.source_pos.y;
            double& pz = src.source_pos.z;
            px = trackedSourcesGetX(sources, i);
            py = trackedSourcesGetY(sources, i);
            pz = trackedSourcesGetZ(sources, i);

            src.longitude = atan2(py, px)                       * 180.0 / M_PI;
            src.latitude  = asin( pz / (px*px + py*py + pz*pz)) * 180.0 / M_PI;

            if (enable_sep_) {
                static const int SIZE = GLOBAL_FRAMESIZE * GLOBAL_OVERLAP;

                src.separation_data.resize(SIZE);
                postprocessorExtractHop(
                    manyears_context_.myPostprocessorSeparated,
                    src.source_id,
                    &(src.separation_data[0]));
                src.postfiltered_data.resize(SIZE);
                postprocessorExtractHop(
                    manyears_context_.myPostprocessorPostfiltered,
                    src.source_id,
                    &(src.postfiltered_data[0]));

                // Output gain.
                for (int j = 0; j < SIZE; ++j) {
                    src.separation_data[j]   *= gain_sep_;
                    src.postfiltered_data[j] *= gain_pf_;
                }
            }

            src.source_probability = potentialSourcesGetProbability(
                manyears_context_.myPotentialSources,
                i);
        }
    }

    pub_sources_.publish(msg_out);

}


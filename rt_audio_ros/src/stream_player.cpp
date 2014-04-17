/// \file stream_player.cpp Simple raw stream player that outputs a sequence of
///                         AudioStream messages from standard input.

#include <rt_audio_ros/AudioStream.h>
#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stream_player");

    // TODO: More parameters!
    
    int nb_channels = 2;
    if (argc >= 2) {
        nb_channels = atoi(argv[1]);
    }

    ros::NodeHandle n;
    ros::Publisher  pub_stream = 
        n.advertise<rt_audio_ros::AudioStream>("audio_stream", 10);


    const int BUFFER_SIZE = 512 * nb_channels * sizeof(int16_t);
    rt_audio_ros::AudioStream msg;
    msg.encoding     = rt_audio_ros::AudioStream::SINT_16_PCM;
    msg.is_bigendian = false;
    msg.channels     = nb_channels;
    msg.sample_rate  = 48000;
    msg.data.resize(BUFFER_SIZE);
    unsigned char* buffer = &(msg.data[0]);

    ros::Rate rate(msg.sample_rate / (512 * nb_channels));
    while (ros::ok() && !std::cin.read((char*)buffer, BUFFER_SIZE).eof()) {
        msg.header.stamp = ros::Time::now();
        pub_stream.publish(msg);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}


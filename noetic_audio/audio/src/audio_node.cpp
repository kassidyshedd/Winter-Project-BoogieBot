#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"

// Create States
enum State{
    WAITING,
    BEAT_DETECTION,
    TRIGGER,
    DONE,
    RESET
};
 
class Audio
{
    public:
        Audio() : nh(), state(WAITING) 
        {
            // Publishers
            // Tempo publisher
            num_tempo_pub = nh.advertise<std_msgs::Int64>("state_num_tempo", 10);
            // call to python node
            start_pub = nh.advertise<std_msgs::String>("start", 10);

            // Subscribers
            // Listen subscriber
            listen_sub = nh.subscribe("listening", 10, &Audio::listen_callback, this);
            // subscribe to num_tempo
            num_tempo_sub = nh.subscribe("num_tempo", 10, &Audio::num_tempo_callback, this);

        }

        void run() 
        {
            ros::Rate loop_rate(100);

            while (ros::ok())
            {
                timer_callback();
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher start_pub;
        ros::Publisher num_tempo_pub;

        ros::Subscriber listen_sub;
        ros::Subscriber reset_sub;
        ros::Subscriber num_tempo_sub;

        State state;
        bool first_message;
        bool reset_message;
        bool tempo_message;
        int tempo;


        void listen_callback(const std_msgs::String::ConstPtr& msg)
        {   
            if (!first_message)
            {
                ROS_INFO("Received message on 'listening' topic, switch to TRIGGER state.");
                state = TRIGGER;
                first_message = true;
            }
        }

        void num_tempo_callback(const std_msgs::Int64::ConstPtr& msg){
            ROS_INFO_ONCE("Inside num_tempo callback!");
            {
                if (!tempo_message)
                {
                    ROS_INFO_ONCE("Audio - Recieved message on num_tempo topic, switch to beat_detection state.");
                    tempo = msg->data;
                    tempo_message = true;
                    state = BEAT_DETECTION;
                }
            }
        }

        void timer_callback()
        {
            if (state == WAITING) 
            {
                ROS_INFO_ONCE("Waiting State");                
            }
            else if (state == TRIGGER)
            {
                ROS_INFO_ONCE("Trigger State");                
                std_msgs::String msg;
                msg.data = "Tempo!";
                start_pub.publish(msg);
                ROS_INFO_ONCE("Published message on 'start' topic, call to python node. ");  
            }
            else if (state == BEAT_DETECTION)
            {
                ROS_INFO_ONCE("Beat Detection State");                

                std_msgs::Int64 tempo_msg;
                tempo_msg.data = tempo;
                num_tempo_pub.publish(tempo_msg);
                ROS_INFO_ONCE("Published message on 'state_num_tempo' topic.");

                state = RESET;
            }
            else if (state == DONE)
            {
                ROS_INFO_ONCE("Done State");
            }

            // Reset all flags
            else if (state == RESET)
            {
                ROS_INFO_ONCE("Reset State");

                first_message = false;
                reset_message = false;
                
                ROS_INFO_ONCE("Reset all flags, switching to waiting state");  
                state = WAITING;
            }
        }
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "audio_node");
    Audio audio;
    audio.run();
    return 0;
}
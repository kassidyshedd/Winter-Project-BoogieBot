#include "ros/ros.h"
#include "std_msgs/String.h"
// Create states
enum State{
    WAITING,
    PLAYING,
    DONE
};

class ActionPlay
{
    public:
        ActionPlay() : nh(), state(WAITING)
        {   
            // Publishers
            // Done action publisher
            done_action_pub = nh.advertise<std_msgs::String>("done", 10);

            // Subscribers
            // Start action subscriber
            start_action_sub = nh.subscribe("startplay", 10, &ActionPlay::start_callback, this);
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

        ros::Publisher done_action_pub;

        ros::Subscriber start_action_sub;

        State state;
        bool first_message;

        void start_callback(const std_msgs::String::ConstPtr& msg)
        {
            if (!first_message)
            {
                ROS_INFO("Received message on 'start' topic, switching to PLAYING state.");
                first_message = true;
                state = PLAYING;
            }
        }

        void timer_callback()
        {
            if (state == WAITING)
            {
                ROS_INFO_ONCE("PLAYER - Waiting State.");
            }
            else if (state == PLAYING)
            {
                ROS_INFO_ONCE("PLAYER - Playing State.");
                state = RESET;
            }
            else if (state == DONE)
            {
                ROS_INFO_ONCE("PLAYER - Done State");

                std_msgs::String msg;
                msg.data = "Done!";
                done_action_pub.publish(msg);
                ROS_INFO_ONCE("Published message on 'done' topic, switching to waiting state.");
                state = WAITING;
            }
        }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "actionplay_node");
    ActionPlay actionPlay;
    actionPlay.run();
    return 0;
}

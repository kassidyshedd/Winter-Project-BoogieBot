#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_srvs/Empty.h"
#include "state/RandomList.h"


// Crreare States 
enum State{
    WAITING,
    LISTENING,
    ACTION_PICKER, 
    ACTION_EDITOR,
    ACTION_PLAYER, 
    RESET
};

class BoogieBot
{
    public:
        BoogieBot() : nh(), state(WAITING) 
        {
            // Publishers
            // Listen publlisher - causes switch in audio_node
            listen_pub = nh.advertise<std_msgs::String>("listening", 10);
            // Tempo publisher - causes switch in action_picking_node
            action_pick_pub = nh.advertise<std_msgs::String>("send_tempo", 10);
            action_picking_pub = nh.advertise<std_msgs::Int64>("send_num_tempo", 10);
            // Action list publisher - causes switch in action_editing_node
            action_edit_pub = nh.advertise<state::RandomList>("Send_List", 10);
            // Action edit publisher - causes switch in action_playing node
            action_play_pub = nh.advertise<std_msgs::String>("STARTING", 10);
            // mimic button
            button_pub = nh.advertise<std_msgs::String>("/robotis/open_cr/button", 10);
            // reset pub - causes variables in all nodes to reset
            reset_pub = nh.advertise<std_msgs::String>("reset", 10);

            // Subscribers
            // Tempo subscriber - when msg recieved, switch to action_picking state
            num_tempo_sub = nh.subscribe("state_num_tempo", 10, &BoogieBot::num_tempo_callback, this);
            // Action List subscriber - when msg recieved, switch to action_editing state
            action_list_sub = nh.subscribe("list_send", 10, &BoogieBot::actionlist_callback, this);
            // Action Edit subscriber - when msg recieved, switch to action_playing state
            action_edit_sub = nh.subscribe("get_ready", 10, &BoogieBot::actionedit_callback, this);
            // Action play subscriber - when msg recieved, switch to reset state
            action_play_sub = nh.subscribe("done", 10, &BoogieBot::actionplay_callback, this);


            // Services
            // Listen service
            listen_srv = nh.advertiseService("listen_trigger", &BoogieBot::listensrv_callback, this);
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

        ros::Publisher listen_pub;
        ros::Publisher action_pick_pub;
        ros::Publisher action_picking_pub;
        ros::Publisher action_edit_pub;
        ros::Publisher action_play_pub;
        ros::Publisher reset_pub;
        ros::Publisher button_pub;

        ros::ServiceServer listen_srv;

        ros::Subscriber num_tempo_sub;
        ros::Subscriber action_list_sub;
        ros::Subscriber action_edit_sub;
        ros::Subscriber action_play_sub;

        State state;
        bool first_message_tempo;
        bool first_message_actionlist;
        bool first_message_actionedit;
        bool first_message_actionplay;
        int tempo;
        std::vector<int64_t> list;

        bool listensrv_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
        {
            ROS_INFO("State - Recieved 'listen_trigger' request.");
            state = LISTENING;
            return true;
        }

        void num_tempo_callback(const std_msgs::Int64::ConstPtr& msg)
        {   ROS_INFO_STREAM(first_message_tempo);
            ROS_INFO("State -Inside num_tempo_callback");
            if (!first_message_tempo)
            {
                ROS_INFO("State -Recieved meessage on 'state_num_tempo' topic, switch to action_list state.");
                tempo = msg->data;
                first_message_tempo = true;
                state = ACTION_PICKER;
            }
        }
 
        void actionlist_callback(const state::RandomList::ConstPtr& msg)
        {
            if (!first_message_actionlist)
            {
                ROS_INFO("State -Recieved message on 'list_sent; topic, switch to action_edit state. ");
                list = msg->nums;
                first_message_actionlist = true;
                state = ACTION_EDITOR;
            }
        }

        void actionedit_callback(const std_msgs::String::ConstPtr& msg)
        {
            if (!first_message_actionedit)
            {
                ROS_INFO("State -Recieved message on 'get_ready' topic, switch to action_play state. ");
                state = ACTION_PLAYER;
                first_message_actionedit = true;
            }
        }

        void actionplay_callback(const std_msgs::String::ConstPtr& msg)
        {
            if (!first_message_actionplay)
            {
                ROS_INFO("State -Recieved message on 'done' topic, switch to reset state. ");
                state = RESET;
                first_message_actionplay = true;
            }
        }

        void timer_callback()
        {   
            // ROS_INFO_STREAM(state);
            // Waiting for further instruction
            if (state == WAITING) 
            {
                ROS_INFO_ONCE("State -Waiting State");
            }

            // listensrv service is called
            // Publish message to listen topic 
            // In audio node, audio processing happens - triggered by listen_pub msg
            else if (state == LISTENING)
            {
                ROS_INFO_ONCE("State -Listen state");

                std_msgs::String msg;
                msg.data = "Ready to listen!";
                listen_pub.publish(msg);

                ROS_INFO_ONCE("State -Published message on 'listening' topic, waiting for audio_node. ");
            }

            // State changes - triggered by tempo_sub
            // Publish message to tempo_pub
            // In action_picker node, randomly choose different pages
            else if (state == ACTION_PICKER) 
            {
                ROS_INFO_ONCE("State -Action_Picker State");

                std_msgs::String msg;
                msg.data = "Ready to pick actions";
                action_pick_pub.publish(msg);

                ROS_INFO_ONCE("State -Published message on 'send_tempo' topic, waiting for action_picker node. ");  

                std_msgs::Int64 tempo_msg;
                msg.data = tempo;
                action_picking_pub.publish(tempo_msg);
                ROS_INFO_ONCE("State -Published message on 'send_num_tempo' topic, waiting for action picker node");
                
            }

            // State changes - triggered by action_sub
            // Publish message to action_pub
            // In action_editing node, edit the actions to match the song.
            else if (state == ACTION_EDITOR) 
            {
                ROS_INFO_ONCE("State -Action_Editing State");     

                // std_msgs::String msg;
                // msg.data = "Ready to edit actions";
                // action_edit_pub.publish(msg);

                state::RandomList list_msg;
                list_msg.nums = list;
                action_edit_pub.publish(list_msg);
                ROS_INFO_ONCE("State -Published message on 'send_list' topic, waiting for action_editor node. ");             
            }

            // State changes - triggered by play_sub
            // Publish message to play_pub
            // In action_player node, play the actions
            else if (state == ACTION_PLAYER) 
            {
                ROS_INFO_ONCE("State -Action_Player State");

                // std_msgs::String msg;
                // msg.data = "Ready to play actions";
                // action_play_pub.publish(msg);

                // ROS_INFO_ONCE("State - Published message on 'send_ready' topic, waiting for action_player node. ");  

                std_msgs::String msg;
                msg.data = "start";
                button_pub.publish(msg);
                ROS_INFO_ONCE("State - mimiced button.")
                
            }

            // state changes - triggered by done_sub
            // Publish message to wait_pub
            // Reset all flags 
            // Return to waiting state 
            else if (state == RESET) 
            {
                ROS_INFO_ONCE("State - Reset State");
                std_msgs::String msg;
                msg.data = "reset!";
                action_play_pub.publish(msg);

                first_message_tempo = false;
                ROS_INFO_STREAM(first_message_tempo);
                first_message_actionlist = false;
                first_message_actionedit = false;
                first_message_actionplay = false;

                ROS_INFO_ONCE("State - Published message on 'reset' topic, reset all flags, switching to waiting state");  
                state = WAITING;
                // ROS_INFO_STREAM(state);
            }
        }
};



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "boogiebot_node");
    BoogieBot boogieBot;
    boogieBot.run();
    return 0;
}
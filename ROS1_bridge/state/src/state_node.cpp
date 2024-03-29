/// \file state_node.cpp
/// \brief Node for controlling the state of the BoogieBot
/// PUBLISHERS:
///     + listen_pub: 'listening' topic - (std_msgs::msg::String), causes switch in the audio node.
///     + action_picking_pub: 'send_num_tempo' topic - (std_msgs::msg::Int64), causes switch in
///                     action_picking node.
///     + action_edit_pub: 'Send_list' topic - (state::msg::RandomList), causes switch in 
///                     action_edit node.
///     + action_play_pub: 'STARTING' topic - (std_msgs::msg::String), causes switch in action_play
///                     node.
///     + button_pub: '/robotis/open_cr/button' topic - (std_msgs::msg::String), mimics button 
///                     press.
///     + reset_pub: 'reset' topic - (std_msgs::msg::String), resets all variables.
/// SUBCRIBERS:
///     + num_tempo_sub: 'state_num_tempo' topic - (std_msgs::msg::Int64): Tempo of current song, 
///                     switch to action_pick state.
///     + action_list_sub: 'list_send' topic - (state::msg::RandomList): List of random actions, 
///                     switch to action_edit state.
///     + action_edit_sub: 'get_ready' topic - (std_msgs::msg::String): YAML action file created, 
///                     switch to action_play state.
///     + action_play_sub: 'done' topic - (std_msgs::msg::String): Actions have been played.
///                     switch to waiting state.
/// SERVICES:
///     + listen_srv: 'listen_trigger' - (std_srvs::Empty::Request): Trigger for listening state.


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_srvs/Empty.h"
#include "state/RandomList.h"

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
            listen_pub = nh.advertise<std_msgs::String>("listening", 10);
            action_picking_pub = nh.advertise<std_msgs::Int64>("send_num_tempo", 10);
            action_edit_pub = nh.advertise<state::RandomList>("Send_List", 10);
            action_play_pub = nh.advertise<std_msgs::String>("STARTING", 10);
            button_pub = nh.advertise<std_msgs::String>("/robotis/open_cr/button", 10);
            reset_pub = nh.advertise<std_msgs::String>("reset", 10);

            // Subscribers
            num_tempo_sub = nh.subscribe("state_num_tempo", 10, &BoogieBot::num_tempo_callback, this);
            action_list_sub = nh.subscribe("list_send", 10, &BoogieBot::actionlist_callback, this);
            action_edit_sub = nh.subscribe("get_ready", 10, &BoogieBot::actionedit_callback, this);
            // Action play subscriber - when msg recieved, switch to reset state
            action_play_sub = nh.subscribe("done", 10, &BoogieBot::actionplay_callback, this);


            // Services
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
            ROS_INFO("STATE - Recieved 'listen_trigger' request.");
            state = LISTENING;
            return true;
        }

        void num_tempo_callback(const std_msgs::Int64::ConstPtr& msg)
        {   ROS_INFO_STREAM(first_message_tempo);
            ROS_INFO("STATE - Inside num_tempo_callback");
            if (!first_message_tempo)
            {
                ROS_INFO("STATE - Recieved meessage on 'state_num_tempo' topic, switch to action_list state.");
                tempo = msg->data;
                first_message_tempo = true;
                state = ACTION_PICKER;
            }
        }
 
        void actionlist_callback(const state::RandomList::ConstPtr& msg)
        {
            if (!first_message_actionlist)
            {
                ROS_INFO("STATE - Recieved message on 'list_send; topic, switch to action_edit state. ");
                list = msg->nums;
                first_message_actionlist = true;
                state = ACTION_EDITOR;
            }
        }

        void actionedit_callback(const std_msgs::String::ConstPtr& msg)
        {
            if (!first_message_actionedit)
            {
                ROS_INFO("STATE - Recieved message on 'get_ready' topic, switch to action_play state. ");
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
            // Waiting for further instruction
            if (state == WAITING) 
            {
                ROS_INFO_ONCE("STATE - Waiting State");
            }

            // listensrv service is called
            // Publish message to listen topic 
            // In audio node, audio processing happens - triggered by listen_pub msg
            else if (state == LISTENING)
            {
                ROS_INFO_ONCE("STATE - Listen state");

                std_msgs::String msg;
                msg.data = "Ready to listen!";
                listen_pub.publish(msg);

                ROS_INFO_ONCE("STATE - Published message on 'listening' topic, waiting for audio_node. ");
            }

            // State changes - triggered by tempo_sub
            // Publish message to tempo_pub
            // In action_picker node, randomly choose different pages
            else if (state == ACTION_PICKER) 
            {
                ROS_INFO_ONCE("STATE - Action_Picker State");
                std_msgs::Int64 tempo_msg;
                tempo_msg.data = tempo;
                action_picking_pub.publish(tempo_msg);
                ROS_INFO_ONCE("STATE - Published message on 'send_num_tempo' topic, waiting for action picker node");
                
            }

            // State changes - triggered by action_sub
            // Publish message to action_pub
            // In action_editing node, edit the actions to match the song.
            else if (state == ACTION_EDITOR) 
            {
                ROS_INFO_ONCE("STATE - Action_Editing State");     
                state::RandomList list_msg;
                list_msg.nums = list;
                action_edit_pub.publish(list_msg);
                ROS_INFO_ONCE("STATE - Published message on 'Send_List' topic, waiting for action_editor node. ");             
            }

            // State changes - triggered by play_sub
            // Publish message to play_pub
            // In action_player node, play the actions
            else if (state == ACTION_PLAYER) 
            {
                ROS_INFO_ONCE("STATE - Action_Player State");
                std_msgs::String msg;
                msg.data = "start";
                button_pub.publish(msg);
                // ROS_INFO("STATE - mimiced button - published message on /robotis/open_cr/button_topic, waiting for action_player node.");
                
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
                first_message_actionlist = false;
                first_message_actionedit = false;
                first_message_actionplay = false;

                ROS_INFO_ONCE("State - Published message on 'reset' topic, reset all flags, switching to waiting state");  
                state = WAITING;
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
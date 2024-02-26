#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "state/RandomList.h"

#include <iostream>
#include <vector>
#include <random>
#include <map>

// Create States
enum State {
    WAITING,
    PICKING,
    DONE,
    RESET
};

class ActionPicking
{
    public:
        ActionPicking() :nh(), state(WAITING)
        {
            // Publishers 
            // action_list_pub - publishes list of actions
            action_list_pub = nh.advertise<state::RandomList>("list_send", 10);

            // Subscribers
            // Subscribe to state_machine node to get the tempo
            num_tempo_sub = nh.subscribe("state_num_tempo", 10, &ActionPicking::num_tempo_callback, this);
            // subscribe to reset from state_machine
            reset_sub = nh.subscribe("reset", 10, &ActionPicking::resetsub_callback, this);
        
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
        ros::Publisher action_list_pub;
        ros::Subscriber state_machine_sub;
        ros::Subscriber num_tempo_sub;
        ros::Subscriber reset_sub;

        State state;
        bool first_message;
        bool reset_message;
        double time;
        int tempo;

        std::map<int, int> danceMap = 
        {
            {144, 1},
            {145, 1},
            {146, 1},
            {147, 1},
            {148, 3},
            {149, 3},
            {150, 1},
            {151, 1},
            {152, 1},
            {153, 1},
            {154, 5},
            {155, 5},
            {156, 3},
            {157, 3},
            {158, 4},  
        };

        void num_tempo_callback(const std_msgs::Int64::ConstPtr& msg)
        {
            if (!first_message)
            {
                ROS_INFO("Action Picking - Recieved message on 'state_num_tempo' callback, switch to picking state.");
                tempo = msg->data;
                first_message = true;
                state = PICKING;
            }
        }

        void resetsub_callback(const std_msgs::String::ConstPtr& msg)
        {
            if (!reset_message)
            {
                ROS_INFO("Action Picking - Recieved message on 'reset' topic, switch to reset state. ");
                state = RESET;
                reset_message = true;
            }
        }

        // Each even action page takes 2 seconds and each odd action page takes 1 second
        // size is the total number of seconds to be accounted for
        // 60 bpm = 1 bps -> 10 seconds = 10 actions ( 8 accounting for initial and final)
        std::vector<int64_t> genList(int duration)
        {
            // Initialize generator and variables
            std::vector<int64_t> routine;
            int elp_time = 0;

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dist(0, danceMap.size() -1);

            // Add 144 as fist element in routine
            routine.push_back(144);
            elp_time += danceMap[144];

            while (elp_time < duration)
            {
                int random = dist(gen);
                auto it = danceMap.begin();
                advance(it, random);
                int danceNum = it->first;
                int dur = it->second;

                if (danceNum != 144 && danceNum !=145 && elp_time + dur <= duration)
                {
                    routine.push_back(danceNum);
                    elp_time += dur;
                }
            }

            routine.push_back(145);
            elp_time += danceMap[145];

            return routine;
        }

        void timer_callback()
        {
            if (state == WAITING)
            {
                ROS_INFO_ONCE("Action Picking - Waiting State");
            }
            
            // message on statemachine_callback is recieved
            // call picker function
            // publish message to 'get_list' topic.
            else if (state == PICKING)
            {
                ROS_INFO_ONCE("Action Picking - Picking State");
                
                // calculate number of actions
                // int size = 60 / 60 * 10;
                // int min = 146;
                // int max = 149;
                int dur = 30;

                std::vector<int64_t> list = genList(dur);

                // Send message
                state::RandomList msg1;
                msg1.nums = list;
                action_list_pub.publish(msg1);
                ROS_INFO_ONCE("Action picking - Published list!");

                ROS_INFO_ONCE("Action Picking - Contents of message");
                for (const auto& num : msg1.nums)
                {
                    ROS_INFO("%ld", num);
                }


                state = RESET;
            }

            // Done with action, waiting for message on 'reset' topic
            else if (state == DONE)
            {
                ROS_INFO_ONCE("Action Picking - Done State");
            }

            // Reset all flags
            else if (state == RESET)
            {
                ROS_INFO_ONCE("Action Picking - Reset State");

                first_message = false;
                reset_message = false;
                
                ROS_INFO_ONCE("Action Picking - Reset all flags, switching to waiting state");  
                state = WAITING;
            }
        }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "action_picking_node");
    ActionPicking actionPicking;
    actionPicking.run();
    return 0;
}


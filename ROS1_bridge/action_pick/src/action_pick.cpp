/// \file action_pick.cpp
/// \brief Node for randomly picking dance moves to create a dance routine for ROBOTIS OP3
/// PUBLISHERS:
///     + action_list_pub: 'list_send' topic - (messages::msg::RandomList): List of int64 (nums)
/// SUBCRIBERS:
///     + num_tempo_sub: 'state_num_tempo' topic - (std_msgs::msg::Int64): Tempo  of current song

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"
#include "messages/msg/random_list.hpp"

#include <iostream>
#include <vector>
#include <random>
#include <map>
#include <chrono>

// Create States
enum State
{
    WAITING,
    PICKING,
    DONE
};

class ActionPick : public rclcpp::Node
{
    public:
        ActionPick()
        : Node("action_pick"), state(State::WAITING)
        {
            // Publishers
            action_list_pub = create_publisher<messages::msg::RandomList>("list_send", 10);

            // Subscribers
            num_tempo_sub = create_subscription<std_msgs::msg::Int64>(
                "send_num_tempo", 10, std::bind(&ActionPick::num_tempo_callback, this, std::placeholders::_1));

            // timer callback
            std::chrono::milliseconds period(1000 / 200);
            timer = create_wall_timer(period, std::bind(&ActionPick::timer_callback, this));
        }
    private:

        // Functions that do things
        // Random List Generator
        std::vector<int64_t> genList(int duration)
        {
            std::vector<int64_t> routine;
            int elp_time = 0;

            // Random number generator
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dist(0, danceMap.size() - 1);

            // ########## Begin Citation 1 ##########
            // First element in 'routine' 144 - initial pose
            routine.push_back(144);
            elp_time += danceMap[144];

            // Add actions to routine based on how much time is left in routine
            while (elp_time < duration)
            {
                int random = dist(gen);
                auto it = danceMap.begin();
                std::advance(it, random);
                int danceNum = it->first;
                int dur = it->second;

                // If random number is not 144 or 145 (inital / final pose) and time to complete
                // is less than total duration
                if (danceNum != 144 && danceNum != 145 && elp_time + dur <= duration)
                {
                    routine.push_back(danceNum);
                    elp_time += dur;
                }
            }

            // Last element in routine is 145 (final pose)
            routine.push_back(145);
            elp_time += danceMap[145];
            return routine;
        }
        // Create map of actions and their tine to execute
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

        // ########## End Citation 1 ##########

        void num_tempo_callback(const std_msgs::msg::Int64 & msg)
        {
            if (!first_message)
            {
                RCLCPP_INFO_ONCE(get_logger(), "Action Picking - Recieved message on 'state_num_tempo' callback. Switch to PICKING state.");
                tempo = msg.data;
                first_message  = true;
                state = PICKING;
            }
        }

        void timer_callback()
        {
            if (state == WAITING)
            {
                RCLCPP_INFO_ONCE(get_logger(), "Action Picking - WAITING.");
            }
            else if (state == PICKING)
            {
                RCLCPP_INFO_ONCE(get_logger(), "Action Picking - PICKING");
                int dur = 30;
                std::vector<int64_t> list = genList(dur);

                messages::msg::RandomList msg;
                msg.nums = list;
                action_list_pub->publish(msg);
                RCLCPP_INFO_ONCE(get_logger(), "Action Picking - Published random list message.");

                state = DONE;
            }
            else if (state == DONE)
            {
                RCLCPP_INFO_ONCE(get_logger(), "Action Picking - DONE.");
            }
        }
    
    // Variable Declarations
    rclcpp::Publisher<messages::msg::RandomList>::SharedPtr action_list_pub;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr num_tempo_sub;
    rclcpp::TimerBase::SharedPtr timer;

    State state;
    bool first_message;
    int tempo;        
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionPick>());
  rclcpp::shutdown();
  return 0;
}
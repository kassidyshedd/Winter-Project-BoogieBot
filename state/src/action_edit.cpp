#include "ros/ros.h"
#include "std_msgs/String.h"
#include "state/RandomList.h"
#include "op3_action_editor/action_editor.h"
#include "op3_action_module/action_module.h"

#include <vector>
#include <fstream>

// Create States
enum State {
    WAITING,
    EDITING,
    DONE,
    RESET
};

class ActionEditing
{
    public:
        ActionEditing() :nh(), state(WAITING)
        {
            // Publishers 
            // action_list_pub - publishes list of actions
            action_edit_pub = nh.advertise<std_msgs::String>("get_ready", 10);

            // Subscribers
            // Subscribe to state_machine node tp get the tempo
            action_list_sub = nh.subscribe("Send_List", 10, &ActionEditing::action_list_callback, this);
            // subscribe to reset from state_machine
            reset_sub = nh.subscribe("reset", 10, &ActionEditing::resetsub_callback, this);
        
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
        ros::Publisher action_edit_pub;
        ros::Subscriber action_list_sub;
        ros::Subscriber reset_sub;

        State state;
        bool first_message;
        bool reset_message;

        std::vector<int64_t> action_pages;

        // Load action file
        std::vector<robotis_op::action_file_define::Page> load_action_file(const std::string& filename)
        {
            // Open current .bin file
            std::ifstream file(filename, std::ios::binary);
            if (!file.is_open())
            {
                return {};
            }

            std::vector<robotis_op::action_file_define::Page> pages;
            robotis_op::action_file_define::Page page;

            while (file.read(reinterpret_cast<char*>(&page), sizeof(page)))
            {
                pages.push_back(page);
            }

            return pages;
        }


        void action_list_callback(const state::RandomList::ConstPtr& msg)
        {
            if (!first_message)
            {
                ROS_INFO("Action Editing - Recieved message on 'send_list' topic, switch to editing state. ");
                
                // Store the list of pages
                action_pages = msg->nums;
                ROS_INFO("Stored Pages");

                std::vector<robotis_op::action_file_define::Page> book = load_action_file("data/motion_4095.bin")

                first_message = true;
                state = EDITING;
                
            }
        }

        void resetsub_callback(const std_msgs::String::ConstPtr& msg)
        {
            if (!reset_message)
            {
                ROS_INFO("Action Editing - Recieved message on 'reset' topic, switch to reset state. ");
                state = RESET;
                reset_message = true;
            }
        }

        // ActionLoader function, which reads in the .bin file storing all of the actions

        void timer_callback()
        {
            if (state == WAITING)
            {
                ROS_INFO_ONCE("Action Editing - Waiting State");
            }
            
            // message on statemachine_callback is recieved
            // publish message to 'get_list' topic.
            else if (state == EDITING)
            {
                ROS_INFO_ONCE("Action Editing - Editing State");
                
                std_msgs::String msg;
                msg.data = "Edited Pages!";
                action_edit_pub.publish(msg);
                ROS_INFO_ONCE("Action Editing - Published message on 'get_ready' topic, switching to done state. ");
                state = DONE;
            }

            else if (state == DONE)
            {
                ROS_INFO_ONCE("Action Editing - Done State");
            }

            // Reset all flags
            else if (state == RESET)
            {
                ROS_INFO_ONCE("Action Editing - Reset State");

                first_message = false;
                reset_message = false;
                
                ROS_INFO_ONCE("Reset all flags, switching to waiting state");  
                state = WAITING;
            }
        }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "action_editing_node");
    ActionEditing actionEditing;
    actionEditing.run();
    return 0;
}


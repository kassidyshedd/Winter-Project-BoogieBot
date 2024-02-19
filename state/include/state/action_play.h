#ifndef ACTION_PLAY_H_
#define ACTION_PLAY_H_

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "boost/thread.hpp"
#include "yaml-cpp/yaml.h"

#include "state/dance_demo.h"
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/SetModule.h"
#include "op3_action_module_msgs/IsRunning.h"


namespace robotis_op
{
    class ActionPlay
    {
        public:
            ActionPlay();
            ~ActionPlay();

            void setDemoEnable();
            void setDemoDisable();

        protected:
            enum ActionCommandIndex
            {
                BrakeActionCommand = -2,
                StopActionCommand = -1,
            };

            enum ActionStatus
            {
                PlayAction = 1,
                PauseAction = 2,
                StopAction = 3,
                ReadyAction = 4,
            };

            const int SPIN_RATE;

            void processThread();
            void callbackThread();

            void process();
            void startProcess(const std::string &set_name = "default");
            void resumeProcess();
            void pauseProcess();
            void stopProcess();

            void handleStatus();

            void parseActionScript(const std::string &path);
            bool parseActionScriptSetName(const std::string &path, const std::string &set_name);

            bool playActionWithSound(int motion_index);

            void playMP3(std::string &path);
            void stopMP3();

            void playAction(int motion_index);
            void stopAction();
            void brakeAction();
            bool isActionRunning();

            void setModuleToDemo(const std::string &module_name);
            void callServiceSettingModule(const std::string &module_name);
            void actionSetNameCallback(const std_msgs::String::ConstPtr& msg);
            void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
            void demoCommandCallback(const std_msgs::String::ConstPtr &msg);

            ros::Publisher module_control_pub_;
            ros::Publisher motion_index_pub_;
            ros::Publisher play_sound_pub_;

            ros::Subscriber buttuon_sub_;
            ros::Subscriber demo_command_sub_;

            ros::ServiceClient is_running_client_;
            ros::ServiceClient set_joint_module_client_;

            std::map<int, std::string> action_sound_table_;
            std::vector<int> play_list_;

            std::string script_path_;
            std::string play_list_name_;
            int play_index_;

            int play_status_;
    };
}

#endif
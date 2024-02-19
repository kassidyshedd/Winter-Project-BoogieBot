#include "ros/ros.h"
#include "std_msgs/String.h"

#include"state/action_play.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

enum Status
{
  Ready = 0,
  Dance = 1,
};

void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
void goInitPose();
void playSound(const std::string &path);
void setLED(int led);
bool checkManagerRunning(std::string& manager_name);
void dxlTorqueChecker();

void demoModeCommandCallback(const std_msgs::String::ConstPtr &msg);

const int SPIN_RATE = 30;

ros::Publisher init_pose_pub;
ros::Publisher play_sound_pub;
ros::Publisher led_pub;
ros::Publisher dxl_torque_pub;

std::string default_mp3_path = "";
int current_status = Ready;
int desired_status = Ready;
bool apply_desired = false;

//node mainconst bool DEBUG_PRINT = false;
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "dance_node");

  //create ros wrapper object
  robotis_op::Dance *current_demo = NULL;
  robotis_op::ActionPlay *action_play = new robotis_op::ActionPlay();

  ros::NodeHandle nh(ros::this_node::getName());

  init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  play_sound_pub = nh.advertise<std_msgs::String>("/play_sound_file", 0);
  led_pub = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  ros::Subscriber buttuon_sub = nh.subscribe("/robotis/open_cr/button", 1, buttonHandlerCallback);
  ros::Subscriber mode_command_sub = nh.subscribe("/robotis/mode_command", 1, demoModeCommandCallback);

  default_mp3_path = ros::package::getPath("op3_demo") + "/data/mp3/";

  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  // wait for starting of manager
  std::string manager_name = "/op3_manager";
  while (ros::ok())
  {
    ros::Duration(1.0).sleep();

    if (checkManagerRunning(manager_name) == true)
    {
      break;
    }
    ROS_WARN("Waiting for op3 manager");
  }

  // init procedure
  playSound(default_mp3_path + "Demonstration ready mode.mp3");

  //node loop
  while (ros::ok())
  {
    // process
    if (apply_desired == true)
    {
      switch (desired_status)
      {
        case Ready:
        {

          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = NULL;

          goInitPose();

          break;
        }

        case Dance:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = action_play;
          current_demo->setDemoEnable();

          break;
        }

        default:
        {
          break;
        }
      }

      apply_desired = false;
      current_status = desired_status;
    }

    //execute pending callbacks
    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}

void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if(apply_desired == true)
    return;

  // in the middle of playing demo
  if (current_status != Ready)
  {
    if (msg->data == "mode_long")
    {
      // go to mode selection status
      desired_status = Ready;
      apply_desired = true;

      playSound(default_mp3_path + "Demonstration ready mode.mp3");
    }
    else if (msg->data == "user_long")
    {
      // it's using in op3_manager
      // torque on and going to init pose
    }
  }
  // ready to start demo
  else
  {
    if (msg->data == "start")
    {
      // select current demo
      desired_status = (desired_status == Ready) ? desired_status + 1 : desired_status;
      apply_desired = true;

      // sound out desired status
      switch (desired_status)
      {
        case Dance:
          dxlTorqueChecker();
          playSound(default_mp3_path + "Start soccer demonstration.mp3");
          break;

        default:
          break;
      }
    }
  }
}

void goInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub.publish(init_msg);
}

void playSound(const std::string &path)
{
  std_msgs::String sound_msg;
  sound_msg.data = path;

  play_sound_pub.publish(sound_msg);
}

void setLED(int led)
{
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led);

  led_pub.publish(syncwrite_msg);
}

bool checkManagerRunning(std::string& manager_name)
{
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
  {
    if (node_list[node_list_idx] == manager_name)
      return true;
  }

  ROS_ERROR("Can't find op3_manager");
  return false;
}

void dxlTorqueChecker()
{
  std_msgs::String check_msg;
  check_msg.data = "check";

  dxl_torque_pub.publish(check_msg);
}

void demoModeCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  // In demo mode
  if (current_status != Ready)
  {
    if (msg->data == "ready")
    {
      // go to mode selection status
      desired_status = Ready;
      apply_desired = true;

      playSound(default_mp3_path + "Demonstration ready mode.mp3");
    }
  }
  // In ready mode
  else
  {
    if(msg->data == "dance")
    {
      desired_status = action_play;
      apply_desired = true;

      // play sound
      dxlTorqueChecker();
      playSound(default_mp3_path + "Start soccer demonstration.mp3");
    }
  }
}





// // Create states
// enum State{
//     WAITING,
//     PLAYING,
//     DONE
// };

// enum staus{
//     Ready = 0,
//     DANCE = 1
// };


// void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
// void goInitPose();
// void playSound(const std::string &path);
// void setLED(int led);
// bool checkManagerRunning(std::string& manager_name);
// void dxlTorqueChecker();

// void demoModeCommandCallback(const std_msgs::String::ConstPtr &msg);

// const int SPIN_RATE = 30;

// ros::Publisher init_pose_pub;
// ros::Publisher play_sound_pub;
// ros::Publisher led_pub;
// ros::Publisher dxl_torque_pub;

// std::string default_mp3_path = "";
// int current_status = Ready;
// int desired_status = Ready;
// bool apply_desired = false;


// class ActionPlay
// {
//     public:
//         ActionPlay() : nh(), state(WAITING)
//         {   
//             // Publishers
//             // Done action publisher
//             done_action_pub = nh.advertise<std_msgs::String>("done", 10);

//             // Subscribers
//             // Start action subscriber
//             start_action_sub = nh.subscribe("startplay", 10, &ActionPlay::start_callback, this);
//         }
        

//         void run()
//         {
//             ros::Rate loop_rate(100);
//             while (ros::ok())
//             {
//                 timer_callback();
//                 ros::spinOnce();
//                 loop_rate.sleep();
//             }
//         }
//     private:
//         ros::NodeHandle nh;

//         ros::Publisher done_action_pub;

//         ros::Subscriber start_action_sub;

//         State state;
//         bool first_message;

//         void start_callback(const std_msgs::String::ConstPtr& msg)
//         {
//             if (!first_message)
//             {
//                 ROS_INFO("PLAYER - Received message on 'start' topic, switching to PLAYING state.");
//                 first_message = true;
//                 state = PLAYING;
//             }
//         }

//         void timer_callback()
//         {
//             if (state == WAITING)
//             {
//                 ROS_INFO_ONCE("PLAYER - Waiting State.");
//             }
//             else if (state == PLAYING)
//             {
//                 ROS_INFO_ONCE("PLAYER - Playing State.");
//                 state = DONE;
//             }
//             else if (state == DONE)
//             {
//                 ROS_INFO_ONCE("PLAYER - Done State");

//                 std_msgs::String msg;
//                 msg.data = "Done!";
//                 done_action_pub.publish(msg);
//                 ROS_INFO_ONCE("PLAYER - Published message on 'done' topic, switching to waiting state.");
//                 state = WAITING;
//             }
//         }
// };

// int main(int argc, char* argv[])
// {
//     ros::init(argc, argv, "actionplay_node");
//     ActionPlay actionPlay;
//     actionPlay.run();
//     return 0;
// }

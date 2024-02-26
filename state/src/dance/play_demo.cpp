#include "state/action_play.h"

namespace robotis_op
{

ActionPlay::ActionPlay()
    : SPIN_RATE(30),
      DEBUG_PRINT(true),
      play_index_(0),
      play_status_(StopAction)
{
  enable_ = false;

  ros::NodeHandle nh(ros::this_node::getName());

  std::string default_path = ros::package::getPath("state") + "/list/action_list.yaml";
  script_path_ = nh.param<std::string>("action_script", default_path);

  std::string default_play_list = "default";
  play_list_name_ = nh.param<std::string>("action_script_play_list", default_play_list);

  demo_command_sub_ = nh.subscribe("/robotis/demo_command", 1, &ActionPlay::demoCommandCallback, this);
  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);

  parseActionScript(script_path_);

  boost::thread queue_thread = boost::thread(boost::bind(&ActionPlay::callbackThread, this));
  boost::thread process_thread = boost::thread(boost::bind(&ActionPlay::processThread, this));
}

ActionPlay::~ActionPlay()
{
}

void ActionPlay::setDemoEnable()
{
  setModuleToDemo("action_module");

  enable_ = true;

  ROS_INFO_COND(DEBUG_PRINT, "Start ActionScript Demo");

  ROS_INFO_ONCE("playing init pose");
//   playAction(InitPose);

  ROS_INFO_ONCE("Move to start process.");
  startProcess(play_list_name_);
}

void ActionPlay::setDemoDisable()
{
  stopProcess();

  enable_ = false;
  ROS_WARN("Set Action demo disable");
  play_list_.resize(0);
}

void ActionPlay::process()
{ 
  ROS_INFO_ONCE("In process");
  switch (play_status_)
  {
    case PlayAction:
    {
      ROS_INFO_ONCE("case - play action");  
      if (play_list_.size() == 0)
      {
        ROS_WARN("Play List is empty.");
        return;
      }

      // action is not running
      if (isActionRunning() == false)
      {      
        // ROS_INFO("Is running = false");
        // play
        bool result_play = playActionWithSound(play_list_.at(play_index_));
        ROS_INFO_STREAM("result play" << result_play);

        ROS_INFO_COND(!result_play, "Fail to play action script.");

        // add play index
        // ROS_INFO_ONCE("add play index");
        int index_to_play = (play_index_ + 1) % play_list_.size();
        play_index_ = index_to_play;
        
        // ROS_INFO_ONCE("update play index");
        return;
      }
      else
      {
        // wait
        ROS_INFO_ONCE("waiting");
        return;
      }
      break;
    }

    case PauseAction:
    {
      ROS_INFO_ONCE("pause");
      stopMP3();
      brakeAction();

      play_status_ = ReadyAction;

      break;
    }

    case StopAction:
    {
      ROS_INFO_ONCE("stop");
      stopMP3();
      stopAction();

      play_status_ = ReadyAction;

      break;
    }

    default:
      break;
  }
}

void ActionPlay::startProcess(const std::string &set_name)
{
  ROS_INFO_ONCE("start process");
  ROS_INFO_STREAM("path %s" << script_path_);
  ROS_INFO_STREAM("path %s" << set_name);

  parseActionScriptSetName(script_path_, set_name);

  
  play_status_ = PlayAction;
  ROS_INFO_STREAM("play status" << play_status_);
}

void ActionPlay::resumeProcess()
{
  play_status_ = PlayAction;
}

void ActionPlay::pauseProcess()
{
  ROS_INFO("pauseprocess()");
  play_status_ = PauseAction;
}

void ActionPlay::stopProcess()
{
  play_index_ = 0;
  play_status_ = StopAction;
}

void ActionPlay::processThread()
{
  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  //node loop
  while (ros::ok())
  {
    if (enable_ == true)
      process();

    //relax to fit output rate
    loop_rate.sleep();
  }
}

void ActionPlay::callbackThread()
{
  ROS_INFO_ONCE("in callback");
  ros::NodeHandle nh(ros::this_node::getName());
  ROS_INFO_ONCE("node created");

  // subscriber & publisher
  module_control_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  ROS_INFO_ONCE("pub create");
  std_msgs::Int32 motion_msg;
  motion_msg.data = 0;
  ROS_INFO_ONCE("about to pub");
  motion_index_pub_.publish(motion_msg);
  ROS_INFO_ONCE("Published initial motion");

  play_sound_pub_ = nh.advertise<std_msgs::String>("/play_sound_file", 0);

//   buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &ActionPlay::buttonHandlerCallback, this);

  is_running_client_ = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");
  set_joint_module_client_ = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

  while (nh.ok())
  {
    ros::spinOnce();

    usleep(1000);
  }
}

void ActionPlay::parseActionScript(const std::string &path)
{
  YAML::Node doc;

  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Fail to load action script yaml. - " << e.what());
    ROS_ERROR_STREAM("Script Path : " << path);
    return;
  }

  // parse action_sound table
  YAML::Node sub_node = doc["action_and_sound"];
  for (YAML::iterator yaml_it = sub_node.begin(); yaml_it != sub_node.end(); ++yaml_it)
  {
    int action_index = yaml_it->first.as<int>();
    std::string mp3_path = yaml_it->second.as<std::string>();

    action_sound_table_[action_index] = mp3_path;
  }

  // default action set
  if (doc["default"])
    play_list_ = doc["default"].as<std::vector<int> >();
}

bool ActionPlay::parseActionScriptSetName(const std::string &path, const std::string &set_name)
{
  ROS_INFO_ONCE("load yaml");
  YAML::Node doc;

  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml.");
    return false;
  }
  ROS_INFO_ONCE("yaml loaded");

  // parse action_sound table
  if (doc[set_name])
  {
    ROS_INFO_ONCE("inside doc[set_name]");
    play_list_ = doc[set_name].as<std::vector<int> >();
    ROS_INFO_STREAM("playlist:");
    for (const auto& item : play_list_) {
    ROS_INFO_STREAM(item);
}
    return true;
  }
  else
    ROS_INFO_ONCE("false");
    return false;
}

bool ActionPlay::playActionWithSound(int motion_index)
{
  // ROS_INFO("Inside play action with sounds");
  std::map<int, std::string>::iterator map_it = action_sound_table_.find(motion_index);
  ROS_INFO("Map is created");

  bool dum = map_it == action_sound_table_.end();
  ROS_INFO_STREAM("dummy" << dum);

  
  if (map_it == action_sound_table_.end())
  {
    ROS_INFO("returning false");
    return false;
  }

  ROS_INFO_STREAM("motion index" << motion_index );
  playAction(motion_index);
  ROS_INFO_STREAM("action played" << motion_index);
  playMP3(map_it->second);

  if (motion_index == 145)
  {
    return false;
  }


  ROS_INFO_STREAM_COND(DEBUG_PRINT, "action : " << motion_index << ", mp3 path : " << map_it->second);

  return true;
}

void ActionPlay::playMP3(std::string &path)
{
  ROS_INFO_ONCE("inside play mp3");  
  std_msgs::String sound_msg;
  sound_msg.data = path;

  play_sound_pub_.publish(sound_msg);
  ROS_INFO_ONCE("publish mp3");
}

void ActionPlay::stopMP3()
{
  std_msgs::String sound_msg;
  sound_msg.data = "";

  play_sound_pub_.publish(sound_msg);
}

void ActionPlay::playAction(int motion_index)
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;
  ROS_INFO_ONCE("play_demo - playAction ");
  motion_index_pub_.publish(motion_msg);
}

void ActionPlay::stopAction()
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = StopActionCommand;
  ROS_INFO_ONCE("play_demo - stopAction ");
  motion_index_pub_.publish(motion_msg);
}

void ActionPlay::brakeAction()
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = BrakeActionCommand;
  ROS_INFO_ONCE("play_demo - brakeAction ");
  motion_index_pub_.publish(motion_msg);
}

// check running of action
bool ActionPlay::isActionRunning()
{
  // ROS_INFO("inside isActionRunning "); 
  op3_action_module_msgs::IsRunning is_running_srv;

  if (is_running_client_.call(is_running_srv) == false)
  {
    ROS_INFO("Failing "); 
    ROS_ERROR("Failed to get action status");
    return true;
  }
  else
  {
    if (is_running_srv.response.is_running == true)
    {
      // ROS_INFO("running is true. "); 
      return true;
    }
  }
  // ROS_INFO("running is false "); 
  return false;
}

void ActionPlay::setModuleToDemo(const std::string &module_name)
{
  callServiceSettingModule(module_name);
  ROS_INFO_STREAM("enable module : " << module_name);
}

void ActionPlay::callServiceSettingModule(const std::string &module_name)
{
    robotis_controller_msgs::SetModule set_module_srv;
    set_module_srv.request.module_name = module_name;

    if (set_joint_module_client_.call(set_module_srv) == false)
    {
      ROS_ERROR("Failed to set module");
      return;
    }

    return ;
}

void ActionPlay::demoCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    resumeProcess();
  }
  else if (msg->data == "stop")
  {
    ROS_INFO("recieve stop message - pause");
    pauseProcess();
  }
}

} /* namespace robotis_op */
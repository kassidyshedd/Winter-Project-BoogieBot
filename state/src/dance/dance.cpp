#include "state/action_play.h"

namespace robotis_op
{
    ActionPlay::ActionPlay()
    : SPIN_RATE(30),
    play_index_(0),
    play_status(StopAction)

    {
        enable_ = false;

        ros::NodeHandle nh(ros::this_node::getName());

        std::string default_path = ros::package::getPath("state") + "/list/actions.yaml";
        script_path_ = nh.param<std::string>("action_script", default_path);

        std::string default_play_list = "default";
        play_list_name_ = nh.param<std::string>("action_script_play_list", default_play_list);

        demo_command_sub = nh.subscribe("/robotis/demo_command", 1, &ActionPlay::demoCommandCallback, this);

        parseActionScript(script_path_);

        boost::therad queue_thread = boost::thread(boost::bind(&ActionPlay::callbackThread, this));
        boost::thread process_thread = boost::thread(boost::bind(&ActionPlay::processThread, this));
    }

    ActionPlay::~ActionPlay() 
    {

    }

    void ActionPlay::setDemoEnable()
    {
        setModule("action_module");
        enable_ = true;
        
        ROS_INFO_ONCE("Start ActionScript");

        playAction(InitPose);
        startProcess(play_list_name_);
    }

    void ActionPlay::setDemoDisable()
    {
        stopProcess();

        enable_ = false;
        ROS_WARN("Set ActionPlay disable");
        play_list_.resize(0);
    }

    void ActionPlay::process()
    {
        switch (play_status_)
        {
            case PlayAction:
            {
                if (play_list_.size() == 0)
                {
                    ROS_WARN("Play list is empty.");
                    return;
                }

                // Action is not running
                if (isActionRunning() == false)
                {
                    // Play
                    bool result_play = playActionWithSound(play_list_.at(play_index_));
                    int index_to_play = (play_index_ + 1) % play_list_.size();
                    play_index = index_to_play;
                }

                else
                {
                    // wait
                    return;
                }
                break;
            }

            case PauseAction:
            {
                stopMP3();
                stopAction();
                play_status_ = ReadyAction;
                break
            }

            default:
                break;
        }
    }

    void ActionPlay::startProcess(const std::string &set_name)
    {
        partActionScriptSetName(script_path_, set_name);
        play_status_ = PlayAction;
    }

    void ActionPlay::resumeProcess()
    {
        play_status_ = PauseAction;
    }

    void ActionPlay::pauseProcess()
    {
        play_status_ = PuaseAction;
    }

    void ActionPlay::stopProcess()
    {
        play_index_ = 0;
        play_status_ = StopAction;
    }

    void ActionPlay::processThread()
    {
        // set node loop rate
        ros::Rate loop_rate(SPIN_RATE);

        // node loop
        while (ros::ok())
        {
            if (enable_ == true)
            {
                process();
                loop_rate.sleep();
            }
        }
    }

    void ActionPlay::callbackThread()
    {
        ros::NodeHandle nh(ros::this_node::getName());

        // subscribers and publishers
        module_control_pub = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
        motion_index_pub = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
        play_sound_pub = nh.advertise<std_msgs::String>("/play_sound_file,", 0);

        button_sub = nh.subscribe("/robotis/open_cr/button", 1, &ActionPlay::buttonHandlerCallback, this);

        is_running_client_ = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");
        set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

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
            doc = YAML::LoadFile(path.c_str());
        }

        catch (const std::exception& e)
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
        if (doc["defualt"])
        {
            play_list_ = doc["default"].as<std::vector<int> >();
        }
    }

    bool ActionPlay::parseActionScriptSetName(const std::string &path, const std::string &set_name)
    {
        YAML::Node doc;

        try
        {
            // load yaml
            doc = YAML::LoadFile(path.c_str());
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Fail to load yaml");
            return false;
        }

        // parse action_sound_table
        if (doc[set_name])
        {
            play_list_ = doc[set_name].as<std::vector<int> >();
            return true;
        }
        else
        {
            return false;
        }
    }

    bool ActionPlay::playActionWithSound(int motion_index)
    {
        std::map<int, std::string>::iterator map_it = action_sound_table_.find(motion_index);
        if (map_it == action_sound_table_.end())
        {
            return false;
        }
        
        playAction(motion_index);
        playMP3(map_it->seconds);
        return true;
    }

    void ActionPlay:playMP3(std::string &path)
    {
        std_msgs::String sound_msg;
        sound_msg.data = path;
        play_sound_pub_.publish(sound_msg);
    }

    void ActionPlay::stopMP3()
    {
        std_msgs::String sound_msg;
        sound_msg.dta = "";
        play_sound_pub_.publish(sound_msg);
    }

    void ActionPlay::playAction(int motion_index)
    {
        std_msgs::Int32 motion_msg;
        motion_msg.data = motion_index;
        motion_index_pub_.publish(motion_msg);
    }

    void ActionPlay::stopAction()
    {
        std_msgs::int32 motion_msg;
        motion_msg.data = StopActionCommand;
        motion_index_pub_.publish(motion_msg);
    }

    void ActionPlay::brakeAction()
    {
        std_msgs::Int32 motion_msg;
        motion_msg.data = BrakeActionCommand;
        motion_index_pub_.publish(motion_msg);
    }

    // check running of action
    bool ActionPlay::isActionRunning()
    {
        op3_action_module_msgs::IsRunning is_running_srv;

        if (is_running_client_.call(is_running_srv) == false)
        {
            ROS_ERROR("Failed to get action status");
            return true;
        }
        else
        {
            if (is_running_srv.response.is_running == true)
            {
                return true;
            }
        }
        return false;
    }

    void ActionPlay::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
    {
        if (enable_ == false)
        {
            return;
        }

        if (msg->data == "start")
        {
            switch (play_status_)
            {
                case PlayAction:
                {
                    pauseProcess();
                    break;
                }

                case PauseAction:
                {
                    resumeProcess();
                    break;
                }

                case StopAction:
                {
                    resumeProcess();
                    break;
                }

                default:
                    break;
            }
        }
        else if (msg->data == "mode"){}
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

        return;
    }

    void ActionPlay::demoCommandCallback(const std_msgs::String::ConstPtr & msg)
    {
        if (enable_ == false)
        {
            return;
        }

        if (msg->data =="start")
        {
            resumeProcess();
        }

        else if (msg->data == "stop")
        {
            pauseProcess();
        }
    }

}
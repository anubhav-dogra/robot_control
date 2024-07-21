#include "ros/ros.h"
#include "RobotLibrary/KinematicTree.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "memory"

// class for PositionControl

class PositionControl
{
public:
    PositionControl()
    {
        ros::NodeHandle nh("~");
        ros::AsyncSpinner spinner(0);
        spinner.start();
        std::string urdf_string;
        while (urdf_string.empty()) {
            // ROS_INFO("Controller is waiting for model");
            nh.getParam(robot_description, urdf_string);
            ros::Duration(0.1).sleep();
        }
        try
        {
           get_model(urdf_string);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        ROS_INFO_STREAM_NAMED("PositionController", "Received urdf from param server, parsing...");
        sub_joint_states = nh.subscribe("/iiwa/joint_states", 1, &PositionControl::joint_states_callback, this);
        pub = nh.advertise<std_msgs::Float64MultiArray>("/iiwa/command/position", 1);

        ros::waitForShutdown();
    }

    void get_model(const std::string& urdf_string)
    {
        model = std::make_unique<KinematicTree>(urdf_string);
        numjoints = model->number_of_joints();
    }
    void joint_states_callback(const sensor_msgs::JointStateConstPtr& msg)
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(numjoints);
        Eigen::VectorXd qdot = Eigen::VectorXd::Zero(numjoints);
        for(int i = 0; i < numjoints; i++)
        {
            q(i) = msg->position[i];
            qdot(i) = msg->velocity[i];
        }
        if(not model->update_state(q, qdot))
        {
            std::cerr << "[ERROR] [URDF TEST] Couldn't update the state for some reason." << std::endl;
        }
    }

private:
    std::unique_ptr<KinematicTree> model;
    ros::Subscriber sub_joint_states;
    ros::Publisher pub;
    std::string robot_description = "/robot_description";
    std::string end_effector = "iiwa_link_ee";
    size_t _ef_index;
    int numjoints;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control");
    PositionControl pc;
    return 0;

}
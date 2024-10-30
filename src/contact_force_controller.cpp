#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <robot_control/PID.h>


double desired_force = 3.0;

class ContactForceController{
    public:
        ContactForceController(ros::NodeHandle& nh)
        : tfBuffer(), tfListener(tfBuffer), // Initialize tfBuffer and tfListener as class members
         pid_controller(Kp, Ki, Kd, alpha),
         roll_pid_controller(Kp_roll_orientation, Ki_orientation, Kd_roll_orientation, alpha), 
         pitch_pid_controller(Kp_pitch_orientation, Ki_orientation, Kd_pitch_orientation, alpha)
        {
            // ros::param::get("end_effector_name", end_effector_name);
            // tf2_ros::Buffer tfBuffer;
            // tf2_ros::TransformListener tfListener(tfBuffer);

            try
            {
                transformStamped_base_to_end = tfBuffer.lookupTransform("iiwa_link_0", "tool_link_ee", ros::Time(0), ros::Duration(1.0));
            }
            catch(tf2::TransformException &e)
            {
                ROS_ERROR("%s", e.what());
            }
            std::cout << transformStamped_base_to_end << std::endl;           
            tf2::convert(transformStamped_base_to_end.transform.rotation, intial_quat);
            intial_quat.normalize();
            // previous_quat.normalize();
            previous_quat = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
            pub_ = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/CartesianImpedance_trajectory_controller/reference_pose", 1);
            sub_ = nh.subscribe("/cartesian_wrench_tool_biased", 1, &ContactForceController::callback, this);
            
        }

        void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
        {
            current_force_z  = msg->wrench.force.z;
            current_force_x = msg->wrench.force.x;
            current_force_y = msg->wrench.force.y;
            // torque_x = msg->wrench.torque.x;
            // torque_y = msg->wrench.torque.y;
            
            // error_fz = desired_force - current_force_z;
            // double roll_error =  current_force_x;
            // double pitch_error = -current_force_y;
            // double roll_error =  torque_x;
            // double pitch_error = torque_y;
            if (first_run)
            {
                prev_time = ros::Time::now();
                first_run = false;
            }
            current_time = ros::Time::now();
            double dt = (current_time - prev_time).toSec();
            prev_time = current_time;
            if (dt>0)
            {
                double dZ = pid_controller.calculateControlSignal(desired_force, abs(current_force_z), dt, alpha);
                double max_dZ = 0.0005;  // Maximum allowed movement per iteration
                if (fabs(dZ) > max_dZ) {
                    dZ = copysign(max_dZ, dZ);  // Clamp the displacement
                    // std::cout << "Clamped dZ" << dZ << std::endl;
                }
                double dRoll = roll_pid_controller.calculateControlSignal(0, -current_force_y, dt, alpha);
                double dPitch = pitch_pid_controller.calculateControlSignal(0, current_force_x, dt, alpha);
                // std::cout << dPitch << std::endl;
                double max_dRoll = 0.0005;  // Maximum allowed movement per iteration
                double max_dPitch = 0.0005;  // Maximum allowed movement per iteration
                if (fabs(dRoll) > max_dRoll) {
                    dRoll = copysign(max_dRoll, dRoll);  // Clamp the displacement
                    std::cout << "Clamped dRoll" << dRoll << std::endl;
                }
                if (fabs(dPitch) > max_dPitch) {
                    dPitch = copysign(max_dPitch, dPitch);  // Clamp the displacement
                    std::cout << "Clamped dPitch" << dPitch << std::endl;
                }
                geometry_msgs::PoseStamped pose = update_pose(dZ, dRoll, dPitch);
                // std::cout << pose << std::endl;
                pub_.publish(pose);
            }
            running_rate.sleep();

        }  

        geometry_msgs::PoseStamped update_pose(double& dZ, double& dRoll, double& dPitch)
        {
            tf2::Quaternion quat_delta;
            quat_delta.setRPY(dRoll, dPitch, 0.0);
            quat_delta.normalize();

            new_quat = quat_delta*previous_quat;
            new_quat.normalize();
            previous_quat = new_quat;

            transformStamped_goal.header.stamp = ros::Time::now();
            transformStamped_goal.header.frame_id = "tool_link_ee";
            // std::cout << "prev_z" << transformStamped_goal.transform.translation.z << std::endl;
            transformStamped_goal.transform.translation.x = 0.0;
            transformStamped_goal.transform.translation.y = 0.0;
            transformStamped_goal.transform.translation.z += dZ;
            // transformStamped_goal.transform.rotation = tf2::toMsg(new_quat);
            transformStamped_goal.transform.rotation.x = new_quat.getX();
            transformStamped_goal.transform.rotation.y = new_quat.getY();
            transformStamped_goal.transform.rotation.z = new_quat.getZ();
            transformStamped_goal.transform.rotation.w = new_quat.getW();

            geometry_msgs::TransformStamped transformed_output;

            tf2::doTransform(transformStamped_goal, transformed_output, transformStamped_base_to_end);
            geometry_msgs::PoseStamped pose_got;
            pose_got.header = transformStamped_base_to_end.header;
            pose_got.header.frame_id = "world";
            pose_got.pose.position.x = transformed_output.transform.translation.x;
            pose_got.pose.position.y = transformed_output.transform.translation.y;
            pose_got.pose.position.z = transformed_output.transform.translation.z;
            pose_got.pose.orientation.x = transformed_output.transform.rotation.x;
            pose_got.pose.orientation.y = transformed_output.transform.rotation.y;
            pose_got.pose.orientation.z = transformed_output.transform.rotation.z;
            pose_got.pose.orientation.w = transformed_output.transform.rotation.w;
            return pose_got;
            // }
            
        } 

    private:
        std::string end_effector_name;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        double current_force_z=0.0, error_fz=0.0, error_integral=0.0, error_derivative=0.0, previous_error_fz=0.0, dZ=0.0;
        double torque_x=0.0, torque_y=0.0, current_force_x=0.0, current_force_y=0.0, previous_error_roll=0.0, previous_error_pitch=0.0, error_d_roll=0.0, error_d_pitch=0.0;
        double alpha = 0.0;
        double smoothed_dFe = 0.0;
        bool first_run = true;
        ros::Rate running_rate = 100;
        ros::Time current_time, prev_time;
        double factor = 10000.0;
        double Kp = 1/factor;
        double Kd = 0.0/factor;
        double Ki = 0.01/factor;

        double Kp_roll_orientation = 1.0/factor;
        double Kd_roll_orientation = 0.0/factor;
        double Ki_orientation = 0.01/factor;
        
        double Kp_pitch_orientation = 1.0/factor;
        double Kd_pitch_orientation = 0.0/factor;

        tf2::Quaternion previous_quat;
        tf2::Quaternion new_quat;
        tf2::Quaternion intial_quat;


        geometry_msgs::TransformStamped transformStamped_base_to_end;
        geometry_msgs::TransformStamped transformStamped_goal;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        PIDController pid_controller;
        PIDController roll_pid_controller;
        PIDController pitch_pid_controller;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "contact_force_control");
    ros::NodeHandle nh;
    ContactForceController fc(nh);
    ros::spin();
    return 0;
}
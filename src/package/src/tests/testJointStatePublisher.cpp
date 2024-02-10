#include "../../include/package/jointStatePublisher.h"
#include "std_msgs/Float64MultiArray.h"
#include <cmath>

#define LOOP_FREQUENCY 1000.0


int main(int argc, char** argv){
    JointStatePublisher myPub;

    ros::init(argc, argv, "custom_joint_pub_node");
    ros::NodeHandle nh;
    myPub.setPubDesJstate(nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1)); 
    ros::Rate loop_rate(LOOP_FREQUENCY);

    // Check if gripper is actuated
    bool gripper_sim;
    nh.getParam("/gripper_sim", gripper_sim);
    myPub.setGripper(gripper_sim);

    
    double time = 0.0;
    std::array<double, 6> amp = {0.3, 0.0, 0.0, 0.0, 0.0, 0.0};  // Amplitude
    std::array<double, 6> freq = {0.2, 0.0, 0.0, 0.0, 0.0, 0.0}; // Frequency
    //std::array<double, 6> q_des;
    std::array<double, 6> q_des0 = { -0.32,-0.78, -2.56,-1.63, -1.57, 3.49};
    myPub.initFilter(q_des0);

    while (ros::ok()) {
        for (size_t i = 0; i < 6; ++i) {
            myPub.q_des[i] = q_des0[i] + amp[i] * std::sin(2 * M_PI * freq[i] * time);
        }
        myPub.send_des_jstate();
        std::cout << myPub.q_des[0] << " " << myPub.q_des[1] << " " << myPub.q_des[2] << " " << myPub.q_des[3] << " " << myPub.q_des[4] << " " << myPub.q_des[5] << std::endl;
        time = std::round((time + 1.0 / LOOP_FREQUENCY) * 1000.0) / 1000.0;
        loop_rate.sleep();
    }


    return 0;
}

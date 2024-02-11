#include "../../include/package/jointStatePublisher.h"
#include "std_msgs/Float64MultiArray.h"

// initalize all arrays with zeros
JointStatePublisher::JointStatePublisher() {
    q_des.fill(0.0); 
    f_des = 0.0;
}

// function to publish desired joint states
void JointStatePublisher::send_des_jstate() {

    std_msgs::Float64MultiArray msg ;
    if(gripper_sim) msg.data = {q_des[0], q_des[1], q_des[2], q_des[3], q_des[4], q_des[5], f_des, f_des};
    else msg.data = {q_des[0], q_des[1], q_des[2], q_des[3], q_des[4], q_des[5]};
    // publish desired joint state
    pub_des_jstate.publish(msg);
}

void JointStatePublisher::setGripper(double _gripper_sim){
    gripper_sim = _gripper_sim;
}

void JointStatePublisher::setPubDesJstate(ros::Publisher _pub_des_jstate){
    pub_des_jstate = _pub_des_jstate;
}

#include "../../include/package/jointStatePublisher.h"
#include "std_msgs/Float64MultiArray.h"

// initalize all arrays with zeros
JointStatePublisher::JointStatePublisher() {
    q_des.fill(0.0); //Riempe l'array di 6 elementi con 0.0
    qd_des.fill(0.0);
    tau_ffwd.fill(0.0);
    filter_1.fill(0.0);
    filter_2.fill(0.0);
}

// function to publish desired joint states
void JointStatePublisher::send_des_jstate() {
    std_msgs::Float64MultiArray msg ;

    msg.data = {q_des[0], q_des[1], q_des[2], q_des[3], q_des[4], q_des[5], f_des, f_des};
 
    // publish desired joint state
    pub_des_jstate.publish(msg);
}

void JointStatePublisher::initFilter(const std::array<double, 6>& q) {
    filter_1 = q;
    filter_2 = q;
}

std::array<double, 6> JointStatePublisher::secondOrderFilter(double input, double rate, double settling_time) {
    double dt = 1.0 / rate;
    double gain = dt / (0.1 * settling_time + dt);
    for (size_t i = 0; i < filter_1.size(); ++i) {
    filter_1[i] = (1 - gain) * filter_1[i] + gain * input;
}
    for (size_t i = 0; i < filter_2.size(); ++i) {
    filter_1[i] = (1 - gain) * filter_2[i] + gain * filter_1[i];
}
    return filter_2;
}

void JointStatePublisher::setGripper(double _gripper_sim){
    gripper_sim = _gripper_sim;
}

void JointStatePublisher::setPubDesJstate(ros::Publisher _pub_des_jstate){
    pub_des_jstate = _pub_des_jstate;
}

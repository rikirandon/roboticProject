#ifndef __JOINSTATEPUBLISHER__
#define __JOINSTATEPUBLISHER__

#include <ros/ros.h>
#include <array>

class JointStatePublisher{
private:
    std::array<double, 6> qd_des, tau_ffwd, filter_1, filter_2;
    ros::Publisher pub_des_jstate;
    bool gripper_sim;
    std::array<double, 6> amp = {0.3, 0.0, 0.0, 0.0, 0.0, 0.0};  // Amplitude
    std::array<double, 6> freq = {0.2, 0.0, 0.0, 0.0, 0.0, 0.0}; // Frequency

public:
    std::array<double, 6> q_des;
    double f_des;
    JointStatePublisher();
    void send_des_jstate();
    void initFilter(const std::array<double, 6>& q);
    std::array<double, 6> secondOrderFilter(double input, double rate, double settling_time);
    void setGripper(double _gripper_sim);
    void setPubDesJstate(ros::Publisher _pub_des_jstate);
};


#endif

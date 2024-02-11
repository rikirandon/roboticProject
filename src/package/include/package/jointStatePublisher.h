#ifndef __JOINSTATEPUBLISHER__
#define __JOINSTATEPUBLISHER__

#include <ros/ros.h>
#include <array>

class JointStatePublisher{
private:
    ros::Publisher pub_des_jstate;
    bool gripper_sim;

public:
    std::array<double, 6> q_des;
    double f_des;
    JointStatePublisher();
    void send_des_jstate();
    void setGripper(double _gripper_sim);
    void setPubDesJstate(ros::Publisher _pub_des_jstate);
};


#endif

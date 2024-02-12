#include "../../include/package/jointStatePublisher.h"
#include "../../include/package/ur5Object.h"
#include "package/coordinates.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include <iostream>



#define LOOP_FREQUENCY 1000.0

// if it is the first movement it does 
bool firstTime = true;
Vector6d Theta;
Vector3d lastX;
Vector3d lastPhi;


void moveGripper(double ds, double df, JointStatePublisher& myPub, UR5& ur5, ros::Rate loop_rate){
    Eigen::VectorXd finger = ur5.moveGripper(ds, df, 0, 2);
    for (int i = 0; i < finger.size(); i++){
        myPub.f_des = finger(i);
        myPub.send_des_jstate();
        loop_rate.sleep();
    }
};

void moveJoints(Vector3d& xs, Vector3d& xf, Vector3d& phis, Vector3d& phif, JointStatePublisher& myPub, UR5& ur5, ros::Rate loop_rate){
    
    ur5.setPoints(xs, xf, phis, phif);
    ur5.polinomialCofficients(0.0, 3.0);
    Eigen::MatrixXd TH = ur5.IDK_wFB(Theta, 0.0, 3.0);
    // public joints
    for(int j = 0; j < TH.cols(); j++ ){
        for (int i = 0; i < 6; i++){
            myPub.q_des[i] = TH(i,j);
        }
        myPub.send_des_jstate();
        loop_rate.sleep();
    }
    Theta = TH.rightCols(1);
};


void moveBrickTo(Vector3d& poseStart, Vector3d&  poseFinal, JointStatePublisher& myPub, ros::Rate loop_rate){
    UR5 ur5(0.001, 1.0 * Matrix3d::Identity(), 1.0 * Matrix3d::Identity());
    Vector3d x1, x2, x3, x4, phi1, phi2, phi3, phi4;
    double ds, df;
    if (firstTime){
        Theta << -0.32,-0.78, -2.56,-1.63, -1.57, 3.49;
        Matrix4d direct = ur5.Direct(Theta); 
        Matrix3d Re = direct.block(0, 0, 3, 3);
        lastPhi = ur5.rotm2eul(Re);
        lastX = direct.col(3).head(3);
        moveGripper(40, 80, myPub, ur5,  loop_rate);
        firstTime = !firstTime;
    }
    phi2 << 0,0,0;
    phi3 = phi4 = phi2;

    x1[0] = poseStart[0];
    x1[1] = poseStart[1];
    x1[2] = poseStart[2] - 0.2;

    x2 = poseStart;

    x3[0] = poseFinal[0];
    x3[1] = poseFinal[1];
    x3[2] = poseFinal[2] - 0.2;

    x4 = poseFinal;


    moveJoints(lastX, x1, lastPhi, phi1, myPub, ur5, loop_rate);
    moveJoints(x1, x2, phi1, phi2, myPub, ur5, loop_rate);
    moveGripper(80, 10, myPub, ur5, loop_rate);
    moveJoints(x2, x1, phi2, phi1, myPub, ur5, loop_rate);
    moveJoints(x1, x3, phi1, phi3, myPub, ur5, loop_rate);
    moveJoints(x3, x4, phi3, phi4, myPub, ur5, loop_rate);
    moveGripper(10, 80, myPub, ur5, loop_rate);
    moveJoints(x4, x3, phi4, phi3, myPub, ur5, loop_rate);

    lastX = x3;
    lastPhi = phi3;
};

int main(int argc, char** argv){
    // node init
    ros::init(argc, argv, "motion_planner_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_FREQUENCY);

    // publisher topic init
    JointStatePublisher myPub;
    myPub.setPubDesJstate(nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1)); 
    // Check if gripper is actuated
    bool gripper_sim;
    nh.getParam("/gripper_sim", gripper_sim);
    myPub.setGripper(gripper_sim);

    // client service init
    ros::ServiceClient client = nh.serviceClient<package::coordinates>("coordinates");
    package::coordinates srv;

    // main loop
    while (ros::ok()) {
        // service call
        if (client.call(srv)) {
            ROS_INFO("success call");
            geometry_msgs::Pose poseStart = srv.response.poseStart;
            geometry_msgs::Pose poseFinal = srv.response.poseFinal;
            std::cout << "Coordinate poseStart: x = " << poseStart.position.x << ", y = " << poseStart.position.y << ", z = " <<poseStart.position.z<< std::endl;
            std::cout << "Coordinate poseFInal: x = " << poseFinal.position.x << ", y = " << poseFinal.position.y << ", z = " <<poseFinal.position.z<< std::endl;
            Vector3d xs (poseStart.position.x, poseStart.position.y, poseStart.position.z);
            Vector3d xf (poseFinal.position.x, poseFinal.position.y, poseFinal.position.z);

            // start mooving precedure
            moveBrickTo(xs, xf, myPub, loop_rate);

        } else {
            ROS_ERROR("Error call");
            return 1;
        }

        // Controllo se la lista di blocchi è vuota
        if (!srv.response.flag) {
            ROS_INFO("block's list finished");
            break; // Esci dal ciclo while
        }

    }
      


    return 0;
}



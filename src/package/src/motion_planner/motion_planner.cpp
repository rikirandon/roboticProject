#include "../../include/package/jointStatePublisher.h"
#include "../../include/package/ur5Object.h"
#include "std_msgs/Float64MultiArray.h"
#include <cmath>

#define LOOP_FREQUENCY 1000.0

int print_eigen(std::string str, Eigen::MatrixXd m)
{
	// Eigen Matrices do have rule to print them with std::cout
	std::cout << str<< std::endl << m << std::endl<< std::endl;
	return 0;
}


void moveJoints(double& xs, double& xf, double& phis, double& phif, JointStatePublisher& myPub, double& maxT){
return;
} 

int main(int argc, char** argv){
    // init node
    ros::init(argc, argv, "custom_joint_pub_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_FREQUENCY);
    
    // init topic publisher
    JointStatePublisher myPub;
    myPub.setPubDesJstate(nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10)); 
    

    // Check if gripper is actuated
    bool gripper_sim;
    nh.getParam("/gripper_sim", gripper_sim);
    myPub.setGripper(gripper_sim);

    //UR5 init
    

    while (ros::ok()) {
        
        for(int j = 0; j < TH.cols(); j++ ){
            for (int i = 0; i < 6; i++){
                myPub.q_des[i] = TH(i,j);
            }
            myPub.send_des_jstate();
            loop_rate.sleep();
        }

        Eigen::VectorXd finger = ur5.moveGripper(40, 60, 0, 2);
        for (int i = 0; i < finger.size(); i++){
            myPub.f_des = finger(i);
            myPub.send_des_jstate();
            loop_rate.sleep();
        }


        Vector3d x2(-0.5, -0.45, 0.655001 );
        Vector3d phi2(0.0, 0.0, 0.0);
        ur5.setPoints(x1, x2, phi1, phi2);
        ur5.polinomialCofficients(0.0, 5.0);
        Eigen::MatrixXd TH2 = ur5.IDK_wFB(TH.rightCols(1), 0.0, 5.0);

        for(int j = 0; j < TH.cols(); j++ ){
            for (int i = 0; i < 6; i++){
                myPub.q_des[i] = TH2(i,j);
            }
            myPub.send_des_jstate();
            loop_rate.sleep();
        }


        return 0;
    }

    


    return 0;
}

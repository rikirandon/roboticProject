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


int main(int argc, char** argv){
    // node init
    //
    JointStatePublisher myPub;

    ros::init(argc, argv, "custom_joint_pub_node");
    ros::NodeHandle nh;
    myPub.setPubDesJstate(nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10)); 
    ros::Rate loop_rate(LOOP_FREQUENCY);

    // Check if gripper is actuated
    bool gripper_sim;
    nh.getParam("/gripper_sim", gripper_sim);
    myPub.setGripper(gripper_sim);

    //Ur5 init
    //
    UR5 ur5(0.001, 2.0 * Matrix3d::Identity(), 2.0 * Matrix3d::Identity());
    Vector6d Theta; Theta << -0.32,-0.78, -2.56,-1.63, -1.57, 3.49;
    Matrix4d direct = ur5.Direct(Theta); 
    Vector3d xs = direct.col(3).head(3);
    Matrix3d Re = direct.block(0, 0, 3, 3);
    Vector3d phis = ur5.rotm2eul(Re);
    Vector3d x1(0.151832, -0.190828, 0.655001 );
    Vector3d phi1(0.0, 0.0, 0);
    ur5.setPoints(xs, x1, phis, phi1);
    ur5.polinomialCofficients(0.0, 5.0);
    Eigen::MatrixXd TH = ur5.IDK_wFB(Theta, 0.0, 5.0);
    

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

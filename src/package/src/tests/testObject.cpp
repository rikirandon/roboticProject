#include "../../include/package/ur5Object.h"

int print_eigen(std::string str, Eigen::MatrixXd m)
{
	// Eigen Matrices do have rule to print them with std::cout
	std::cout << str<< std::endl << m << std::endl<< std::endl;
	return 0;
}

int main(){


    Vector6d Theta;
    Theta << -0.32,-0.78, -2.56,-1.63, -1.57, 3.49;
    UR5 ur5(0.001, 2.0 * Matrix3d::Identity(), 2.0 * Matrix3d::Identity());

    //test direct
    Matrix4d direct = ur5.Direct(Theta);
    print_eigen("directKinematics", direct);

    // test rot2eul
    Matrix3d Re = direct.block(0, 0, 3, 3);
    Vector3d xe = direct.col(3).head(3);
    Vector3d xyz = ur5.rotm2eul(Re);
    print_eigen("X, Y, Z", xyz);
    
    //test combine2vector
    Vector6d combined = ur5.combine2Vector(Vector3d(0, 1, 2), Vector3d(3, 4, 5));
    print_eigen("combinedVectors", combined);

    //test polinomialcoefficient
    Vector3d x1(0.1, 0.2, 0.5);
    Vector3d phi1(0, 0, 0);

    ur5.setPoints(xe, x1, xyz, phi1);
    ur5.polinomialCofficients(0.0, 1.0);
    Eigen::MatrixXd TH = ur5.IDK_wFB(Theta, 0.0, 1.0);
    print_eigen("IDK_wFB", TH);


    
    Vector3d x2(0.1, 0.2, 0.5);
    Vector3d phi2(0, 0, 0);
    ur5.setPoints(x1, x2, phi1, phi2);
    ur5.polinomialCofficients(0.0, 1.0);
    Eigen::MatrixXd TH2 = ur5.IDK_wFB(Theta, 0.0, 1.0);
    print_eigen("IDK_wFB", TH2);

    


    //test jacobian
    // Matrix6d jac = ur5.Jacobian(Theta);
    //print_eigen("Jacobian ", jac);


    // test ur5.phid()
    /*
    for(double i = 0; i<5; i += 0.1){
        Vector3d phi = ur5.phid(i);
        print_eigen("phi", phi);
    }
    */

    // test gripper
    Eigen::VectorXd finger = ur5.moveGripper(40 ,60 ,0.0 ,2.0 );
    print_eigen("finger", finger);

  
    

    //test IDK_wFB
    //Eigen::MatrixXd TH = ur5.IDK_wFB(Theta, 0.0, 1.0);
    //print_eigen("IDK_wFB", TH);

    

    return 0;
}

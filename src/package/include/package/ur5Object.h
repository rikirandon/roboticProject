#ifndef __UR5OBJECT__
#define __UR5OBJECT__

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix3d Matrix3d;
typedef Eigen::Matrix4d Matrix4d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector4d Vector4d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;


class UR5{
private:
    double Dt;
    //
    Matrix3d Kp, Kphi;
    //D-H paramiters
    Vector6d A, D, Alpha;
    //Polinomial Coefficients
    Eigen::Matrix<double, 4, 3> PC;


    Vector3d xs, xf, phis, phif;


public:
    //Constructor
    UR5(double _Dt, Matrix3d _Kp, Matrix3d _Kphi);
    
    //Set start and final position and orentation of the end effector
    void setPoints(Vector3d& _xs, Vector3d& _xf, Vector3d& _phis, Vector3d& _phif);

    // 
    Matrix4d generalTransformMatrix(double theta, double alpha, double d, double a);
    //Given joints' positions return Direct Kinematics matrix
    Matrix4d Direct(const Vector6d& Theta);
    //Given joints' positions return Jacobian matrix
    Matrix6d Jacobian(const Vector6d& Theta);
    //Return new joints' velocity 
    Vector6d IDK_newVelocity(Vector6d& q, Vector3d& xe, const Vector3d& xd, Vector3d& vd, Vector3d& phie, const Vector3d& phid, Vector3d& phiddot);
    //Calculate the trajectory and return all the joints' positions in time
    Eigen::MatrixXd IDK_wFB(const Vector6d& TH0, double minT, double maxT);

    //
    void polinomialCofficients(double minT, double maxT);
    Vector3d xd(double t);
    Vector3d phid(double t);
    
    // Gripper
    double mapToGripperJoints(double diameter);
    Eigen::VectorXd moveGripper(double ds, double df, double minT, double maxT);

    
    //
    template <int Rows, int Cols>
    void Purge(Eigen::Matrix<double, Rows, Cols>& matrix);

    Vector3d rotm2eul(const Matrix3d& rotationMatrix);

    Vector6d combine2Vector(const Vector3d& v1, const Vector3d& v2);



};

#endif // __UR5OBJECT__

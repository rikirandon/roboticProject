#include "../../include/package/ur5Object.h"

/* initializing UR5 object*/  
UR5::UR5(double _Dt, Matrix3d _Kp, Matrix3d _Kphi){
    // time delta 
    Dt = _Dt;
    // proportional constants for translation and rotation
    Kp = _Kp;
    Kphi = _Kphi;

    // DH parameters init (link length, link offset and joint angles)
    A <<  0, -0.425, -0.3922, 0, 0, 0;
    D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    Alpha << M_PI/2,0,0,M_PI/2,-M_PI/2,0;
}


/* set the start and final position and orientation*/
void UR5::setPoints(Vector3d& _xs, Vector3d& _xf, Vector3d& _phis, Vector3d& _phif){
    xs = _xs;
    xf = _xf;
    phis = _phis;
    phif = _phif;
}

/* Generates the transform matrix */
Matrix4d UR5::generalTransformMatrix(double theta, double alpha, double d, double a){
    // input the DH parameters computed following the convention
    Matrix4d m;
    // substitute them into the generic matrix
    m <<    cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
            0, sin(alpha), cos(alpha), d,
            0, 0, 0, 1;
    // return the transformation matrix
    return m;     
}

/* Calculate and return the direct kinematics*/
Matrix4d UR5::Direct(const Vector6d& Theta){
    /*provide the theta values' vector
    Note that the other parameters where initialized in the UR5
    object constructor*/
    //initialize and compute the transition matrices
    Matrix4d T10, T21, T32, T43, T54, T65, T60;
    T10 = generalTransformMatrix(Theta(0), Alpha(0), D(0), A(0));
    T21 = generalTransformMatrix(Theta(1), Alpha(1), D(1), A(1));
    T32 = generalTransformMatrix(Theta(2), Alpha(2), D(2), A(2));
    T43 = generalTransformMatrix(Theta(3), Alpha(3), D(3), A(3));
    T54 = generalTransformMatrix(Theta(4), Alpha(4), D(4), A(4));
    T65 = generalTransformMatrix(Theta(5), Alpha(5), D(5), A(5));

    // multiply the transition matrices 
    T60 = T10 * T21 * T32 * T43 * T54 * T65;

    // adjust close-to-zero values
    Purge(T60);
    return T60;
}

/* given a set of joint angles, compute the jacobian matrix*/
Matrix6d UR5::Jacobian(const Vector6d& Theta){

    // extract the parameters initialized in the constructor 
    double A1 = A(0), A2 = A(1), A3 = A(2), A4 = A(3), A5 = A(4), A6 = A(5);
    double D1 = D(0), D2 = D(1), D3 = D(2), D4 = D(3), D5 = D(4), D6 = D(5);
    double th1 = Theta(0), th2 = Theta(1), th3 = Theta(2), th4 = Theta(3), th5 = Theta(4), th6 = Theta(5);

    // init 6 6d vectors to store every column of the jacobian
    Vector6d J1, J2, J3, J4, J5, J6;

    //UR5 has only revolute joints, use the formula
    //z_(i-1) x (p_6-p_(i-1))
    //       z_(i-1)
    J1 <<   D5*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) + D4*cos(th1) - A2*cos(th2)*sin(th1) - D5*sin(th2 + th3 + th4)*sin(th1) - A3*cos(th2)*cos(th3)*sin(th1) + A3*sin(th1)*sin(th2)*sin(th3),
            D5*(cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5)) + D4*sin(th1) + A2*cos(th1)*cos(th2) + D5*sin(th2 + th3 + th4)*cos(th1) + A3*cos(th1)*cos(th2)*cos(th3) - A3*cos(th1)*sin(th2)*sin(th3),
            0,  
            0,
            0,
            1;
    
    J2 <<   -cos(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4))),
            -sin(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4))),
            A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + A2*cos(th2) + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4),
            sin(th1),
            -cos(th1),
            0;
    
    J3 <<   cos(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5)),
            sin(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5)),
            A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4),
            sin(th1),
            -cos(th1),
            0;
    
    J4 <<   D5*cos(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5)),
            D5*sin(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5)),
            D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4) - sin(th2 + th3 + th4 + th5)/2),
            sin(th1),
            -cos(th1),
            0;

    J5 <<   D5*cos(th1)*cos(th2)*cos(th5)*sin(th3)*sin(th4) - D5*cos(th1)*cos(th2)*cos(th3)*cos(th4)*cos(th5) - D5*sin(th1)*sin(th5) + D5*cos(th1)*cos(th3)*cos(th5)*sin(th2)*sin(th4) + D5*cos(th1)*cos(th4)*cos(th5)*sin(th2)*sin(th3),
            D5*cos(th1)*sin(th5) + D5*cos(th2)*cos(th5)*sin(th1)*sin(th3)*sin(th4) + D5*cos(th3)*cos(th5)*sin(th1)*sin(th2)*sin(th4) + D5*cos(th4)*cos(th5)*sin(th1)*sin(th2)*sin(th3) - D5*cos(th2)*cos(th3)*cos(th4)*cos(th5)*sin(th1),
            -D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4 + th5)/2),
            sin(th2 + th3 + th4)*cos(th1),
            sin(th2 + th3 + th4)*sin(th1),
            -cos(th2 + th3 + th4);
    J6 <<   0,
            0,
            0,
            cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5),
            -cos(th1)*cos(th5) - cos(th2 + th3 + th4)*sin(th1)*sin(th5),
            -sin(th2 + th3 + th4)*sin(th5);
    
    // assemble the matrix
    Matrix6d Jacobian;
    Jacobian.col(0) = J1;
    Jacobian.col(1) = J2;
    Jacobian.col(2) = J3;
    Jacobian.col(3) = J4;
    Jacobian.col(4) = J5;
    Jacobian.col(5) = J6;

    // as before, purge small values 
    Purge(Jacobian);
    return Jacobian;

}
/* Implement the inverse differential kinematics to compute the joints' velocities*/
Vector6d UR5::IDK_newVelocity(Vector6d& q, Vector3d& xe, const Vector3d& xd, Vector3d& vd, Vector3d& phie, const Vector3d& phid, Vector3d& phiddot){
    /*the parameters are
    - the current joints' angles
    - the effective end effector position and orientation
    - the desired end effector position and orientation
    - the end effector velocity (translational and rotational)*/

    // compute the jacobian given the current joints' angles
    Matrix6d Jac = Jacobian(q);
    // joints' velocities 
    Vector6d dotq;
    
    // extract the Euler angles from the end effector orientation vector
    double alpha = phie(2), beta = phie(1), gamma = phie(0);
    
    Eigen::MatrixXd T(3,3), Ta(6,6), Ja(6,6);

    // create the rotation matrix T from the euler's angles
    T << cos(beta)*cos(gamma), -sin(gamma), 0,
        cos(beta)*sin(gamma), cos(gamma), 0,
        -sin(beta), 0, 1;

    // if the determinant is fairly close to zero, it means that we are near a singular configurartion
    if (abs(T.determinant()) < 1e-3){
        std::cerr << "Near singular configuration\n ";
        std::cerr << "alpha: " << phie(2) << ", beta: " << phie(1) << ", gamma: " << phie(0) << std::endl;
        // choose a small damping factor
        
    }
    
 
        
    // dotq = Jac_pseudo_inverse.inverse() * combine2Vector((vd + Kp * (xd - xe)), (phiddot + Kphi * (phid - phie)));

    // assemble Ta matrix from T
    Ta << Matrix3d::Identity(), Matrix3d::Zero(),
          Matrix3d::Zero(), T;
    // with the computed Ta calculate the analytical Jacobian
    Ja << Ta.inverse() * Jac;

    double damping_factor = 0.0001; 
    // compute the damped pseudo inverse
    Eigen::MatrixXd Ja_pseudo_inverse = (Ja.transpose() * Ja + damping_factor * Eigen::MatrixXd::Identity(6, 6)).inverse()*Ja.transpose();
    // calculate them using the pre-initialized proportial factor as a correction metric
    dotq = Ja_pseudo_inverse * combine2Vector((vd + Kp * (xd - xe)), (phiddot + Kphi * (phid - phie)));

    return dotq;
} 

// inverse kinematics with feedback
/* calculates joint trajectories over a given time range by iteratively 
updating the joint positions using the calculated joint velocities */
Eigen::MatrixXd UR5::IDK_wFB(const Vector6d& TH0, double minT, double maxT) {

    // calulate the number of time steps and create a vectos T of time frames
    int L = static_cast<int>((maxT - minT) / Dt) + 1;
    Eigen::VectorXd T = Eigen::VectorXd::LinSpaced(L, minT, maxT);

    // initialize the joint angles 
    Vector6d qk = TH0;

    // create a matrix where to store the joints' positions over time
    Eigen::MatrixXd q(TH0.rows(), L);
    // store the initial vector value in the first column of the matrix
    q.col(0) = qk;

    Matrix6d J;
    Vector6d dotqk;
    Matrix4d dirKin;
    Matrix3d Re;
    Vector3d xe, vd, phie, phiddot;
    double t;

    // loop for every time frame
    for (int i = 1; i < L - 1; ++i) {
        t = T(i); //save the current time

        //calculate the direct kinematics to get the end effector pose and orientation
        dirKin = Direct(qk);

        // extract the rotation matrix from the direct kinematics matric
        Re = dirKin.block(0, 0, 3, 3);
        // extract the translation vector 
        xe = dirKin.col(3).head(3);
        // convert the current end effector orientation in euler angles
        phie = rotm2eul(Re);

        // compute the desired end effector velocity
        vd = (xd(t)-xd(t-Dt))/Dt;
        // and desired angular velocity
        phiddot = (phid(t)-phid(t-Dt))/Dt;
        
        // calculate the joints' velocities using the inverse kinematics
        dotqk = IDK_newVelocity(qk, xe, xd(t), vd, phie, phid(t), phiddot);
        
        // update the joints' position 
        qk = qk + dotqk * Dt;

        //store the position in the q matrix
        q.col(i + 1) = qk;
    }

    return q;
}

// function used to calculate the desired end effector position using polynomial trajectory
Vector3d UR5::xd(double t){
    Vector3d xd;
    for (int i = 0; i<3; ++i){
        xd(i) = positionPC(0, i) + positionPC(1, i) * t + positionPC(2, i) * t * t + positionPC(3, i) * t * t * t;
    }
    return xd;
}

// Function to calculate the desired end-effector orientation at time t using linear interpolation
Vector3d UR5::phid(double t){
    Vector3d phid;
    for (int i = 0; i<3; ++i){
        phid(i) = orentationPC(0, i) + orentationPC(1, i) * t + orentationPC(2, i) * t * t + orentationPC(3, i) * t * t * t;
    }
    return phid;
}

// Function to calculate polynomial coefficients for trajectory planning
void UR5::polinomialCofficients(double minT, double maxT){

    for (int i = 0; i < 3; ++i) {
        Matrix4d M;
        M << 1, minT, minT * minT, minT * minT * minT,
             0, 1, 2 * minT, 3 * minT * minT,
             1, maxT, maxT * maxT, maxT * maxT * maxT,
             0, 1, 2 * maxT, 3 * maxT * maxT;
        Vector4d b;
        // position coefficients
        b << xs(i), 0, xf(i), 0;
        positionPC.col(i) = M.inverse() * b;
        // orentation coefficients
        b << phis(i), 0, phif(i), 0;
        orentationPC.col(i) = M.inverse() * b;
    }

}


// Gripper 
// function to map the gripper diamater to joint angles
double UR5::mapToGripperJoints(double diameter){
    int D0 = 40, L = 60;
    // calculate the angular displacement of the gripper joints based on the diameter
    double delta = 0.5 * (diameter - D0);

    // return gripper joint angles
    return atan2(delta, L);
}
// gripper movement function
Eigen::VectorXd UR5::moveGripper(double ds, double df, double minT, double maxT){
    // build the polynomial coefficients 
    Matrix4d M;
    M << 1, minT, minT * minT, minT * minT * minT,
            0, 1, 2 * minT, 3 * minT * minT,
            1, maxT, maxT * maxT, maxT * maxT * maxT,
            0, 1, 2 * maxT, 3 * maxT * maxT;

    Vector4d b;
    // define inital and final position of the gripper opening
    b << ds, 0, df, 0;

    // calculate the polynomial coefficients 
    b = M.inverse() * b;

    // number of steps based on time and a vector where to store the angle values
    int steps = static_cast<int>((maxT - minT) / 0.001) + 1;
    Eigen::VectorXd finger(steps);
    double diameter;
    int i = 0;

    for (double t = 0; t <= maxT; t+=0.001){
        // calculate the gripper aperture using cubic polynomials
        diameter = b(0) + b(1) * t + b(2) * t * t + b(3) * t * t * t;
        // store the values in the vector
        finger(i++) = mapToGripperJoints(diameter);
    }
    return finger;
}

//
/* set to zero the small enough values*/
template <int Rows, int Cols>
void UR5::Purge(Eigen::Matrix<double, Rows, Cols>& matrix){
    // threshold under which we can approx to 0
    double threshold = 0.000001;
    /*iterate on the whole matrix, perform the threshold check
    and decide wether to substitute the value with 0*/
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            if (std::abs(matrix(i, j)) < threshold) {
                matrix(i, j) = 0.0;
            }
        }
    }
}
/* rotation matrix to euler angles representation*/
Vector3d UR5::rotm2eul(const Matrix3d& rotationMatrix) {
    // vector where to store the computed euler angles (RPY)
    Vector3d euler;
    euler(0) = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));  // Roll (X-axis rotation)
    euler(1) = asin(-rotationMatrix(2, 0));                        // Pitch (Y-axis rotation)
    euler(2) = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));  // Yaw (Z-axis rotation)
    return euler;
}

/* function used to concatenate the translation and rotation parts*/
Vector6d UR5::combine2Vector(const Vector3d& v1, const Vector3d& v2){
    return (Vector6d() << v1, v2).finished();
}





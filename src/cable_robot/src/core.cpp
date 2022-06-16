#include "cable_robot/core.h"

Core::Core()
: Node("core"), update(false), mass(0.), a(6, 0.), v(6, 0.), vd(6, 0.), lengths(4), L0(7, 7), /*f0hat(4, 0.)*/f0hat(4, 4.905)
{
    RCLCPP_INFO(this->get_logger(), "Initializing CDPR Core");
    // Load params
    mass = this->declare_parameter<double>("platform.mass", 0.);
    fMax = this->declare_parameter<double>("joints.actuated.effort", 0.);
    fMin = this->declare_parameter<double>("joints.actuated.min", 0.);
//    std::cout << fMax << " " << fMin << std::endl;

    controlUpdateInterval = this->declare_parameter<double>("controlUpdateInterval", 0.);
    controlUpdateTimer = this->create_wall_timer(
            std::chrono::microseconds(int(controlUpdateInterval*1000000)), [this] { updateCallback(); });

    // inertia matrix
    auto inertiaVec = this->declare_parameter<std::vector<double>>("platform.inertia");
    Ip.resize(3, 3);
    for(int i = 0; i < 3; ++i)
    {
        Ip[i][i] = inertiaVec[i];
    }
    Ip[0][1] = Ip[1][0] = inertiaVec[3];
    Ip[0][2] = Ip[2][0] = inertiaVec[4];
    Ip[2][1] = Ip[1][2] = inertiaVec[5];

    // cable attach points
    std::vector<double> point;
    nCables = (int)this->declare_parameter<int>("n_cables", 4);
    for (int i = 0; i < nCables; ++i)
    {
        point = this->declare_parameter<std::vector<double>>("points.frame" + std::to_string(i));
        framePoints.emplace_back(point);
        point = this->declare_parameter<std::vector<double>>("points.platform" + std::to_string(i));
        platformPoints.emplace_back(point);
    }

    // initial desired pose = home
    auto xyz = this->declare_parameter<std::vector<double>>("platform.position.xyz");
    auto rpy = this->declare_parameter<std::vector<double>>("platform.position.rpy");
    r.buildFrom(xyz[0], xyz[1], xyz[2]);
    R.buildFrom(vpRxyzVector(rpy[0], rpy[1], rpy[2]));
    rd = r;
    alpha0 = rpy[0], beta0 = rpy[1], gamma0 = rpy[2];
    computeLengths();
    computeL();
//    vpRxyzVector rpy(rpy[0], rpy[1], rpy[2]);
//    Td.insert(vpRotationMatrix(r));
//    Td.insert(vpTranslationVector(xyz[0], xyz[1], xyz[2]));

    char cable_name[256];
    for (int i = 0; i < nCables; ++i)
    {
        sprintf(cable_name, "cable%i", i);
        tensionsMsg.name.push_back(std::string(cable_name));
    }
    tensionsMsg.effort.resize(nCables);

    platformStateSub = this->create_subscription<gazebo_msgs::msg::LinkState>(
            "platform_state", 5, std::bind(&Core::platformStateCallback, this, std::placeholders::_1)
    );
    platformAccelSub = this->create_subscription<geometry_msgs::msg::Accel>(
            "platform_accel", 5, std::bind(&Core::platformAccelCallback, this, std::placeholders::_1)
    );
    tensionsPub = this->create_publisher<sensor_msgs::msg::JointState>("tensions", 5);
    realPositionPub = this->create_publisher<geometry_msgs::msg::Point>("real_position", 5);
    desiredPositionSub = this->create_subscription<geometry_msgs::msg::Point>(
            "desired_position", 5, std::bind(&Core::desiredPositionCallback, this, std::placeholders::_1));
    desiredVelocitySub = this->create_subscription<geometry_msgs::msg::Twist>(
            "desired_velocity", 5, std::bind(&Core::desiredVelocityCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "CDPR Core initialized");
}

void Core::computeW(vpMatrix &W)
{
    // build W matrix depending on current attach points
    vpTranslationVector u;
    vpColVector bu;
    for (int i = 0; i < nCables; ++i)
    {
//        // vector between platform point and frame point in platform frame
//        f = R.t() * (framePoints[i] - t) - platformPoints[i];
//        f /= f.frobeniusNorm();
//        // corresponding force in platform frame
//        w = platformPoints[i].skew() * f;
//        for (unsigned int k = 0; k < 3; ++k)
//        {
//            W[k][i] = f[k];
//            W[k+3][i] = w[k];
//        }

        // cable unit vector in global frame
        u = framePoints[i] - r - R*platformPoints[i]; // l = a - c - Rb;
        u /= u.frobeniusNorm();
        // corresponding force in global frame
        bu = (R*platformPoints[i]).skew() * u;
//        bu = u.skew() * R*platformPoints[i];
        for (unsigned int k = 0; k < 3; ++k)
        {
            W[k][i] = u[k];
            W[k+3][i] = bu[k];
        }
    }
}

void Core::computeLengths()
{
    for (int i = 0; i < nCables; ++i)
        lengths[i] = (framePoints[i] - r - R * platformPoints[i]).frobeniusNorm();
}

void Core::computeL()
{
    for (int i = 0; i < nCables; ++i)
        L0[i][i] = lengths[i];
}

void Core::computeQPMatrices(vpMatrix &Aeq, vpColVector &Beq, vpMatrix &Aineq, vpColVector &Bineq)
//void Core::computeQPMatrices(Eigen::MatrixXd &Aeq, Eigen::VectorXd &Beq, Eigen::MatrixXd &Aineq, Eigen::VectorXd &Bineq)
{
    double  a11 = framePoints[0][0], a12 = framePoints[0][1], a13 = framePoints[0][2],
            a21 = framePoints[1][0], a22 = framePoints[1][1], a23 = framePoints[1][2],
            a31 = framePoints[2][0], a32 = framePoints[2][1], a33 = framePoints[2][2],
            a41 = framePoints[3][0], a42 = framePoints[3][1], a43 = framePoints[3][2];
    double  b11 = platformPoints[0][0], b12 = platformPoints[0][1], b13 = platformPoints[0][2],
            b21 = platformPoints[1][0], b22 = platformPoints[1][1], b23 = platformPoints[1][2],
            b31 = platformPoints[2][0], b32 = platformPoints[2][1], b33 = platformPoints[2][2],
            b41 = platformPoints[3][0], b42 = platformPoints[3][1], b43 = platformPoints[3][2];

    double x = rd[0], y = rd[1], z = rd[2];
    double sa = sin(alpha0), ca = cos(alpha0), sb = sin(beta0), cb = cos(beta0), sg = sin(gamma0), cg = cos(gamma0);
    double s2a = sin(2*alpha0), c2a = cos(2*alpha0), s2b = sin(2*beta0), c2b = cos(2*beta0), s2g = sin(2*gamma0), c2g = cos(2*gamma0);
    double sa2 = pow(sa, 2), ca2 = pow(ca, 2), sb2 = pow(sb, 2), cb2 = pow(cb, 2), sg2 = pow(sg, 2), cg2 = pow(cg, 2);
    double f01 = f0hat[0], f02 = f0hat[1], f03 = f0hat[2], f04 = f0hat[3];
    double Ixx = Ip[0][0], Iyy = Ip[1][1], Izz = Ip[2][2], Ixy = Ip[0][1], Ixz = Ip[0][2], Iyz = Ip[1][2];

    double eps = 1e-1;
    double Kp = 3, Kd = 0.01, dtm1 = 1/controlUpdateInterval;
    double c = Kp + Kd*dtm1;// + dtm1*dtm1;

    std::cout << "c: " << c << std::endl;
    std::cout << a11 << " " << a12 << " " << a13 << std::endl;
    std::cout << b11 << " " << b12 << " " << b13 << std::endl;

    std::vector<double> dX =     {rd[0]-r[0], rd[1]-r[1], rd[2]-r[2], 0, 0, 0};
    std::vector<double> dXdot =  {vd[0]-v[0], vd[1]-v[1], vd[2]-v[2], 0, 0, 0};
//    vpColVector X0 = -c*vpColVector(X) + (-dtm1 - Kd)*v;
//    double X01 = X0[0], X02 = X0[1], X03 = X0[2], X04 = X0[3], X05 = X0[4], X06 = X0[5];
    vpColVector X0 = (Kd + dtm1)*vpColVector(dXdot) + Kp*vpColVector(dX);
    double X01 = X0[0], X02 = X0[1], X03 = X0[2], X04 = X0[3], X05 = X0[4], X06 = X0[5];

    Aeq[0][0] = a11 - x - b13*sb + cb*(-b11*cg + b12*sg);
    Aeq[1][0] = a12 - y + b13*cb*sa - b11*cg*sb*sa + b12*sb*sa*sg - ca*(b12*cg + b11*sg);
    Aeq[2][0] = a13 - z - b13*cb*ca - b12*cg*sa - b11*sa*sg + ca*sb*(b11*cg - b12*sg);
    Aeq[3][0] = a13*b12 - a12*b13 + b13*y - b12*z;
    Aeq[4][0] = -a13*b11 + a11*b13 - b13*x + b11*z;
    Aeq[5][0] = a12*b11 - a11*b12 + b12*x - b11*y;

    Aeq[0][1] = a21 - x - b23*sb + cb*(-b21*cg + b22*sg);
    Aeq[1][1] = a22 - y + b23*cb*sa - b21*cg*sb*sa + b22*sb*sa*sg - ca*(b22*cg + b21*sg);
    Aeq[2][1] = a23 - z - b23*cb*ca - b22*cg*sa - b21*sa*sg + ca*sb*(b21*cg - b22*sg);
    Aeq[3][1] = a23*b22 - a22*b23 + b23*y - b22*z;
    Aeq[4][1] = -a23*b21 + a21*b23 - b23*x + b21*z;
    Aeq[5][1] = a22*b21 - a21*b22 + b22*x - b21*y;

    Aeq[0][2] = a31 - x - b33*sb + cb*(-b31*cg + b32*sg);
    Aeq[1][2] = a32 - y + b33*cb*sa - b31*cg*sb*sa + b32*sb*sa*sg - ca*(b32*cg + b31*sg);
    Aeq[2][2] = a33 - z - b33*cb*ca - b32*cg*sa - b31*sa*sg + ca*sb*(b31*cg - b32*sg);
    Aeq[3][2] = a33*b32 - a32*b33 + b33*y - b32*z;
    Aeq[4][2] = -a33*b31 + a31*b33 - b33*x + b31*z;
    Aeq[5][2] = a32*b31 - a31*b32 + b32*x - b31*y;

    Aeq[0][3] = a41 - x - b43*sb + cb*(-b41*cg + b42*sg);
    Aeq[1][3] = a42 - y + b43*cb*sa - b41*cg*sb*sa + b42*sb*sa*sg - ca*(b42*cg + b41*sg);
    Aeq[2][3] = a43 - z - b43*cb*ca - b42*cg*sa - b41*sa*sg + ca*sb*(b41*cg - b42*sg);
    Aeq[3][3] = a43*b42 - a42*b43 + b43*y - b42*z;
    Aeq[4][3] = -a43*b41 + a41*b43 - b43*x + b41*z;
    Aeq[5][3] = a42*b41 - a41*b42 + b42*x - b41*y;

    Aeq[0][4] = -((b12*f01 + b22*f02 + b32*f03 + b42*f04)*sb) - (b13*f01 + b23*f02 + b33*f03 + b43*f04)*cb*sg;
    Aeq[1][4] = (b13*f01 + b23*f02 + b33*f03 + b43*f04)*ca*cg + sa*((b12*f01 + b22*f02 + b32*f03 + b42*f04)*cb - (b13*f01 + b23*f02 + b33*f03 + b43*f04)*sb*sg);
    Aeq[2][4] = -((b12*f01 + b22*f02 + b32*f03 + b42*f04)*cb*ca) + (b13*f01 + b23*f02 + b33*f03 + b43*f04)*(cg*sa + ca*sb*sg);
    Aeq[3][4] = -c*(Izz*sb2 + Ixz*cg*s2b - 2*Iyz*cb*sb*sg + cb2*(Ixx*cg2 - 2*Ixy*cg*sg + Iyy*sg2));
    Aeq[4][4] = -c*(-cb2*sa*(Ixz*cg - Iyz*sg) + sb*(ca*(Iyz*cg + Ixz*sg) + sb*sa*(Ixz*cg - Iyz*sg)) + cb*(ca*(Ixy*c2g + (Ixx-Iyy)*cg*sg) + sb*sa*(-Izz + Ixx*cg2 - 2*Ixy*cg*sg + Iyy*sg2)));
    Aeq[5][4] = -c*(cb2*ca*(Ixz*cg - Iyz*sg) + sb*(sa*(Iyz*cg + Ixz*sg) - ca*sb*(Ixz*cg - Iyz*sg)) + cb*(sa*(Ixy*c2g + (Ixx-Iyy)*cg*sg) + ca*sb*(Izz - Ixx*cg2 - Iyy*sg2 + Ixy*s2g)));

    Aeq[0][5] = -((b13*f01 + b23*f02 + b33*f03 + b43*f04)*cb*cg) + (b11*f01 + b21*f02 + b31*f03 + b41*f04)*sb;
    Aeq[1][5] = -((b11*f01 + b21*f02 + b31*f03 + b41*f04)*cb*sa) - (b13*f01 + b23*f02 + b33*f03 + b43*f04)*(cg*sb*sa + ca*sg);
    Aeq[2][5] = (b11*f01 + b21*f02 + b31*f03 + b41*f04)*cb*ca + (b13*f01 + b23*f02 + b33*f03 + b43*f04)*(ca*cg*sb - sa*sg);
    Aeq[3][5] = -c*(-cb2*sa*(Ixz*cg - Iyz*sg) + sb*(ca*(Iyz*cg + Ixz*sg) + sb*sa*(Ixz*cg - Iyz*sg)) + cb*(ca*(Ixy*c2g + (Ixx-Iyy)*cg*sg) + sb*sa*(-Izz+Ixx*cg2 - 2*Ixy*cg*sg + Iyy*sg2)));
    Aeq[4][5] = -c*(Izz*cb2*sa2 - 2*Ixz*cb*cg*sb*sa2 + Ixx*cg2*sb2*sa2 + Ixy*cg2*sb*s2a - 2*Ixy*cg*sb2*sa2*sg + Iyz*s2b*sa2*sg + Iyy*sb2*sa2*sg2 - 2*ca*sa*(sb*sg*((-Ixx+Iyy)*cg + Ixy*sg) + cb*(Iyz*cg + Ixz*sg)) + ca2*(Iyy*cg2 + Ixx*sg2 + Ixy*s2g));
    Aeq[5][5] = c*(Izz*cb2*ca*sa + cb*(-2*Ixz*ca*cg*sb*sa + Iyz*cg*sa2 + (Ixz*sa2 + Iyz*sb*s2a)*sg - ca2*(Iyz*cg + Ixz*sg)) + 1./2*(ca2*sb*(2*Ixy*c2g + (Ixx-Iyy)*s2g) - sb*sa2*(2*Ixy*c2g + (Ixx-Iyy)*s2g) - 2*ca*sa*(cg2*(Iyy-Ixx*sb2) + (Ixx-Iyy*sb2)*sg2 + Ixy*(1 + sb2)*s2g)));

    Aeq[0][6] = cb*((b12*f01 + b22*f02 + b32*f03 + b42*f04)*cg + (b11*f01 + b21*f02 + b31*f03 + b41*f04)*sg);
    Aeq[1][6] = sb*sa*((b12*f01 + b22*f02 + b32*f03 + b42*f04)*cg + (b11*f01 + b21*f02 + b31*f03 + b41*f04)*sg) + ca*(-((b11*f01 + b21*f02 + b31*f03 + b41*f04)*cg) + (b12*f01 + b22*f02 + b32*f03 + b42*f04)*sg);
    Aeq[2][6] = -(ca*sb*((b12*f01 + b22*f02 + b32*f03 + b42*f04)*cg + (b11*f01 + b21*f02 + b31*f03 + b41*f04)*sg)) + sa*(-((b11*f01 + b21*f02 + b31*f03 + b41*f04)*cg) + (b12*f01 + b22*f02 + b32*f03 + b42*f04)*sg);
    Aeq[3][6] = -c*(cb2*ca*(Ixz*cg - Iyz*sg) + sb*(sa*(Iyz*cg + Ixz*sg) - ca*sb*(Ixz*cg - Iyz*sg)) + cb*(sa*(Ixy*c2g + (Ixx - Iyy)*cg*sg) + ca*sb*(Izz - Ixx*cg2 - Iyy*sg2 + Ixy*s2g)));
    Aeq[4][6] = c*(Izz*cb2*ca*sa + cb*(-2*Ixz*ca*cg*sb*sa + Iyz*cg*sa2 + (Ixz*sa2 + Iyz*sb*s2a)*sg - ca2*(Iyz*cg + Ixz*sg)) + (ca2*sb*(2*Ixy*c2g + (Ixx - Iyy)*s2g) - sb*sa2*(2*Ixy*c2g + (Ixx - Iyy)*s2g) - 2*ca*sa*(cg2*(Iyy - Ixx*sb2) + (Ixx - Iyy*sb2)*sg2 + Ixy*(1 + sb2)*s2g))/2.);
    Aeq[5][6] = -c*(Izz*cb2*ca2 + Iyy*cg2*sa2 + Ixx*sa2*sg2 + Ixy*sb*s2a*sg2 - cg*sb*s2a*(Ixy*cg + (Ixx - Iyy)*sg) + cb*(-2*Ixz*ca2*cg*sb + s2a*(Iyz*cg + Ixz*sg)) +ca2*(Ixx*cg2*sb2 - 2*Ixy*cg*sb2*sg + sg*(Iyz*s2b + Iyy*sb2*sg)) + Ixy*sa2*s2g);

    Beq[0] = mass*X01 - f01*(a11 - x - b11*cb*cg - b13*sb + b12*cb*sg) - f02*(a21 - x - b21*cb*cg - b23*sb + b22*cb*sg) - f03*(a31 - x - b31*cb*cg - b33*sb + b32*cb*sg) - f04*(a41 - x - b41*cb*cg - b43*sb + b42*cb*sg);
    Beq[1] = mass*X02 - f01*(a12 - y + b13*cb*sa - b11*(cg*sb*sa + ca*sg) - b12*(ca*cg - sb*sa*sg)) - f02*(a22 - y + b23*cb*sa - b21*(cg*sb*sa + ca*sg) - b22*(ca*cg - sb*sa*sg)) - f03*(a32 - y + b33*cb*sa - b31*(cg*sb*sa + ca*sg) - b32*(ca*cg - sb*sa*sg)) - f04*(a42 - y + b43*cb*sa - b41*(cg*sb*sa + ca*sg) - b42*(ca*cg - sb*sa*sg));
    Beq[2] = 9.81*mass + mass*X03 - f01*(a13 - z - b13*cb*ca - b12*(cg*sa + ca*sb*sg) - b11*(-(ca*cg*sb) + sa*sg)) - f02*(a23 - z - b23*cb*ca - b22*(cg*sa + ca*sb*sg) - b21*(-(ca*cg*sb) + sa*sg)) - f03*(a33 - z - b33*cb*ca - b32*(cg*sa + ca*sb*sg) - b31*(-(ca*cg*sb) + sa*sg)) - f04*(a43 - z - b43*cb*ca - b42*(cg*sa + ca*sb*sg) - b41*(-(ca*cg*sb) + sa*sg));
    Beq[3] = -(f01*(a13*b12 - a12*b13 + b13*y - b12*z)) - f02*(a23*b22 - a22*b23 + b23*y - b22*z) - f03*(a33*b32 - a32*b33 + b33*y - b32*z) - f04*(a43*b42 - a42*b43 + b43*y - b42*z) + X04*(cb*cg*(Ixx*cb*cg + Ixz*sb - Ixy*cb*sg) - cb*sg*(Ixy*cb*cg + Iyz*sb - Iyy*cb*sg) + sb*(Ixz*cb*cg + Izz*sb - Iyz*cb*sg)) + X06*(cb*ca*(Ixz*cb*cg + Izz*sb - Iyz*cb*sg) + (Ixy*cb*cg + Iyz*sb - Iyy*cb*sg)*(cg*sa + ca*sb*sg) + (Ixx*cb*cg + Ixz*sb - Ixy*cb*sg)*(-(ca*cg*sb) + sa*sg)) + X05*(-(cb*sa*(Ixz*cb*cg + Izz*sb - Iyz*cb*sg)) + (Ixx*cb*cg + Ixz*sb - Ixy*cb*sg)*(cg*sb*sa + ca*sg) + (Ixy*cb*cg + Iyz*sb - Iyy*cb*sg)*(ca*cg - sb*sa*sg));
    Beq[4] = -(f01*(-(a13*b11) + a11*b13 - b13*x + b11*z)) - f02*(-(a23*b21) + a21*b23 - b23*x + b21*z) - f03*(-(a33*b31) + a31*b33 - b33*x + b31*z) - f04*(-(a43*b41) + a41*b43 - b43*x + b41*z) + X06*((-(ca*cg*sb) + sa*sg)*(-(Ixz*cb*sa) + Ixx*(cg*sb*sa + ca*sg) + Ixy*(ca*cg - sb*sa*sg)) + (cg*sa + ca*sb*sg)*(-(Iyz*cb*sa) + Ixy*(cg*sb*sa + ca*sg) + Iyy*(ca*cg - sb*sa*sg)) + cb*ca*(-(Izz*cb*sa) + Ixz*(cg*sb*sa + ca*sg) + Iyz*(ca*cg - sb*sa*sg))) + X04*(cb*cg*(-(Ixz*cb*sa) + Ixx*(cg*sb*sa + ca*sg) + Ixy*(ca*cg - sb*sa*sg)) - cb*sg*(-(Iyz*cb*sa) + Ixy*(cg*sb*sa + ca*sg) + Iyy*(ca*cg - sb*sa*sg)) + sb*(-(Izz*cb*sa) + Ixz*(cg*sb*sa + ca*sg) + Iyz*(ca*cg - sb*sa*sg))) + X05*((cg*sb*sa + ca*sg)*(-(Ixz*cb*sa) + Ixx*(cg*sb*sa + ca*sg) + Ixy*(ca*cg - sb*sa*sg)) + (ca*cg - sb*sa*sg)*(-(Iyz*cb*sa) + Ixy*(cg*sb*sa + ca*sg) + Iyy*(ca*cg - sb*sa*sg)) - cb*sa*(-(Izz*cb*sa) + Ixz*(cg*sb*sa + ca*sg) + Iyz*(ca*cg - sb*sa*sg)));
    Beq[5] = -(f01*(a12*b11 - a11*b12 + b12*x - b11*y)) - f02*(a22*b21 - a21*b22 + b22*x - b21*y) - f03*(a32*b31 - a31*b32 + b32*x - b31*y) - f04*(a42*b41 - a41*b42 + b42*x - b41*y) + X06*((-(ca*cg*sb) + sa*sg)*(Ixz*cb*ca + Ixy*(cg*sa + ca*sb*sg) + Ixx*(-(ca*cg*sb) + sa*sg)) + (cg*sa + ca*sb*sg)*(Iyz*cb*ca + Iyy*(cg*sa + ca*sb*sg) + Ixy*(-(ca*cg*sb) + sa*sg)) + cb*ca*(Izz*cb*ca + Iyz*(cg*sa + ca*sb*sg) + Ixz*(-(ca*cg*sb) + sa*sg))) + X04*(cb*cg*(Ixz*cb*ca + Ixy*(cg*sa + ca*sb*sg) + Ixx*(-(ca*cg*sb) + sa*sg)) - cb*sg*(Iyz*cb*ca + Iyy*(cg*sa + ca*sb*sg) + Ixy*(-(ca*cg*sb) + sa*sg)) + sb*(Izz*cb*ca + Iyz*(cg*sa + ca*sb*sg) + Ixz*(-(ca*cg*sb) + sa*sg))) + X05*((cg*sb*sa + ca*sg)*(Ixz*cb*ca + Ixy*(cg*sa + ca*sb*sg) + Ixx*(-(ca*cg*sb) + sa*sg)) + (ca*cg - sb*sa*sg)*(Iyz*cb*ca + Iyy*(cg*sa + ca*sb*sg) + Ixy*(-(ca*cg*sb) + sa*sg)) - cb*sa*(Izz*cb*ca + Iyz*(cg*sa + ca*sb*sg) + Ixz*(-(ca*cg*sb) + sa*sg)));

//    Beq[0] = c*mass*x + mass*X01 - f01*(a11 - x - b11*cb*cg - b13*sb + b12*cb*sg) - f02*(a21 - x - b21*cb*cg - b23*sb + b22*cb*sg) - f03*(a31 - x - b31*cb*cg - b33*sb + b32*cb*sg) - f04*(a41 - x - b41*cb*cg - b43*sb + b42*cb*sg);
//    Beq[1] = mass*X02 + c*mass*y - f01*(a12 - y + b13*cb*sa - b11*(cg*sb*sa + ca*sg) - b12*(ca*cg - sb*sa*sg)) - f02*(a22 - y + b23*cb*sa - b21*(cg*sb*sa + ca*sg) - b22*(ca*cg - sb*sa*sg)) - f03*(a32 - y + b33*cb*sa - b31*(cg*sb*sa + ca*sg) - b32*(ca*cg - sb*sa*sg)) - f04*(a42 - y + b43*cb*sa - b41*(cg*sb*sa + ca*sg) - b42*(ca*cg - sb*sa*sg));
//    Beq[2] = 9.81*mass + mass*X03 + c*mass*z - f01*(a13 - z - b13*cb*ca - b12*(cg*sa + ca*sb*sg) - b11*(-(ca*cg*sb) + sa*sg)) - f02*(a23 - z - b23*cb*ca - b22*(cg*sa + ca*sb*sg) - b21*(-(ca*cg*sb) + sa*sg)) - f03*(a33 - z - b33*cb*ca - b32*(cg*sa + ca*sb*sg) - b31*(-(ca*cg*sb) + sa*sg)) - f04*(a43 - z - b43*cb*ca - b42*(cg*sa + ca*sb*sg) - b41*(-(ca*cg*sb) + sa*sg));
//    Beq[3] = -(f01*(a13*b12 - a12*b13 + b13*y - b12*z)) - f02*(a23*b22 - a22*b23 + b23*y - b22*z) - f03*(a33*b32 - a32*b33 + b33*y - b32*z) - f04*(a43*b42 - a42*b43 + b43*y - b42*z) +  X04*(cb*cg*(Ixx*cb*cg + Ixz*sb - Ixy*cb*sg) - cb*sg*(Ixy*cb*cg + Iyz*sb - Iyy*cb*sg) + sb*(Ixz*cb*cg + Izz*sb - Iyz*cb*sg)) + X06*(cb*ca*(Ixz*cb*cg + Izz*sb - Iyz*cb*sg) + (Ixy*cb*cg + Iyz*sb - Iyy*cb*sg)*(cg*sa + ca*sb*sg) + (Ixx*cb*cg + Ixz*sb - Ixy*cb*sg)*(-(ca*cg*sb) + sa*sg)) + X05*(-(cb*sa*(Ixz*cb*cg + Izz*sb - Iyz*cb*sg)) + (Ixx*cb*cg + Ixz*sb - Ixy*cb*sg)*(cg*sb*sa + ca*sg) + (Ixy*cb*cg + Iyz*sb - Iyy*cb*sg)*(ca*cg - sb*sa*sg));
//    Beq[4] = -(f01*(-(a13*b11) + a11*b13 - b13*x + b11*z)) - f02*(-(a23*b21) + a21*b23 - b23*x + b21*z) - f03*(-(a33*b31) + a31*b33 - b33*x + b31*z) - f04*(-(a43*b41) + a41*b43 - b43*x + b41*z) + X06*((-(ca*cg*sb) + sa*sg)*(-(Ixz*cb*sa) + Ixx*(cg*sb*sa + ca*sg) + Ixy*(ca*cg - sb*sa*sg)) + (cg*sa + ca*sb*sg)*(-(Iyz*cb*sa) + Ixy*(cg*sb*sa + ca*sg) + Iyy*(ca*cg - sb*sa*sg)) + cb*ca*(-(Izz*cb*sa) + Ixz*(cg*sb*sa + ca*sg) + Iyz*(ca*cg - sb*sa*sg))) + X04*(cb*cg*(-(Ixz*cb*sa) + Ixx*(cg*sb*sa + ca*sg) + Ixy*(ca*cg - sb*sa*sg)) - cb*sg*(-(Iyz*cb*sa) + Ixy*(cg*sb*sa + ca*sg) + Iyy*(ca*cg - sb*sa*sg)) + sb*(-(Izz*cb*sa) + Ixz*(cg*sb*sa + ca*sg) + Iyz*(ca*cg - sb*sa*sg))) + X05*((cg*sb*sa + ca*sg)*(-(Ixz*cb*sa) + Ixx*(cg*sb*sa + ca*sg) + Ixy*(ca*cg - sb*sa*sg)) + (ca*cg - sb*sa*sg)*(-(Iyz*cb*sa) + Ixy*(cg*sb*sa + ca*sg) + Iyy*(ca*cg - sb*sa*sg)) - cb*sa*(-(Izz*cb*sa) + Ixz*(cg*sb*sa + ca*sg) + Iyz*(ca*cg - sb*sa*sg)));
//    Beq[5] = -(f01*(a12*b11 - a11*b12 + b12*x - b11*y)) - f02*(a22*b21 - a21*b22 + b22*x - b21*y) - f03*(a32*b31 - a31*b32 + b32*x - b31*y) - f04*(a42*b41 - a41*b42 + b42*x - b41*y) + X06*((-(ca*cg*sb) + sa*sg)*(Ixz*cb*ca + Ixy*(cg*sa + ca*sb*sg) + Ixx*(-(ca*cg*sb) + sa*sg)) + (cg*sa + ca*sb*sg)*(Iyz*cb*ca + Iyy*(cg*sa + ca*sb*sg) + Ixy*(-(ca*cg*sb) + sa*sg)) + cb*ca*(Izz*cb*ca + Iyz*(cg*sa + ca*sb*sg) + Ixz*(-(ca*cg*sb) + sa*sg))) + X04*(cb*cg*(Ixz*cb*ca + Ixy*(cg*sa + ca*sb*sg) + Ixx*(-(ca*cg*sb) + sa*sg)) - cb*sg*(Iyz*cb*ca + Iyy*(cg*sa + ca*sb*sg) + Ixy*(-(ca*cg*sb) + sa*sg)) + sb*(Izz*cb*ca + Iyz*(cg*sa + ca*sb*sg) + Ixz*(-(ca*cg*sb) + sa*sg))) + X05*((cg*sb*sa + ca*sg)*(Ixz*cb*ca + Ixy*(cg*sa + ca*sb*sg) + Ixx*(-(ca*cg*sb) + sa*sg)) + (ca*cg - sb*sa*sg)*(Iyz*cb*ca + Iyy*(cg*sa + ca*sb*sg) + Ixy*(-(ca*cg*sb) + sa*sg)) - cb*sa*(Izz*cb*ca + Iyz*(cg*sa + ca*sb*sg) + Ixz*(-(ca*cg*sb) + sa*sg)));

    for (int i = 0; i < nCables; i++)
    {
        Aineq[i][i] = lengths[i];
        Aineq[i+7][i] = -lengths[i];
        Bineq[i] = -lengths[i]*f0hat[i] + fMax;
        Bineq[i+7] = lengths[i]*f0hat[i] - fMin;
    }
    for (int i = nCables; i < nCables+3; i++)
    {
        Aineq[i][i] = 1;
        Aineq[i+7][i] = -1;
        Bineq[i] = eps;
        Bineq[i+7] = eps;
    }

    X0.print(std::cout, 5, "X0");
}

void Core::sendTensions(vpColVector &f)
{
    // write effort to jointstate
    for (int i = 0; i < nCables; ++i)
        tensionsMsg.effort[i] = f[i];
    tensionsMsg.header.stamp = this->now();
    tensionsPub->publish(tensionsMsg);
}

void printVec(vpColVector &v, std::string name)
{
    std::cout << name << ": {";
    for (int i = 0; i < (int)v.size()-1; i++)
        std::cout << v[i] << ",";
    std::cout << v[v.size()-1] << "}" << std::endl;
}

void printMat(vpMatrix &A, std::string name)
{
    std::cout << name << ": ";
    for (int i = 0; i < (int)A.getRows(); i++)
    {
        std::cout << "{";
        for (int k = 0; k < (int)A.getCols()-1; k++)
            std::cout << A[i][k] << ",";
        std::cout << A[i][A.getCols()-1] << "},";
    }
    std::cout << std::endl;
}

void Core::updateCallback()
{
/*//    auto dt = controlUpdateInterval;
//    // current position
//    vpHomogeneousMatrix Tcur;
//    this->getPose(Tcur);
//    vpRotationMatrix R;
//    Tcur.extract(R);
//
//    vpColVector g(6);
//    g[2] = - mass * 9.81;
//    vpMatrix W(6, nCables);
//
////    w = -(M*(Kp*err + Kd*(vd - v)) + g);
////    w = -g;
//    vpColVector omega(3); omega[0] = a[3]; omega[1] = a[4]; omega[2] = a[5];
//    vpColVector Cor(6); Cor[0] = Cor[1] = Cor[2] = 0;
//    vpMatrix Ig = R * Ip * R.t();
//    vpColVector Colpart = vpColVector::skew(omega) * Ig * omega;
//    Cor[3] = Colpart[0]; Cor[4] = Colpart[1]; Cor[5] = Colpart[2];
//
//    vpMatrix M(6, 6);
//    for (int i = 0; i < 3; i++)
//    {
//        M[i][i] = mass;
//        M[i+3][i+3] = Ig[i][i];
//    }
//    M[0+3][1+3] = M[1+3][0+3] = Ig[0][1];
//    M[0+3][2+3] = M[2+3][0+3] = Ig[0][2];
//    M[1+3][2+3] = M[2+3][1+3] = Ig[1][2];
//    w = M*a + Cor - g;
//
//    err = this->getPoseError();
//    vpMatrix R_R(6, 6);               // ?????????
//    for(unsigned int i=0;i<3;++i)
//        for(unsigned int j=0;j<3;++j)
//            R_R[i][j] = R_R[i+3][j+3] = R[i][j];
//    // position error in fixed frame
//    err = R_R * err;
//
//    double Kp = 5., Kd = 3;//, Ki = 0.1
//    w = M*(Kp*err + Kd*(vd - v)) - g;
//
//    // build W matrix depending on current attach points
//    this->computeW(W);
//    tau.cppPrint(std::cout, "tda");

    // send tensions
//    this->sendTensions(tau);*/

    if (!update)
        return;

//    std::vector<double> f0hatEx(7, 0.);
//    for (int i = 0; i < (int)f0hat.size(); i++)
//        f0hatEx[i] = f0hat[i];
//    vpColVector c = -L0 * vpColVector(f0hatEx);
//    vpMatrix Aeq(6, 7), Aineq(14, 7);
//    vpColVector Beq(6), Bineq(14);
//    computeQPMatrices(Aeq, Beq, Aineq, Bineq);
//    vpColVector res(7);
//    std::vector<bool> active(1, false);
//    solveQP(L0, c, Aeq, Beq, Aineq, Bineq, res, active, this->get_logger());

    Eigen::MatrixXd G(7, 7);
    Eigen::VectorXd g0(7);
//    QP::solve_quadprog();

//    std::vector<double> dfhat = {res[0], res[1], res[2], res[3]};
//    RCLCPP_INFO(this->get_logger(), "f0hat: (%.3f, %.3f, %.3f, %.3f); dfhat: (%.3f, %.3f, %.3f, %.3f); dF: (%.3f, %.3f, %.3f)",
//                f0hat[0], f0hat[1], f0hat[2], f0hat[3], res[0], res[1], res[2], res[3], res[4], res[5], res[6]);
////    f0hat = vpColVector(dfhat);
//    f0hat += vpColVector(dfhat);
//    alpha0 += res[4], beta0 += res[5], gamma0 += res[6];
//
//    vpColVector tau = f0hat;
//    for (int i = 0; i < nCables; i++)
//        tau[i] *= lengths[i];
//    this->sendTensions(tau);
//
//    RCLCPP_INFO(this->get_logger(), "Tensions: (%.3f, %.3f, %.3f, %.3f); v: (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
//                tau[0], tau[1], tau[2], tau[3], v[0], v[1], v[2], v[3], v[4], v[5]);

//    L0.print(std::cout, 5, "L0");
//    c.print(std::cout, 5, "c");
//    Aeq.print(std::cout, 5, "Aeq");
//    Beq.print(std::cout, 5, "Beq");
//    Aineq.print(std::cout, 5, "Aineq");
//    Bineq.print(std::cout, 5, "Bineq");

//    printMat(L0, "L0");
//    printVec(c, "c");
//    printMat(Aeq, "Aeq");
//    printVec(Beq, "Beq");
//    printMat(Aineq, "Aineq");
//    printVec(Bineq, "Bineq");

    exit(1);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto core = std::make_shared<Core>();

    rclcpp::spin(core);
    rclcpp::shutdown();
    return 0;
}

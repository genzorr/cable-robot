#include "cable_robot/core.h"

Core::Core()
: Node("core"), mass(0.), a(6), v(6), vd(6)
{
    RCLCPP_INFO(this->get_logger(), "Initializing CDPR Core");
    // Load params
    mass = this->declare_parameter<double>("platform.mass", 0.);
    fMax = this->declare_parameter<double>("joints.actuated.effort", 0.);
    fMin = this->declare_parameter<double>("joints.actuated.min", 0.);

    controlUpdateInterval = this->declare_parameter<double>("controlUpdateInterval", 0.);
    controlUpdateTimer = this->create_wall_timer(
            std::chrono::microseconds(int(controlUpdateInterval*1000000)), [this] { updateCallback(); });

    // inertia matrix
    auto inertiaVec = this->declare_parameter<std::vector<double>>("platform.inertia");
    Ir.resize(3, 3);
    for(int i = 0; i < 3; ++i)
    {
        Ir[i][i] = inertiaVec[i];
    }
    Ir[0][1] = Ir[1][0] = inertiaVec[3];
    Ir[0][2] = Ir[2][0] = inertiaVec[4];
    Ir[2][1] = Ir[1][2] = inertiaVec[5];

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
    vpRxyzVector r(rpy[0], rpy[1], rpy[2]);
    Td.insert(vpRotationMatrix(r));
    Td.insert(vpTranslationVector(xyz[0], xyz[1], xyz[2]));

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

    tdaSolver = std::make_unique<TDA>(mass, nCables, fMin, fMax, TDA::closed_form);

    RCLCPP_INFO(this->get_logger(), "CDPR Core initialized");
}

void Core::computeW(vpMatrix &W)
{
    // build W matrix depending on current attach points
    vpTranslationVector t;  T.extract(t);
    vpRotationMatrix R;     T.extract(R);

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
        u = framePoints[i] - t - R*platformPoints[i]; // l = a - c - Rb;
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

void Core::computeDesiredW(vpMatrix &Wd)
{
    // build W matrix depending on current attach points
    vpTranslationVector td;  Td.extract(td);
    vpRotationMatrix Rd;     Td.extract(Rd);

    vpTranslationVector fd, P_p;
    vpColVector wd;
    for (int i = 0; i < nCables; ++i)
    {
        // vector between platform point and frame point in platform frame
        fd = Rd.t() * (framePoints[i] - td) - platformPoints[i];
        fd /= fd.frobeniusNorm();
        //fd=Rd*fd;
        //P_p = Rd*platformPoints[i];
        // corresponding force in platform frame
        wd = platformPoints[i].skew() * fd;
        for (unsigned int k = 0; k < 3; ++k)
        {
            Wd[k][i] = fd[k];
            Wd[k+3][i] = wd[k];
        }
    }
}

void Core::computeLength(vpColVector &L)
{
    // build W matrix depending on current attach points
    vpTranslationVector t;  T.extract(t);
    vpRotationMatrix R;     T.extract(R);

    vpTranslationVector f;
    for (int i=0; i < nCables; ++i)
    {
        // vector between platform point and frame point in platform frame
        f = R.t() * (framePoints[i] - t) - platformPoints[i];
        f=R*f;
        //L[i]= sqrt(f.sumSquare());
        L[i] = sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]) ;
    }
}

void Core::computeDesiredLength(vpColVector &Ld)
{
    // build W matrix depending on current attach points
    vpTranslationVector td;  Td.extract(td);
    vpRotationMatrix Rd;     Td.extract(Rd);

    vpTranslationVector fd;
    for (int i = 0; i < nCables; ++i)
    {
        // vector between platform point and frame point in platform frame
        fd = Rd.t() * (framePoints[i] - td) - platformPoints[i];
        fd = Rd*fd;
        //Ld[i]= sqrt(fd.sumSquare());
        Ld[i] = sqrt(fd[0]*fd[0] + fd[1]*fd[1] + fd[2]*fd[2]);
    }
}

void Core::sendTensions(vpColVector &f)
{
    // write effort to jointstate
    for (int i = 0; i < nCables; ++i)
        tensionsMsg.effort[i] = f[i];
    tensionsMsg.header.stamp = this->now();
    tensionsPub->publish(tensionsMsg);
}

void Core::updateCallback()
{
    auto dt = controlUpdateInterval;

    // current position
    vpHomogeneousMatrix Tcur;
    this->getPose(Tcur);
    vpRotationMatrix R;
    Tcur.extract(R);

//    vpColVector g(6), err, err_i(6), err0(6), v(6), d_err(6), w(6);
//    g[2] = - mass * 9.81;
//    vpMatrix R_R(6, 6), W(6, nCables);
//
//    // position error in platform frame
//    err = this->getPoseError();
//    for (unsigned int i = 0; i < 3; ++i)
//        for (unsigned int j = 0; j < 3; ++j)
//            R_R[i][j] = R_R[i+3][j+3] = R[i][j];
//
//    // position error in fixed frame
//    err = R_R * err;
//    // I term to wrench in fixed frame
//    for (unsigned int i = 0; i < 6; ++i)
//        if (w[i] < mass * 9.81)
//            err_i[i] += err[i] * dt;
//
//    double Kp = 5., Ki = 0.1; //, Kd = 3;
//    w = Kp * (err + Ki*err_i);
//
////    // D term
////    if (err0.infinityNorm())
////    {
////        // compute and filter error derivative
////        d_err = (err - err0)/dt;
////        filter.Filter(d_err);
////
////        w += Kp * Kd * d_err;
////    }
//
//    err0 = err;
//
//    // remove gravity + to platform frame
//    w = R_R.t() * (w-g);
//
//    vpColVector g(6), err, err_i(6), err0(6), v(6), d_err(6), w(6);
//    vpMatrix R_R(6, 6), W(6, nCables);
    vpColVector g(6), w(6), err(6);
    g[2] = - mass * 9.81;
    vpMatrix W(6, nCables);

//    w = -(M*(Kp*err + Kd*(vd - v)) + g);
//    w = -g;
    vpColVector omega(3); omega[0] = a[3]; omega[1] = a[4]; omega[2] = a[5];
    vpColVector Cor(6); Cor[0] = Cor[1] = Cor[2] = 0;
    vpMatrix Ig = R*Ir*R.t();
    vpColVector Colpart = vpColVector::skew(omega) * Ig * omega;
    Cor[3] = Colpart[0]; Cor[4] = Colpart[1]; Cor[5] = Colpart[2];

    vpMatrix M(6, 6);
    for (int i = 0; i < 3; i++)
    {
        M[i][i] = mass;
        M[i+3][i+3] = Ig[i][i];
    }
    M[0+3][1+3] = M[1+3][0+3] = Ig[0][1];
    M[0+3][2+3] = M[2+3][0+3] = Ig[0][2];
    M[1+3][2+3] = M[2+3][1+3] = Ig[1][2];
//    w = M*a + Cor - g;

    err = this->getPoseError();
//    vpMatrix R_R(6, 6);               // ?????????
//    for(unsigned int i=0;i<3;++i)
//        for(unsigned int j=0;j<3;++j)
//            R_R[i][j] = R_R[i+3][j+3] = R[i][j];
//    // position error in fixed frame
//    err = R_R * err;

    double Kp = 5., Kd = 3;//, Ki = 0.1
    w = M*(Kp*err + Kd*(vd - v)) - g;

    // build W matrix depending on current attach points
    this->computeW(W);
    // call TDA solver
    auto tau = tdaSolver->ComputeDistribution(W, w);

    // send tensions
    this->sendTensions(tau);
//    this->sendRealPosition();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto core = std::make_shared<Core>();

    rclcpp::spin(core);
    rclcpp::shutdown();
    return 0;
}

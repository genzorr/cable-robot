#include "cable_robot/core.h"

Core::Core()
: Node("core"), mass(0.)
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
    inertia.resize(3, 3);
    for(int i = 0; i < 3; ++i)
        inertia[i][i] = inertiaVec[i];
    inertia[0][1] = inertia[1][0] = inertiaVec[3];
    inertia[0][2] = inertia[2][0] = inertiaVec[4];
    inertia[2][1] = inertia[1][2] = inertiaVec[5];

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
    Md.insert(vpRotationMatrix(r));
    Md.insert(vpTranslationVector(xyz[0], xyz[1], xyz[2]));

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
    tensionsPub = this->create_publisher<sensor_msgs::msg::JointState>("tensions", 5);
    realPositionPub = this->create_publisher<geometry_msgs::msg::Point>("real_position", 5);
    desiredPositionSub = this->create_subscription<geometry_msgs::msg::Point>(
            "desired_position", 5, std::bind(&Core::desiredPositionCallback, this, std::placeholders::_1));

    tdaSolver = std::make_unique<TDA>(mass, nCables, fMin, fMax, TDA::noMin);

    RCLCPP_INFO(this->get_logger(), "CDPR Core initialized");
}

void Core::computeW(vpMatrix &W)
{
    // build W matrix depending on current attach points
    vpTranslationVector T;  M.extract(T);
    vpRotationMatrix R;     M.extract(R);

    vpTranslationVector f;
    vpColVector w;
    for (int i = 0; i < nCables; ++i)
    {
        // vector between platform point and frame point in platform frame
        f = R.t() * (framePoints[i] - T) - platformPoints[i];
        f /= f.frobeniusNorm();
        // corresponding force in platform frame
        w = platformPoints[i].skew() * f;
        for (unsigned int k = 0; k < 3; ++k)
        {
            W[k][i] = f[k];
            W[k+3][i] = w[k];
        }
    }
}

void Core::computeDesiredW(vpMatrix &Wd)
{
    // build W matrix depending on current attach points
    vpTranslationVector Td;  Md.extract(Td);
    vpRotationMatrix Rd;     Md.extract(Rd);

    vpTranslationVector fd, P_p;
    vpColVector wd;
    for (int i = 0; i < nCables; ++i)
    {
        // vector between platform point and frame point in platform frame
        fd = Rd.t() * (framePoints[i] - Td) - platformPoints[i];
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
    vpTranslationVector T;  M.extract(T);
    vpRotationMatrix R;     M.extract(R);

    vpTranslationVector f;
    for (int i=0; i < nCables; ++i)
    {
        // vector between platform point and frame point in platform frame
        f = R.t() * (framePoints[i] - T) - platformPoints[i];
        f=R*f;
        //L[i]= sqrt(f.sumSquare());
        L[i] = sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]) ;
    }
}

void Core::computeDesiredLength(vpColVector &Ld)
{
    // build W matrix depending on current attach points
    vpTranslationVector Td;  Md.extract(Td);
    vpRotationMatrix Rd;     Md.extract(Rd);

    vpTranslationVector fd;
    for (int i = 0; i < nCables; ++i)
    {
        // vector between platform point and frame point in platform frame
        fd = Rd.t() * (framePoints[i] - Td) - platformPoints[i];
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

void Core::sendRealPosition()
{
    vpTranslationVector t;
    M.extract(t);
    geometry_msgs::msg::Point position;
    position.x = t[0];
    position.y = t[1];
    position.z = t[2];
    realPositionPub->publish(position);
}

void Core::updateCallback()
{
    auto dt = controlUpdateInterval;

    // current position
    this->getPose(Mupd);
    vpRotationMatrix R;
    Mupd.extract(R);

    vpColVector g(6), err, err_i(6), err0(6), v(6), d_err(6), w(6);
    g[2] = - mass * 9.81;
    vpMatrix R_R(6, 6), W(6, nCables);

    // position error in platform frame
    err = this->getPoseError();
    for (unsigned int i = 0; i < 3; ++i)
        for (unsigned int j = 0; j < 3; ++j)
            R_R[i][j] = R_R[i+3][j+3] = R[i][j];

    // position error in fixed frame
    err = R_R * err;
    // I term to wrench in fixed frame
    for (unsigned int i = 0; i < 6; ++i)
        if (w[i] < mass * 9.81)
            err_i[i] += err[i] * dt;

    double Kp = 5., Ki = 0.1; //, Kd = 3;
    w = Kp * (err + Ki*err_i);

//    // D term
//    if (err0.infinityNorm())
//    {
//        // compute and filter error derivative
//        d_err = (err - err0)/dt;
//        filter.Filter(d_err);
//
//        w += Kp * Kd * d_err;
//    }

    err0 = err;

    // remove gravity + to platform frame
    w = R_R.t() * (w-g);
    // build W matrix depending on current attach points
    this->computeW(W);
    // call TDA solver
    auto tau = tdaSolver->ComputeDistribution(W, w);

    // send tensions
    this->sendTensions(tau);
    this->sendRealPosition();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto core = std::make_shared<Core>();

    rclcpp::spin(core);
    rclcpp::shutdown();
    return 0;
}

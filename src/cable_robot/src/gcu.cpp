#include <chrono>
#include <functional>
#include <iostream>
#include "cable_robot/gcu.h"

GCU::GCU()
: Node("gcu"), t_(0.), t_prev_(0.), x_(0.), y_(0.), z_(0.),
areaSideX_(0.), areaSideY_(0.), areaSideZ_(0.),
platformSide_(0.), maxSpeed_(0.), mass_(0.),
xSpeed_(0.), ySpeed_(0.), zSpeed_(0.),
controlUpdateInterval_(0.), debugInterval_(0.),
cables_ok(false), platform_ok(false), trajectory_ok(false),
tda(0., 0, 0., 0., TDA::noMin)
{
    RCLCPP_INFO(this->get_logger(), "Initializing General Control Unit");
    // Load params
    {
        x_              = this->declare_parameter<double>("cdpr.x", 0.0);
        y_              = this->declare_parameter<double>("cdpr.y", 0.0);
        z_              = this->declare_parameter<double>("cdpr.z", 0.0);
        areaSideX_      = this->declare_parameter<double>("cdpr.areaSideX", 0.);
        areaSideY_      = this->declare_parameter<double>("cdpr.areaSideY", 0.);
        areaSideZ_      = this->declare_parameter<double>("cdpr.areaSideZ", 0.);
        platformSide_   = this->declare_parameter<double>("cdpr.platformSide", 0.);
        maxSpeed_       = this->declare_parameter<double>("cdpr.maxSpeed", 0.);
        mass_           = this->declare_parameter<double>("cdpr.platform.mass", 0.);
        fMax            = this->declare_parameter<double>("cdpr.joints.actuated.effort", 0.);
        fMin            = this->declare_parameter<double>("cdpr.joints.actuated.min", 0.);

        debugInterval_          = this->declare_parameter<double>("gcu.debugInterval", 0.);
        controlUpdateInterval_  = this->declare_parameter<double>("gcu.controlUpdateInterval", 0.);

        // inertia matrix
        auto inertiaVec = this->declare_parameter<std::vector<double>>("cdpr.platform.inertia");
        inertia_.resize(3,3);
        for(unsigned int i = 0; i < 3; ++i)
            inertia_[i][i] = inertiaVec[i];
        inertia_[0][1] = inertia_[1][0] = inertiaVec[3];
        inertia_[0][2] = inertia_[2][0] = inertiaVec[4];
        inertia_[2][1] = inertia_[1][2] = inertiaVec[5];

        // cable attach points
        std::vector<double> point;
        n_cable = this->declare_parameter<int>("cdpr.points.nCables", 4);
        for (unsigned int i = 0; i < n_cable;++i)
        {
            point = this->declare_parameter<std::vector<double>>("cdpr.points.frame" + std::to_string(i));
            Pf.emplace_back(point);
            point = this->declare_parameter<std::vector<double>>("cdpr.points.platform" + std::to_string(i));
            Pp.emplace_back(point);
        }

        // initial desired pose = home
        auto xyz = this->declare_parameter<std::vector<double>>("cdpr.platform.position.xyz");
        auto rpy = this->declare_parameter<std::vector<double>>("cdpr.platform.position.rpy");
        vpRxyzVector r(rpy[0], rpy[1], rpy[2]);
        Md_.insert(vpRotationMatrix(r));
        Md_.insert(vpTranslationVector(xyz[0], xyz[1], xyz[2]));

        char cable_name[256];
        for (unsigned int i = 0; i < n_cable; ++i)
        {
            sprintf(cable_name, "cable%i", i);
            tensions_msg.name.push_back(std::string(cable_name));
        }
        tensions_msg.effort.resize(n_cable);

        debugTimer_ = this->create_wall_timer(
                std::chrono::milliseconds(int(debugInterval_*1000)), [this] { debugCallback(); });
        controlUpdateTimer_ = this->create_wall_timer(
                std::chrono::microseconds(int(controlUpdateInterval_*1000000)), [this] { updateCallback(); });

        // Subs
        controlSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 5,
                std::bind(&GCU::controlsUpdateCallback, this, std::placeholders::_1)
        );

        // init listener to platform state
        platform_sub = this->create_subscription<gazebo_msgs::msg::LinkState>(
                "platform_state", 5,
                std::bind(&GCU::PFState_cb, this, std::placeholders::_1)
        );

        // init listener to pose setpoint
        setpoint_sub = this->create_subscription<geometry_msgs::msg::Pose>(
                "pf_setpoint", 5,
                std::bind(&GCU::Setpoint_cb, this, std::placeholders::_1)
        );

        desiredVel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "desired_vel", 5,
                std::bind(&GCU::DesiredVel_cb, this, std::placeholders::_1)
        );

        desiredAcc_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "desired_acc", 5,
                std::bind(&GCU::DesiredAcc_cb, this, std::placeholders::_1)
        );

        // init listener to cable states
        cables_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                "cable_joints_state", 5,
                std::bind(&GCU::Cables_cb, this, std::placeholders::_1)
        );

        // publisher to cable tensions
        tensions_pub = this->create_publisher<sensor_msgs::msg::JointState>("cable_control", 5);
    }

    tda = TDA(mass_, (int)n_cable, fMin, fMax, TDA::noMin);

    RCLCPP_INFO(this->get_logger(), "General Control Unit initialized");
}

void GCU::controlsUpdateCallback(const geometry_msgs::msg::Twist::ConstSharedPtr vel)
{
    // Update and restrain speed
    if (vel->linear.x <= maxSpeed_)
        xSpeed_ = vel->linear.x;
    else
        RCLCPP_WARN(this->get_logger(), "Reached limit for x speed");
    if (vel->linear.y <= maxSpeed_)
        ySpeed_ = vel->linear.y;
    else
        RCLCPP_WARN(this->get_logger(), "Reached limit for y speed");
    if (vel->linear.z <= maxSpeed_)
        zSpeed_ = vel->linear.z;
    else
        RCLCPP_WARN(this->get_logger(), "Reached limit for z speed");
}

void updateCoordinate(double &x, double &dx, double min, double max)
{
    if (x + dx < min)
        x = min;
    else if (x + dx > max)
        x = max;
    else
        x += dx;
}

void GCU::debugCallback()
{
    RCLCPP_DEBUG(this->get_logger(), "x: %.4f, y: %.4f, z: %.4f", x_, y_, z_);
//    RCLCPP_DEBUG(this->get_logger(), "dt: %.15f, t: %.15f, t_prev: %.15f", t_ - t_prev_, t_, t_prev_);
}

void GCU::updateCallback()
{
//    t_prev_ = t_;
//    t_ = (double)this->now().nanoseconds() / 1e+9;
//    t_ = this->get_clock()->now().seconds();

//    // Update and restrain coordinate
    auto dt = controlUpdateInterval_;
//    double dx = xSpeed_ * dt, dy = ySpeed_ * dt, dz = zSpeed_ * dt;
//    updateCoordinate(x_, dx, -areaSideX_ / 2, areaSideX_ / 2);
//    updateCoordinate(y_, dy, -areaSideY_ / 2, areaSideY_ / 2);
//    updateCoordinate(z_, dz, 0, areaSideZ_);
//    this->setDesiredPose(x_, y_, z_, 0., 0., 0.);

    // current position
    this->getPose(M);
    vpRotationMatrix R;
    M.extract(R);

    vpColVector g(6), err, err_i(6), err0(6), v(6), d_err(6), w(6);
    g[2] = - mass_ * 9.81;
    vpMatrix R_R(6, 6), W(6, n_cable);

    // position error in platform frame
    err = this->getPoseError();
    //    cout << "Position error in platform frame: " << err.t() << fixed << endl;
    for(unsigned int i=0;i<3;++i)
        for(unsigned int j=0;j<3;++j)
            R_R[i][j] = R_R[i+3][j+3] = R[i][j];

    // position error in fixed frame
    err = R_R * err;
    //  cout << "Position error in world frame: " << err.t() << fixed << endl;
    //robot.sendError(err);
    // I term to wrench in fixed frame
    for (unsigned int i = 0; i < 6; ++i)
        if (w[i] < mass_ * 9.81)
            err_i[i] += err[i] * dt;

    double Kp = 1., Ki = 0.1; // Kd = 3;
    w = Kp * (err + Ki*err_i);

    // D term?
//    if (err0.infinityNorm())
//    {
//        // compute and filter error derivative
//        d_err =(err - err0)/dt;
//        filter.Filter(d_err);
//
//        w += Kp * Kd * d_err;
//    }

    err0 = err;

    //     cout << "Desired wrench in fixed frame: " << w.t() << fixed << endl;
    // remove gravity + to platform frame
    w = R_R.t() * (w-g);
    //   cout << "Desired wrench in platform frame: " << w.t() << fixed << endl;

    // build W matrix depending on current attach points
    this->computeW(W);

    // call cable tension distribution
    auto tau = tda.ComputeDistribution(W, w);
//    RCLCPP_INFO(this->get_logger(), "%d", tau.size());

    //    cout << "sending tensions: " << tau.t() << endl;

    // send tensions
    this->sendTensions(tau);
//    RCLCPP_INFO(this->get_logger(), "%f %f %f %f", tau[0], tau[1], tau[2], tau[3]);

//    auto end = std::chrono::system_clock::now();
//    auto elapsed_seconds = end - start;
    // log
//    M_.buildFrom(this->getPoseError());
//    pose_err.buildFrom(M.inverse());
//    comp_time[0] = elapsed_seconds.count();
//    residual = W*tau - w;
}

void GCU::computeW(vpMatrix &W)
{
    // build W matrix depending on current attach points
    vpTranslationVector T;  M_.extract(T);
    vpRotationMatrix R;     M_.extract(R);

    vpTranslationVector f;
    vpColVector w;
    for (unsigned int i = 0; i < n_cable; ++i)
    {
        // vector between platform point and frame point in platform frame
        f = R.t() * (Pf[i] - T) - Pp[i];
        f /= f.frobeniusNorm();
        // corresponding force in platform frame
        w = Pp[i].skew() * f;
        for (unsigned int k = 0; k < 3; ++k)
        {
            W[k][i] = f[k];
            W[k+3][i] = w[k];
        }
    }
}

void GCU::computeDesiredW(vpMatrix &Wd)
{
    // build W matrix depending on current attach points
    vpTranslationVector Td;  Md_.extract(Td);
    vpRotationMatrix Rd;     Md_.extract(Rd);

    vpTranslationVector fd, P_p;
    vpColVector wd;
    for (unsigned int i = 0; i < n_cable; ++i)
    {
        // vector between platform point and frame point in platform frame
        fd = Rd.t() * (Pf[i] - Td) - Pp[i];
        fd /= fd.frobeniusNorm();
        //fd=Rd*fd;
        //P_p = Rd*Pp[i];
        // corresponding force in platform frame
        wd = Pp[i].skew() * fd;
        for (unsigned int k = 0; k < 3; ++k)
        {
            Wd[k][i] = fd[k];
            Wd[k+3][i] = wd[k];
        }
    }
}

void GCU::computeLength(vpColVector &L)
{
    // build W matrix depending on current attach points
    vpTranslationVector T;  M_.extract(T);
    vpRotationMatrix R;     M_.extract(R);

    vpTranslationVector f;
    for (unsigned int i=0;i<n_cable;++i)
    {
        // vector between platform point and frame point in platform frame
        f = R.t() * (Pf[i] - T) - Pp[i];
        f=R*f;
        //L[i]= sqrt(f.sumSquare());
        L[i] = sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]) ;
    }
}

void GCU::computeDesiredLength(vpColVector &Ld)
{
    // build W matrix depending on current attach points
    vpTranslationVector Td;  Md_.extract(Td);
    vpRotationMatrix Rd;     Md_.extract(Rd);

    vpTranslationVector fd;
    for (unsigned int i = 0; i < n_cable; ++i)
    {
        // vector between platform point and frame point in platform frame
        fd = Rd.t() * (Pf[i] - Td) - Pp[i];
        fd = Rd*fd;
        //Ld[i]= sqrt(fd.sumSquare());
        Ld[i] = sqrt(fd[0]*fd[0] + fd[1]*fd[1] + fd[2]*fd[2]);
    }
}

void GCU::sendTensions(vpColVector &f)
{
    // write effort to jointstate
    for (unsigned int i = 0; i < n_cable; ++i)
        tensions_msg.effort[i] = f[i];
    tensions_msg.header.stamp = this->now();
    tensions_pub->publish(tensions_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto gcu = std::make_shared<GCU>();
    gcu->setDesiredPose(1, -1, 3.5, 0, 0, 0);

    rclcpp::spin(gcu);
    rclcpp::shutdown();
    return 0;
}

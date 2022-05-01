#include <chrono>
#include <functional>
#include <iostream>
#include "cable_robot/gcu.h"

GCU::GCU()
: Node("gcu"), x_(0.), y_(0.), z_(0.),
areaSideX_(0.), areaSideY_(0.), areaSideZ_(0.),
platformSide_(0.), maxSpeed_(0.), mass_(0.),
xSpeed_(0.), ySpeed_(0.), zSpeed_(0.),
controlUpdateInterval_(0.), debugInterval_(0.),
cables_ok(false), platform_ok(false), trajectory_ok(false)
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

//        for (auto &vec: Pf)
//            for (size_t i = 0; i < vec.size(); i++)
//                std::cout << vec[i] << " ";
//        std::cout << std::endl;
//
//        for (auto &vec: Pp)
//            for (size_t i = 0; i < vec.size(); i++)
//                std::cout << vec[i] << " ";
//        std::cout << std::endl;


        // initial desired pose = home
        auto xyz = this->declare_parameter<std::vector<double>>("cdpr.platform.position.xyz");
        auto rpy = this->declare_parameter<std::vector<double>>("cdpr.platform.position.rpy");
        vpRxyzVector r(rpy[0], rpy[1], rpy[2]);
        Md_.insert(vpRotationMatrix(r));
        Md_.insert(vpTranslationVector(xyz[0], xyz[1], xyz[2]));

        char cable_name[256];
        for(unsigned int i=0;i<n_cable;++i)
        {
            sprintf(cable_name, "cable%i", i);
            tensions_msg.name.push_back(std::string(cable_name));
        }
        tensions_msg.effort.resize(n_cable);
    }

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
        "pf_state", 1,
        std::bind(&GCU::PFState_cb, this, std::placeholders::_1)
    );

    // init listener to pose setpoint
    setpoint_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        "pf_setpoint", 1,
        std::bind(&GCU::Setpoint_cb, this, std::placeholders::_1)
    );

    desiredVel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "desired_vel", 1,
        std::bind(&GCU::DesiredVel_cb, this, std::placeholders::_1)
    );

    desiredAcc_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "desired_acc", 1,
        std::bind(&GCU::DesiredAcc_cb, this, std::placeholders::_1)
    );

    // init listener to cable states
    cables_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "cable_states", 1,
        std::bind(&GCU::Cables_cb, this, std::placeholders::_1)
    );

    // publisher to cable tensions
    tensions_pub = this->create_publisher<sensor_msgs::msg::JointState>("cable_command", 1);

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

void restrainSpeed(double &x, double &dx, double &restrain)
{
    if (x + dx < 0.)
        x = 0.;
    else if (x + dx > restrain)
        x = restrain;
    else
        x += dx;
}

void GCU::debugCallback()
{
    RCLCPP_DEBUG(this->get_logger(), "x: %.4f, y: %.4f, z: %.4f", x_, y_, z_);
}

void GCU::updateCallback()
{
    // Update and restrain coordinate
    auto dt = controlUpdateInterval_;
    double dx = xSpeed_ * dt, dy = ySpeed_ * dt, dz = zSpeed_ * dt;
    restrainSpeed(x_, dx, areaSideX_);
    restrainSpeed(y_, dy, areaSideY_);
    restrainSpeed(z_, dz, areaSideZ_);
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
//    tensions_msg.header.stamp = rosnode_->get_clock()->now().seconds();

    tensions_pub->publish(tensions_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto gcu = std::make_shared<GCU>();
    rclcpp::spin(gcu);
    rclcpp::shutdown();
    return 0;
}

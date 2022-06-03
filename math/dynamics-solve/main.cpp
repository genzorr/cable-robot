#include <iostream>
#include <armadillo>
#include <vector>

typedef std::vector<arma::vec3> points;
const int m_num = 4;

void getRotMat(arma::mat33 &R, arma::vec3 &phi)
{
    // 0 - phi, 1 - theta, 2 - psi
    R(0, 0) =   cos(phi[1])*cos(phi[2]);
    R(0, 1) = - cos(phi[1])*sin(phi[2]);
    R(0, 2) =   sin(phi[1]);
    R(1, 0) =   cos(phi[0])*sin(phi[2]) + sin(phi[0])*sin(phi[1])*cos(phi[2]);
    R(1, 1) =   cos(phi[0])*cos(phi[2]) - sin(phi[0])*sin(phi[1])*sin(phi[2]);
    R(1, 2) = - sin(phi[0])*cos(phi[1]);
    R(2, 0) =   sin(phi[0])*sin(phi[2]) - cos(phi[0])*sin(phi[1])*cos(phi[2]);
    R(2, 1) =   sin(phi[0])*cos(phi[2]) + cos(phi[0])*sin(phi[1])*sin(phi[2]);
    R(2, 2) =   cos(phi[0])*cos(phi[1]);
}

void getNormLengths(points &u, points &a, points &b, arma::vec3 &r, arma::mat33 &R)
{
    for (int i = 0; i < m_num; i++)
    {
        arma::vec3 length = a[i] - r - R*b[i];
        u[i] = length / arma::norm(length);
    }
}

void getWrenchMat(arma::mat &W, points &a, points &b, arma::vec3 &r, arma::mat33 &R)
{
    points u(m_num);
    getNormLengths(u, a, b, r, R);

    for (int i = 0; i < m_num; i++)
    {
        arma::vec3 bu = arma::cross(b[i], u[i]);
        for (int k = 0; k < 3; k++)
        {
            W(k, i) = u[i][k];
            W(k+3, i) = bu[k];
        }
//        W(3, i) = bu[0];
    }
}

void searchTensions(points &a, points &b, arma::vec3 &r, arma::vec6 &wp, const int *starts, const int *ends, double step)
{
    double eps = 1e-0;
    double bestNorm = 10000000;
    arma::vec4 bestTensions;
    arma::vec3 bestPhiHat;
    arma::mat bestW;
    bestW.reshape(6, 4);

    arma::vec4 tensions;
    arma::mat33 R;
    arma::mat W;
    W.reshape(6, 4);
    double tmin = 1;
    double tmax = 100;

    arma::vec3 phiHat;
    for (int phi_i = starts[0]; phi_i <= ends[0]; phi_i++)
    {
        std::cout << phi_i << " " << phi_i*step*M_PI/180 << std::endl;
        phiHat(0) = phi_i*step*M_PI/180;

        for (int theta_i = starts[1]; theta_i <= ends[1]; theta_i++)
        {
            phiHat(1) = theta_i*step*M_PI/180;

            for (int psi_i = starts[2]; psi_i < ends[2]; psi_i++)
            {
                phiHat(2) = psi_i*step*M_PI/180;

                getRotMat(R, phiHat);
                getWrenchMat(W, a, b, r, R);
                tensions = arma::pinv(W) * wp;

                bool cond = true;
                for (arma::uword j = 0; j < tensions.size(); j++)
                {
                    if (tensions(j) < tmin or tensions(j) > tmax)
                    {
                        cond = false;
                        break;
                    }
                }

                arma::vec zero = W * tensions - wp;
//                if (cond)
                if (cond and (arma::norm(zero) < eps))
                {
                    double norm = arma::norm(tensions);
                    if (norm < bestNorm)
                    {
                        bestNorm = norm;
                        bestPhiHat = phiHat;
                        bestTensions = tensions;
                        bestW = W;

//                        phiHat.print(std::cout, "phi hat");
//                        arma::vec res = bestW * bestTensions - wp;
//                        std::cout << "res norm: " << arma::norm(res) << std::endl;
//                        res.print(std::cout, "res");
                    }
                }
            }
        }
    }

    std::cout << "Results:\n" << "Norm: " << bestNorm << "\n";
    bestPhiHat.print(std::cout, "Phi");
    bestTensions.print(std::cout, "Tensions");

    arma::vec res = bestW * bestTensions - wp;
    res.print(std::cout, "res");
    std::cout << "res norm: " << arma::norm(res) << std::endl;
}

int main()
{
    double m = 4;
//    arma::vec4 fg = {0, 0, -m*9.81, 0};
//    arma::vec4 wp = -fg;
    arma::vec6 fg = {0, 0, -m*9.81, 0, 0, 0};
    arma::vec6 wp = -fg;
    points a = {{-2.0, -2.0, 4.0}, {-2.0, 2.0, 4.0}, {2.0, 2.0, 4.0}, {2.0, -2.0, 4.0}};
    points b = {{-0.2, -0.2, 0.0}, {-0.2, 0.2, 0.0}, {0.2, 0.2, 0.0}, {0.2, -0.2, 0.0}};

    arma::vec3 r = {0.2, 0.2, 2};

    double step = (double)1/20;
    double angle = 10;
    int starts[3] = {int((0 - angle)/step), int((0 - angle)/step), int((0 - angle)/step)};
    int ends[3] = {int((0 + angle)/step), int((0 + angle)/step), int((0 + angle)/step)};

    auto start_t = std::chrono::system_clock::now();
    searchTensions(a, b, r, wp, starts, ends, step);
    std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - start_t;
    std::cout << "Elapsed: " << elapsed.count() << std::endl;

//    arma::vec3 phi = {0, 0, 0};
//    arma::mat33 R;
//    getRotMat(R, phi);
//    R.print(std::cout, "R");
//
//    arma::mat W;
//    W.reshape(6, 4);
//    getWrenchMat(W, a, b, r, R);
//    W.print(std::cout, "W");
//
//    std::cout << arma::rank(W) << std::endl;
//
//    arma::vec4 t = arma::pinv(W) * wp;
//    t.print(std::cout, "tensions");

    return 0;
}

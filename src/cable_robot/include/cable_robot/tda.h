#ifndef TDA_H
#define TDA_H

#include <cmath>
#include <algorithm>
#include <rclcpp/node.hpp>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpSubColVector.h>
#include <visp/vpSubMatrix.h>

void solveQP(const vpMatrix &_Q, const vpColVector _r,
              vpMatrix _A, vpColVector _b,
              const vpMatrix &_C, const vpColVector &_d,
              vpColVector &_x, std::vector<bool> &active, rclcpp::Logger logger);

#endif // TDA_H

#include "Pendulum.hpp"
#include <cmath>

DoublePendulum::DoublePendulum(double m1_, double m2_, double L1_, double L2_, double theta1_, double theta2_, double _c1, double _c2)
    : m1(m1_), m2(m2_), L1(L1_), L2(L2_), theta1(theta1_), theta2(theta2_), c1(_c1), c2(_c2),
      omega1(0.0), omega2(0.0), g(9.81) {}

void DoublePendulum::computeAcceleration(double t1, double t2, double w1, double w2,
                                         double &a1, double &a2) const
{
    double delta = t1 - t2;
    double M11 = (m1 + m2) * L1 * L1;
    double M12 = m2 * L1 * L2 * std::cos(delta); // M12 = M21
    double M22 = m2 * L2 * L2;
    double G1 = (m1 + m2) * g * L1 * std::sin(t1);
    double G2 = m2 * g * L2 * std::sin(t2);
    double Q1 = m2 * L1 * L2 * std::sin(delta) * w2 * w2;
    double Q2 = -m2 * L1 * L2 * std::sin(delta) * w1 * w1;
    double D1 = (c1 + c2) * w1 - c2 * w2;
    double D2 = c2 * w2 - c2 * w1;

    a1 = (M12 * (Q2 + G2 + D2) - M22 * (Q1 + G1 + D1)) / (M11 * M22 - M12 * M12);
    a2 = (M12 * (Q1 + G1 + D1) - M11 * (Q2 + G2 + D2)) / (M11 * M22 - M12 * M12);
}

void DoublePendulum::step(double dt)
{
    double k1_t1, k1_t2, k1_w1, k1_w2;
    double k2_t1, k2_t2, k2_w1, k2_w2;
    double k3_t1, k3_t2, k3_w1, k3_w2;
    double k4_t1, k4_t2, k4_w1, k4_w2;

    double a1, a2;

    // k1
    computeAcceleration(theta1, theta2, omega1, omega2, a1, a2);
    k1_t1 = omega1;
    k1_t2 = omega2;
    k1_w1 = a1;
    k1_w2 = a2;

    // k2
    computeAcceleration(theta1 + 0.5 * k1_t1 * dt, theta2 + 0.5 * k1_t2 * dt,
                        omega1 + 0.5 * k1_w1 * dt, omega2 + 0.5 * k1_w2 * dt, a1, a2);
    k2_t1 = omega1 + 0.5 * k1_w1 * dt;
    k2_t2 = omega2 + 0.5 * k1_w2 * dt;
    k2_w1 = a1;
    k2_w2 = a2;

    // k3
    computeAcceleration(theta1 + 0.5 * k2_t1 * dt, theta2 + 0.5 * k2_t2 * dt,
                        omega1 + 0.5 * k2_w1 * dt, omega2 + 0.5 * k2_w2 * dt, a1, a2);
    k3_t1 = omega1 + 0.5 * k2_w1 * dt;
    k3_t2 = omega2 + 0.5 * k2_w2 * dt;
    k3_w1 = a1;
    k3_w2 = a2;

    // k4
    computeAcceleration(theta1 + k3_t1 * dt, theta2 + k3_t2 * dt,
                        omega1 + k3_w1 * dt, omega2 + k3_w2 * dt, a1, a2);
    k4_t1 = omega1 + k3_w1 * dt;
    k4_t2 = omega2 + k3_w2 * dt;
    k4_w1 = a1;
    k4_w2 = a2;

    theta1 += (dt / 6.0) * (k1_t1 + 2 * k2_t1 + 2 * k3_t1 + k4_t1);
    theta2 += (dt / 6.0) * (k1_t2 + 2 * k2_t2 + 2 * k3_t2 + k4_t2);
    omega1 += (dt / 6.0) * (k1_w1 + 2 * k2_w1 + 2 * k3_w1 + k4_w1);
    omega2 += (dt / 6.0) * (k1_w2 + 2 * k2_w2 + 2 * k3_w2 + k4_w2);
}

std::pair<double, double> DoublePendulum::getBob1Position() const
{
    double x1 = L1 * std::sin(theta1);
    double y1 = -L1 * std::cos(theta1);
    return {x1, y1};
}

std::pair<double, double> DoublePendulum::getBob2Position() const
{
    auto [x1, y1] = getBob1Position();
    double x2 = x1 + L2 * std::sin(theta2);
    double y2 = y1 - L2 * std::cos(theta2);
    return {x2, y2};
}

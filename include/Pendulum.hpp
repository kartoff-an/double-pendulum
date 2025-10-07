#pragma once
#include <vector>

class DoublePendulum
{
public:
    DoublePendulum(double m1, double m2, double L1, double L2, double theta1, double theta2, double c1, double c2);
    void step(double dt);
    std::pair<double, double> getBob1Position() const;
    std::pair<double, double> getBob2Position() const;

private:
    void computeAcceleration(double t1, double t2, double w1, double w2, double &a1, double &a2) const;

    double m1, m2, L1, L2, g;
    double theta1, theta2;
    double c1, c2;
    double omega1, omega2;
};
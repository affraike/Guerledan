#include <cmath>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <unistd.h>
#include "test_kalman_command/Message_consigne.h"
/*
  Warning : penser à inclure ce type de message dans le CMakeListe
*/
using namespace std;
using namespace Eigen;

void lissajous(Vector2d& w, Vector2d& dw, Vector2d& ddw, double t, const double p, const double q, const double a, double b){
    // p = 1, q = 2, donne la courbe attendue. Multiplier p et q par un même facteur permet de gérer la vitesse de
    // parcours de la courbe.
    w << -20 + b * sin(q * t), a * sin(p * t);
    dw << b * q * cos(q * t), a * p * cos(p * t);
    ddw << -b * pow(q, 2) * sin(q * t), -a * pow(p, 2) * sin(p * t);
}

void waypoint(Vector2d& w, Vector2d& dw, Vector2d& ddw, double t, double x, double y){
    w << x, y;
    dw << 0, 0;
    ddw << 0, 0;
}

void rotating_ellipse(Vector2d& w, Vector2d& dw, Vector2d& ddw, double t, double a, double r, double rp, int i , int m){
    // mettre T = 1, sinon le bateau ne suit pas
    Vector2d c = {cos(a * t + 2 * i * M_PI / m), sin(a * t + 2 * i * M_PI / m) };
    Vector2d dc = {-a * sin(a * t + 2 * i * M_PI / m), a * cos(a * t + 2 * i * M_PI / m) };
    Vector2d ddc = {-pow(a, 2) * cos(a * t + 2 * i * M_PI / m), -pow(a, 2) * sin(a * t + 2 * i * M_PI / m) };
    Matrix2d R;
    R << cos(a * t), -sin(a * t), sin(a * t), cos(a * t);
    Matrix2d D;
    D << r + rp * sin(a * t), 0, 0, r;
    Matrix2d Rp;
    Rp << -a * sin(a * t), -a * cos(a * t), a * cos(a * t), -a * sin(a * t);
    Matrix2d Dp;
    Dp << a * rp * cos(a * t), 0, 0, 0;
    Matrix2d Rpp = -pow(a, 2) * R;
    Matrix2d Dpp;
    Dp << -pow(a, 2) * rp * sin(a * t), 0, 0, 0;

    w = R*D*c;
    dw = R * D * dc + R * Dp * c + Rp * D * c;
    ddw = R * D * ddc + R * Dpp * c + Rpp * D * c + 2 * (Rp * D * dc + R * Dp * dc + Rp * Dp * c);
}

void static_ellipse(Vector2d& w, Vector2d& dw, Vector2d& ddw, double t, double a, double r, double rp, int i , int m){
    Vector2d c = {cos(a * t + 2 * i * M_PI / m), sin(a * t + 2 * i * M_PI / m) };
    Vector2d dc = {-a * sin(a * t + 2 * i * M_PI / m), a * cos(a * t + 2 * i * M_PI / m) };
    Vector2d ddc = {-pow(a, 2) * cos(a * t + 2 * i * M_PI / m), -pow(a, 2) * sin(a * t + 2 * i * M_PI / m) };
    Matrix2d D;
    D << r + rp , 0, 0, r;

    w = D*c;
    dw = D * dc;
    ddw = D * ddc;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "command_bateau");
    ros::NodeHandle n;
    ros::Publisher cons = n.advertise<test_kalman_command::Message_consigne>("command", 1000);
    ros::Rate loop_rate(10.);
    double t = 0, dt = 0.1;
    const double p = 0.1,q = 0.2,a = 40,b = 20;
    Vector2d w, dw, ddw;


    while (ros::ok()){
        ros::spinOnce();
        waypoint(w, dw, ddw, t, 0., 0.);
        t += dt;
        test_kalman_command::Message_consigne msg_c;
        msg_c.w0 = w(0);
        msg_c.w1 = w(1);
        msg_c.dw0 = dw(0);
        msg_c.dw1 = dw(1);
        msg_c.ddw0 = ddw(0);
        msg_c.ddw1 = ddw(1);

        cons.publish(msg_c);

        loop_rate.sleep();
    }
    return 0;
}


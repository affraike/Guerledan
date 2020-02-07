#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "tf/tf.h"
#include <eigen3/Eigen/Dense>
#include <unistd.h>
#include "test_kalman_command/Message_consigne.h"
#include "test_kalman_command/custom_cmd_motors.h"
#include "gpsd_client/GnssPose.h"
/*
  Warning : penser à inclure ce type de message dans le CMakeListe
*/
using namespace std;
using namespace Eigen;

Vector2d w, dw, ddw;
Vector4d vecteur_etat;
const double T = 1; // constante de convergence
double err = 0;

Vector2d command(Vector4d& x, Vector2d& w, Vector2d& dw, Vector2d& ddw, const double T){
    Matrix2d A;
    A << cos(x[2]) -x[3] * sin(x[2]), cos(x[2]) + x[3] * sin(x[2]),
    sin(x[2]) + x[3] * cos(x[2]), sin(x[2]) - x[3] * cos(x[2]);
    Vector2d b = {-x[3] * abs(x[3]) * cos(x[2]), -x[3] * abs(x[3]) * sin(x[2])};
    Vector2d y = {x[0], x[1]};
    Vector2d dy = {x[3] * cos(x[2]), x[3] * sin(x[2])};
    Vector2d v = pow(1.0/T, 2) * (w - y) + (2.0/T) * (dw - dy) + ddw;
    Vector2d u = A.inverse() * (v - b);
    return u;
}

Vector2d regul(Vector4d& x , Vector2d& w){
    Vector2d u;
    double heading_boat = x(2) * M_PI / 180.0;
    double heading_waypoint = atan2(w(1) - x(1), w(0) - x(0));
    err = 2 * atan(tan((heading_boat - heading_waypoint)/2));
    ROS_INFO("boat :%f, waypoint:%f, err:%f", heading_boat, heading_waypoint, err);
    double Kp = 8, Kd = 1;
    double d = sqrt(pow(w(0) - x(0), 2) + pow(w(1) - x(1), 2));
    if (d > 2.){
        if (abs(err) <= M_PI /2.0){
            u = {130. - Kp * err, 130. + Kp * err};
        }else{
            u = {50., 45.};
        }
    }else{
        u = {0.1, 0.1};
    }

    return u;
}

void consignCallback(const test_kalman_command::Message_consigne& msg){
    w << msg.w0, msg.w1;
    dw << msg.dw0, msg.dw1;
    ddw << msg.ddw0, msg.ddw1;
}

void stateCallback(const gpsd_client::GnssPose::ConstPtr& msg){
    /*
      Permet de récupérer le vecteur d'état après Kalman
    */
    vecteur_etat(0) = msg->east;
    vecteur_etat(1) = msg->north;
    vecteur_etat(3) = msg->speed;
    //on met dans z la vitesse estimée du bateau
}
void capCallback(const std_msgs::Float64::ConstPtr& msg){
    vecteur_etat(2) = msg->data;
    vecteur_etat(2) -= 90.;
    if (vecteur_etat(2) < 0.){
        vecteur_etat(2) += 360.;
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "control_bateau");
    ros::NodeHandle n;
    ros::Publisher control = n.advertise<test_kalman_command::custom_cmd_motors>("u", 1000);
    ros::Subscriber sub = n.subscribe("command", 1000, consignCallback);
    ros::Subscriber sub2 = n.subscribe("poseRaw", 1000, stateCallback);
    ros::Subscriber sub3 = n.subscribe("cap", 1000, capCallback);
    ros::Rate loop_rate(5.);

    while (ros::ok()){
        ros::spinOnce();
        /*
          Partie commande
        */
        Vector2d u;
        double u1, u2;

        //u = command(vecteur_etat, w, dw, ddw, T);
        u = regul(vecteur_etat, w);
        u1 = u(0);
        u2 = u(1);
        /*
          Publication des commandes moteurs
        */
        test_kalman_command::custom_cmd_motors msg_control;
        msg_control.u1 = u1; //commande moteur 1
        msg_control.u2 = u2; //commande moteur 2
        control.publish(msg_control);

        loop_rate.sleep();
    }
    return 0;
}
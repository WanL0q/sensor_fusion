#include <iostream>
#include <random>
#include <cmath>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <GPS_Conversion/navsat_conversion.h>

#define RATE 50.0
#define DT 0.1
#define PI 3.141592653

double distance_gps2ori = 0.0;
double acc_offset = -0.03;

using namespace std;
using namespace Eigen;
using namespace NavsatConversions;

class EKF {
private:
    Matrix2f Q;
    Matrix3f R;
    double dt;
public:
    EKF(Matrix2f Q_, Matrix3f R_, double rate) : Q(Q_), R(R_), dt(1.0 / rate) {}
    
    // x_{t+1} = F@x_{t}+B@u_t
    Vector4f motion_model(Vector4f x, Vector2f u) {
        Matrix4f F_;
        F_ <<   1.0, 0.0, 0.0, cos(x(2)) * dt,
                0.0, 1.0, 0.0, sin(x(2)) * dt,
                0.0, 0.0, 1.0,            0.0,
                0.0, 0.0, 0.0,            1.0;

        Matrix<float, 4, 2> B_;
        B_ <<   0.0, 0.0,
                0.0, 0.0,
                0.0,  dt,
                 dt, 0.0;

        return F_ * x + B_ * u;
    };

    pair<Matrix4f, Matrix<float, 4, 2>> jacobF_B(Vector4f x, Vector2f u) {
        Matrix4f jF_ = Matrix4f::Identity();
        float yaw = x(2);
        float v = x(3);
        jF_ <<  1.0, 0.0, -dt * sin(yaw) * v, dt * cos(yaw),
                0.0, 1.0,  dt * cos(yaw) * v, dt * sin(yaw),
                0.0, 0.0,                1.0,           0.0,
                0.0, 0.0,                0.0,           1.0;

        Matrix<float, 4, 2> jB_;
        jB_ <<  0.0, 0.0,
                0.0, 0.0,
                0.0,  dt,
                 dt, 0.0;

        return make_pair(jF_, jB_);
    };

    // observation model H
    Vector3f observation_model(Vector4f x) {
        Matrix<float, 3, 4> H_;
        H_ <<   1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
        return H_ * x;
    };

    Matrix<float, 3, 4> jacobH() {
        Matrix<float, 3, 4> jH_;
        jH_ <<  1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
        return jH_;
    };

    pair<Vector4f, Matrix4f> ekf_estimation( Vector4f &xEst, 
                                             Matrix4f &PEst, 
                                             Vector3f z, 
                                             Vector2f u) {

        // Predict
        Vector4f xPred = motion_model(xEst, u);
        pair<Matrix4f, Matrix<float, 4, 2>> jacobF_B_result = jacobF_B(xPred, u);
        Matrix4f jF = jacobF_B_result.first;
        Matrix<float, 4, 2> jB = jacobF_B_result.second;
        Matrix4f PPred = jF * PEst * jF.transpose() + jB * Q * jB.transpose();

        // Correction
        Matrix<float, 3, 4> jH = jacobH();
        Vector3f zPred = observation_model(xPred);
        Vector3f y = z - zPred;
        Matrix3f S = jH * PPred * jH.transpose() + R;
        Matrix<float, 4, 3> K = PPred * jH.transpose() * S.inverse();
        xEst = xPred + K * y;
        PEst = (Matrix4f::Identity() - K * jH) * PPred;

        return make_pair(xEst, PEst);
    };

    pair<Vector4f, Matrix4f> estimate( Vector4f& xEst,
                                        Matrix4f& PEst,
                                        Vector3f& z,
                                        Vector2f& u,
                                        Matrix3f& R_) {
        R = R_;                                                        
        return ekf_estimation(xEst, PEst, z, u);
    }
};

class SensorFusion {
private:
    ros::NodeHandle nh_;

    ros::Subscriber gps_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher pub_;
    ros::Publisher pub_gps_origin_;
    ros::Publisher pub_utm_origin_;
    ros::Publisher pub_imu_;
    ros::Publisher pub_gps_;

    double x_utm_;
    double y_utm_;
    double roll_imu_;
    double pitch_imu_;
    double yaw_imu_;
    double acc_imu_;
    double w_imu_;
    double v_odom_;

    bool sub_imu_;
    bool sub_odom_;
    bool sub_gps_;
    
    double long_origin_;
    double lat_origin_;
    double lat_;
    double long_;

    bool flag_start_;
    double easting_ori_;
    double northing_ori_;
    double zone_number_ori_;
    string zone_letter_ori_;
    double easting_ori2odom_ori_;
    double northing_ori2odom_ori_;
    double x_fusion_;
    double y_fusion_;
    int rate_;
    Matrix2f Q_;
    Matrix3f R_;
    ros::Rate rate;

public:
    SensorFusion(const Matrix2f& Q, const Matrix3f& R) : nh_(""), x_utm_(0.0), y_utm_(0.0), roll_imu_(0.0), pitch_imu_(0.0), yaw_imu_(0.0),
                     acc_imu_(0.0), w_imu_(0.0), sub_imu_(false), v_odom_(0.0), sub_odom_(false),
                     long_origin_(std::nan("")), lat_origin_(std::nan("")), lat_(0.0), long_(0.0), sub_gps_(false),
                     easting_ori_(std::nan("")), northing_ori_(std::nan("")), zone_number_ori_(std::nan("")), zone_letter_ori_(""),
                     easting_ori2odom_ori_(0.0), northing_ori2odom_ori_(0.0), flag_start_(false),
                     x_fusion_(0.0), y_fusion_(0.0), rate_(50), Q_(Q), R_(R), rate(rate_) {

        // Initialize subscribers
        gps_sub_ = nh_.subscribe("/ublox/fix", 100, &SensorFusion::gps_callback, this);
        imu_sub_ = nh_.subscribe("/imu/data", 100, &SensorFusion::imu_callback, this);
        odom_sub_ = nh_.subscribe("/odom", 100, &SensorFusion::odom_callback, this);

        // Initialize publishers
        pub_ = nh_.advertise<nav_msgs::Odometry>("ekf4/odometry/filtered", 10);
        pub_gps_origin_ = nh_.advertise<sensor_msgs::NavSatFix>("ekf4/gps/origin", 10);
        pub_utm_origin_ = nh_.advertise<geometry_msgs::Point>("ekf4/utm/origin", 10);
        pub_imu_ = nh_.advertise<sensor_msgs::Imu>("ekf4/imu/filtered", 10);
        pub_gps_ = nh_.advertise<sensor_msgs::NavSatFix>("ekf4/gps/filtered", 10);

        // Initialize rate
        rate = ros::Rate(rate_);
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
        sub_imu_ = true;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(msg->orientation, orientation);
        tf::Matrix3x3 mat(orientation);
        mat.getRPY(roll_imu_, pitch_imu_, yaw_imu_);
        w_imu_ = msg->angular_velocity.z;
        acc_imu_ = msg->linear_acceleration.x;
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        sub_odom_ = true;
        v_odom_ = msg->twist.twist.linear.x;
    }

    pair<double, double> longlat2xy(double longitude, double latitude) {
        double x, y;
        double easting, northing;
        string zone;
        LLtoUTM(latitude, longitude, northing, easting, zone);
        x = easting - easting_ori_ + easting_ori2odom_ori_;
        y = northing - northing_ori_ + northing_ori2odom_ori_;
        return make_pair(x, y);
    }

    pair<double, double> xy2longlat(double x, double y) {
        double longitude, latitude;
        UTMtoLL(northing_ori_ + y - northing_ori2odom_ori_,
                easting_ori_ + x - easting_ori2odom_ori_, 
                zone_letter_ori_,
                latitude,longitude);
        return make_pair(longitude, latitude);
    }

    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        sub_gps_ = true;
        double longitude = msg->longitude;
        double latitude = msg->latitude;

        if (msg->position_covariance[0] < 0.3) {
            if (isnan(long_origin_) || isnan(lat_origin_)) {
                long_origin_ = longitude;
                lat_origin_ = latitude;
                easting_ori2odom_ori_ = x_fusion_;
                northing_ori2odom_ori_ = y_fusion_;
            }
            if (isnan(easting_ori_) || isnan(northing_ori_) || zone_letter_ori_.empty()) {
                LLtoUTM(lat_origin_, long_origin_, northing_ori_, easting_ori_, zone_letter_ori_);
            }
        }

        if (msg->status.status == 4) {
            if (isnan(long_origin_) || isnan(lat_origin_)) {
                long_origin_ = longitude;
                lat_origin_ = latitude;
                easting_ori2odom_ori_ = x_fusion_;
                northing_ori2odom_ori_ = y_fusion_;
            }
            if (isnan(easting_ori_) || isnan(northing_ori_) || zone_letter_ori_.empty()) {
                LLtoUTM(lat_origin_, long_origin_, northing_ori_, easting_ori_, zone_letter_ori_);
            }
        }

        //Matrix2d R;
        if (msg->status.status == 4) {
            R_(0, 0) = msg->position_covariance[0];
            R_(1, 1) = R_(0, 0);
        } else if (msg->status.status == 5) {
            R_(0, 0) = 1000.0;
            R_(1, 1) = R_(0, 0);
        } else {
            R_(0, 0) = 1000.0;
            R_(1, 1) = R_(0, 0);
        }

        if (isnan(easting_ori_) != 1 && isnan(northing_ori_) != 1) {
            pair<double, double> xy = longlat2xy(longitude, latitude);
            x_utm_ = xy.first + distance_gps2ori * cos(yaw_imu_);
            y_utm_ = xy.second + distance_gps2ori * sin(yaw_imu_);
        }
    }

    void pub_gps_origin() {
        sensor_msgs::NavSatFix msg;
        msg.latitude = lat_origin_;
        msg.longitude = long_origin_;
        pub_gps_origin_.publish(msg);
    }

    void pub_utm_origin() {
        geometry_msgs::Point msg;
        msg.x = easting_ori_;
        msg.y = northing_ori_;
        pub_utm_origin_.publish(msg);
    }

    void run() {
        ROS_INFO("EKF");

        while ((!(sub_imu_ && sub_gps_ && sub_odom_)) && ros::ok()) {
            ROS_INFO("imu: %d, gps: %d, odom: %d", sub_imu_, sub_gps_, sub_odom_);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("EKF started");    
        Vector4f xEst = Vector4f::Zero();
        xEst(2) = yaw_imu_;
        Matrix4f PEst = Matrix4f::Identity();
        Vector3f z;
        Vector2f ud;
        tie(z, ud) = __observation();
        EKF ekf(Q_, R_, rate_);

        double timer = ros::Time::now().toSec();
        while (ros::ok()) {

            tie(z, ud) = __observation();
            tie(xEst, PEst) = ekf.estimate(xEst, PEst, z, ud, R_);
            x_fusion_ = xEst(0);
            y_fusion_ = xEst(1);

            nav_msgs::Odometry msg;
            msg.pose.pose.position.x = xEst(0);
            msg.pose.pose.position.y = xEst(1);
            msg.pose.pose.position.z = 0.0;

            tf::Quaternion q;
            q.setRPY(roll_imu_, pitch_imu_, xEst(2));
            msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(xEst(2));

            msg.twist.twist.linear.x = xEst(3);
            msg.twist.twist.linear.y = 0.0;
            msg.twist.twist.linear.z = 0.0;

            msg.twist.twist.angular.x = 0.0;
            msg.twist.twist.angular.y = 0.0;
            msg.twist.twist.angular.z = w_imu_;
            msg.header.frame_id = "odom";

            sensor_msgs::Imu imu_msg;
            imu_msg.header.frame_id = "odom";
            imu_msg.orientation = msg.pose.pose.orientation;
            imu_msg.orientation_covariance[0] = PEst(2, 0);

            pub_imu_.publish(imu_msg);
            pub_.publish(msg);

            if (isnan(long_origin_) != 1 && isnan(lat_origin_) != 1) {
                sensor_msgs::NavSatFix msg_gps;
                msg_gps.header.frame_id = "sensor_link";
                pair<double, double> longlat = xy2longlat(xEst(0), xEst(1));
                msg_gps.latitude = longlat.second;
                msg_gps.longitude = longlat.first;
                msg_gps.position_covariance[0] = PEst(0, 0);
                pub_gps_.publish(msg_gps);
            }

            // Publish origin
            if (ros::Time::now().toSec() - timer > 1.0) {  // publish 1hz
                pub_gps_origin();
                // pub_utm_origin(); // Uncomment if publishing UTM origin
                timer = ros::Time::now().toSec();
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

    pair<Vector3f, Vector2f> __observation() {
        Vector3f z;
        z << x_utm_, y_utm_, v_odom_;

        Vector2f ud;
        ud << acc_imu_, w_imu_; // IMU data

        return make_pair(z, ud);
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_fusion_node");

    Matrix2f Q;
    Q << 0.1, 0.0,
        0.0, pow(PI/180.0, 2);

    Matrix3f R;
    R << pow(1000, 2), 0.0,           0.0,
        0.0,          pow(1000, 2),  0.0,
        0.0,          0.0,           pow(1000, 2);

    SensorFusion sensor_fusion(Q, R);
    sensor_fusion.run();
    return 0;
}

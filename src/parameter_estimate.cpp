#include <stdio.h>
#include <iostream>
#include <ceres/ceres.h>
#include <chrono>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include "math.h"
#include <algorithm>
#include <ros/ros.h>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/timer.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <roboteq_diff_msgs/Encoder.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))
const double encoder_cpr = 5000.0;
const double radian = 57.295779;

struct ESTIMATE_POSE_COST
{
    ESTIMATE_POSE_COST(int64_t encoder_r, int64_t encoder_l, double vel_x, 
                  double vel_y):_encoder_r( encoder_r ),_encoder_l( encoder_l ),
                  _vel_x( vel_x ),_vel_y( vel_y ){}
    template <typename T_1>
    bool operator()(const T_1* const abc, T_1* residual) const
    {
        T_1 linear = (T_1((double)_encoder_r)/encoder_cpr * abc[0] 
                      + T_1((double)_encoder_l)/encoder_cpr * abc[1])/2.0;
        T_1 angular  = (T_1((double)_encoder_l)/encoder_cpr * abc[1]
                       - T_1((double)_encoder_r)/encoder_cpr * abc[0])/abc[2];
        residual[0] = linear - T_1(_vel_x);
        residual[1] = angular  - T_1(_vel_y);
        
        return true;
    }
    const int64_t _encoder_r, _encoder_l;
    const double _vel_x, _vel_y;
 
};

struct MessageSync
{
public:
   void SubscribMsg(const nav_msgs::Odometry::ConstPtr& OdomMsg,const roboteq_diff_msgs::Encoder::ConstPtr& EncoMsg);
   void EstimatePara();
public:
   std::vector<int64_t> RightEncoderBuf_, LeftEncoderBuf_;
   std::vector<float> CameraOdomXBuf_, CameraOdomYBuf_, CameraOdomYawBuf_;
   std::vector<float> residual_;
   int64_t MsgNum_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "SubMsg");
    ros::start();
    ros::NodeHandle nh;
    
    MessageSync MsgSub;
    MessageSync *p = &MsgSub;
    
    message_filters::Subscriber<nav_msgs::Odometry> OdomSub(nh, "/zed_odom", 1);
    message_filters::Subscriber<roboteq_diff_msgs::Encoder> EncoderSub(nh, "/roboteq/encoder", 1);
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, roboteq_diff_msgs::Encoder> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10),OdomSub,EncoderSub);
    sync.registerCallback(boost::bind(&MessageSync::SubscribMsg, p ,_1,_2));
    
    ros::spin();
    ros::shutdown();
    p->EstimatePara();

    return 0; 
}

void MessageSync::SubscribMsg(const nav_msgs::Odometry::ConstPtr& OdomMsg,const roboteq_diff_msgs::Encoder::ConstPtr& EncoMsg)
{
    ++MsgNum_;
    RightEncoderBuf_.push_back(EncoMsg->right_vel);
    LeftEncoderBuf_.push_back(EncoMsg->left_vel);
    CameraOdomXBuf_.push_back(OdomMsg->pose.pose.position.x);
    CameraOdomYBuf_.push_back(OdomMsg->pose.pose.position.y);
    
    tf2::Quaternion q(
    OdomMsg->pose.pose.orientation.x,
    OdomMsg->pose.pose.orientation.y,
    OdomMsg->pose.pose.orientation.z,
    OdomMsg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    CameraOdomYawBuf_.push_back(yaw * radian);
    std::cout<<"RightEnco: "<<EncoMsg->right_vel<<"  LeftEnco: "<<EncoMsg->left_vel
             <<"  XPos: "<<OdomMsg->pose.pose.position.x<<"  YPOs: "<<OdomMsg->pose.pose.position.y
             <<"  Yaw: "<<yaw * radian<<std::endl;
    while(!ros::ok())
    {
        std::cout<<"Error!"<<std::endl;
    }
}

void MessageSync::EstimatePara()
{
    double abc[3] = {1.0,1.0, 1.0};
    ceres::Problem problem;
    for(int i = 1; i< RightEncoderBuf_.size(); i++)
    {
        double camera_dx = (CameraOdomXBuf_[i] - CameraOdomXBuf_[i-1]);
        double camera_dy = (CameraOdomYBuf_[i] - CameraOdomYBuf_[i-1]);//在odom坐标系下
        
        double camera_linear = sqrt(camera_dx*camera_dx + camera_dy*camera_dy);
        double camera_th = NORMALIZE((CameraOdomYawBuf_[i] - CameraOdomYawBuf_[i-1])/radian);
        
        problem.AddResidualBlock(
               new ceres::AutoDiffCostFunction<ESTIMATE_POSE_COST, 2, 3>(
                  new ESTIMATE_POSE_COST(RightEncoderBuf_[i],LeftEncoderBuf_[i],
                                         camera_linear, camera_th)
               ),
               nullptr,
               abc
       );
    }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout<<"time cost: "<<time_used.count()<<std::endl;

    std::cout<<summary.BriefReport()<<std::endl;
    std::cout<<"a b c = ";
    for(auto a:abc) std::cout<<a<<" ";
    std::cout<<std::endl;
}
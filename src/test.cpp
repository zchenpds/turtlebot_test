/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <string.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <math.h>

#define PI 3.1415927
//using namespace visualization_msgs;

const double LOOP_FREQ = 10;

double posX, posY, theta;

bool ready = false;

class Test
{
public:


    ros::NodeHandle nh, nh_odom;
    ros::Publisher vel_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber adp_sub; // subscribe to ADP omg

    double linear_scale;
    double angular_scale;

    std::string link_name;


    double posXCmd, posYCmd, oriZCmd;
    double Kr, Ka, Kb;
    double omgInst; // adp output - intantaneous omega
    
    int mode;

public:
    static void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void adpCallback(const geometry_msgs::Twist::ConstPtr& msg);

    Test() : nh("~")
    {
      nh.param<std::string>("link_name", link_name, "/base_link");
      //nh.param<double>("linear_scale", linear_scale, 1.0);
      //nh.param<double>("angular_scale", angular_scale, 2.2);
      nh.getParam("linear_scale", linear_scale);
      nh.getParam("angular_scale", angular_scale);
      nh.getParam("Kr", Kr);
      nh.getParam("Ka", Ka);
      nh.getParam("Kb", Kb);
      vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
      odom_sub = nh.subscribe("/odom", 1, &odomCallback);
      adp_sub = nh.subscribe("/cmd_omg", 1, &Test::adpCallback, this);
      ROS_INFO("[Test] Initialized.");
    }
    void circleControl(double stepSize);
    void sineVelControl(double stepSize);
    void sinePosControl(double stepSize);
    void execPosCmd();
    void execTwist(geometry_msgs::Twist vel);
    void execTwist1();
    void execTwist2();
    int getMode();
    void adpVelControl(double stepSize, double omgInst);
    
};

void Test::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    double oriZ = msg->pose.pose.orientation.z;
//    double oriW = msg->pose.pose.orientation.w;
    theta = asin(oriZ) * 2;
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("x:[%f], y:[%f], theta:[%f]", posX, posY, theta);
//    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    ready = true;

}

void Test::circleControl(double stepSize)
{
    geometry_msgs::Twist velCmd;
    velCmd.linear.x = 0.2;
    velCmd.angular.z = 0.8;
    execTwist(velCmd);
}

void Test::sineVelControl(double stepSize)
{
    //const double OMEGA = 0.5;
    double omega, init_phase;
    nh.getParam("omega", omega);
    nh.getParam("init_phase", init_phase);
    const double A = 1;
    //if (omega > 0.6/A) omega = 0.6/A;
    static double phi = init_phase;
    if (phi > 2*PI) phi -= 2 * PI;
    else if (phi < -2*PI) phi += 2 * PI;
    phi += omega * stepSize;

    geometry_msgs::Twist velCmd;
    velCmd.linear.x = A*omega*sin(phi);
    velCmd.angular.z = 0;
    //ROS_INFO("Velocity Control Mode!");
    //if(ready) execTwist(velCmd);
    execTwist(velCmd);

}


void Test::sinePosControl(double stepSize)
{
    const double OMEGA = 0;	
    const double A = 1.5;
    static double phi = PI/2;
    if (phi > 2*PI) phi -= 2 * PI;
    else if (phi < -2*PI) phi += 2 * PI;
    phi += OMEGA * stepSize;
    // posXCmd = A*(1 + sin(phi));
    switch (mode)
    {
    case 1:     posXCmd = 0; posYCmd = 0; break;
    case 2:     posXCmd = 1; posYCmd = 1; break;
    //case 3:

    }
    ROS_INFO("posXCmd = [%f], posYCmd = [%f]", posXCmd, posYCmd);
    if(ready) execPosCmd();
}

void Test::execPosCmd()
{
    double errorPosX = posXCmd - posX;
    double errorPosY = posYCmd - posY;

    double rho = sqrt(errorPosX*errorPosX + errorPosY*errorPosY);
    double alpha = -theta + atan2(errorPosY, errorPosX);
//    while (alpha > PI) alpha -= 2*PI;
//    while (alpha < -PI) alpha += 2*PI;
    double beta = -theta -alpha;
    while (beta > PI) beta -= 2*PI;
    while (beta < -PI) beta += 2*PI;

    double cmdX;
    double cmdZ;
    if (rho > 0.01)
    {
        cmdX = Kr*rho;
        cmdZ = (Ka*alpha + Kb*beta);
    }
//    else //if (rho > 0.1)
//    {
//        cmdX = Kr*rho;
//        cmdZ = (-Ka*alpha + Kb*beta);//*rho;
//    }
    else
    {
        cmdX = 0;
        cmdZ = -4*theta;
    }
    ROS_INFO("alpha = [%f], beta = [%f]", alpha, beta);

    geometry_msgs::Twist velCmd;
    if (alpha >= -PI/2 && alpha <= PI/2) 
    {
        velCmd.linear.x = cmdX;
        velCmd.angular.z = cmdZ;
    }
    else
    {
        velCmd.linear.x = -cmdX;//-cmdX;
        if (alpha > PI/2) velCmd.angular.z = Ka*(alpha - PI);
        else velCmd.angular.z = Ka*(alpha + PI);        
    }
    execTwist(velCmd);

}

void Test::execTwist(geometry_msgs::Twist vel)
{
    ROS_INFO("velCmd--- x: [%f], z: [%f]", vel.linear.x, vel.angular.z);
    vel_pub.publish(vel);
}

void Test::execTwist1()
{
    // Handle angular change (yaw is the only direction in which you can rotate)
    // double yaw = tf::getYaw(feedback->pose.orientation);

    geometry_msgs::Twist vel;
    // vel.angular.z = angular_scale*yaw;
    // vel.linear.x = linear_scale*feedback->pose.position.x;
    vel.angular.z = angular_scale*0.3;
    vel.linear.x = linear_scale*0;

    vel_pub.publish(vel);
}

void Test::execTwist2()
{
    // Handle angular change (yaw is the only direction in which you can rotate)
    // double yaw = tf::getYaw(feedback->pose.orientation);

    geometry_msgs::Twist vel;
    // vel.angular.z = angular_scale*yaw;
    // vel.linear.x = linear_scale*feedback->pose.position.x;
    vel.angular.z = -angular_scale*0.3;
    vel.linear.x = linear_scale*0;

    vel_pub.publish(vel);
}
int Test::getMode()
{
    nh.getParam("mode", mode);
    return mode;
}

// ***** ADP robot control ***** //
void Test::adpVelControl(double stepSize, double omgInst)
{
    //const double OMEGA = 0.5;
    double init_phase;    
    nh.getParam("init_phase", init_phase);
    const double A = 1.5;
    //if (omega > 0.6/A) omega = 0.6/A;
    static double phi = init_phase;
    if (phi > 2*PI) phi -= 2 * PI;
    else if (phi < -2*PI) phi += 2 * PI;
    phi += omgInst * stepSize;

    geometry_msgs::Twist velCmd;
    velCmd.linear.x = A*omgInst*sin(phi);
    velCmd.angular.z = 0;
    //ROS_INFO("Velocity Control Mode!");
    //if(ready) execTwist(velCmd);
    ROS_INFO("inst omega:[%f]", omgInst);
    execTwist(velCmd);

}


// ***** ADP instantaneous omega callback ***** //
void Test::adpCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    this->omgInst = msg->angular.x;
    //ROS_INFO("inst omega:[%f]", omgInst);
}


int main(int argc, char** argv)
{
    ROS_INFO("[Test] Ready to initial.");
    ros::init(argc, argv, "test");

    Test test;
    int mode;
    //double omgInst;
    ros::Rate loop_rate(LOOP_FREQ);

    while (ros::ok())
    {
       // ros::spin();
//        test.execTwist1();
//        ros::spinOnce();
//        loop_rate.sleep();
//        test.execTwist2();
//        ros::spinOnce();
//        loop_rate.sleep();
        
        ros::spinOnce();
        mode = test.getMode();
        //ROS_INFO("inst omega:[%f]", omgInst);
        if(mode == 3) test.circleControl(1/LOOP_FREQ);
        else if(mode == 2) test.sinePosControl(1/LOOP_FREQ);
        else if(mode == 1) test.sineVelControl(1/LOOP_FREQ);        
        else if(mode == 4) test.adpVelControl(1/LOOP_FREQ, test.omgInst);
        loop_rate.sleep();
    }
}

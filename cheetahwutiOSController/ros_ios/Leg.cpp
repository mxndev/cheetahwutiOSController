//
//  Leg.m
//  ros_ios
//
//  Created by Mikołaj-iMac on 14.12.2015.
//  Copyright © 2015 Ronan Chauvin. All rights reserved.
//

#include <Leg.h>

#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/lexical_cast.hpp>

#define FRONT_L1 0.2
#define FRONT_L2 0.18
#define REAR_L1 0.14
#define REAR_L2 0.12
#define REAR_L3 0.12

#define HIP_MIN -1.5
#define HIP_MAX 2

#define KNEE_MIN 0.4
#define KNEE_MAX 2.5

#define REAR_BOTTOM_X -0.1
#define REAR_TOP_X -0.05
#define FRONT_BOTTOM_X 0.1
#define FRONT_TOP_X 0.05
#define FRONT_MIN_Y 0.24
#define FRONT_MAX_Y 0.32
#define REAR_MIN_Y 0.24
#define REAR_MAX_Y 0.32

Leg::Leg(LegType leg_type, int current_time_forward, int current_time_reverse, ros::NodeHandle &nh) :
leg_type_(leg_type), current_time_forward(current_time_forward), current_time_reverse(current_time_reverse), nh_(nh)
{
    //helper value - shortens various expressions
    is_front_leg_ = (leg_type_ == LegTypes::FrontLeft || leg_type_ == LegTypes::FrontRight);
    
    //helper value used in publisher/subscriber topics
    std::string leg_number = boost::lexical_cast<std::string>(int(leg_type_));
    
    //Initialize publishers
    hip_pub_ = nh_.advertise<std_msgs::Float64>("/cheetahwut/hip"+leg_number+"_position_controller/command", 1000);
    knee_pub_ = nh_.advertise<std_msgs::Float64>("/cheetahwut/knee"+leg_number+"_position_controller/command", 1000);
    if (!is_front_leg_)
    {
        ankle_pub_ = nh_.advertise<std_msgs::Float64>("/cheetahwut/ankle"+leg_number+"_position_controller/command", 1000);
    }
    
    //set geometry
    if (is_front_leg_)
    {
        geometry_.l1 = FRONT_L1;
        geometry_.l2 = FRONT_L2;
    }
    else
    {
        geometry_.l1 = REAR_L1 + REAR_L3;
        geometry_.l2 = REAR_L2;
    }
    
    calculateDefaultPositionTable();
}

int Leg::GetTimer(bool reverse)
{
    return (reverse) ? current_time_reverse : current_time_forward;
}

LegType Leg::getLegType() const
{
    return leg_type_;
}

void Leg::calculateLinearPathFromResolutionStartStop(int offset, int length, Position start, Position stop, bool reverse)
{
    double increment_y = (stop.y - start.y) / length;
    double increment_x = (stop.x - start.x) / length;
    
    for (int i = 0; i < length; i++)
    {
        if(reverse)
        {
            positionTableReverse[offset + i].x = start.x + (i * increment_x);
            positionTableReverse[offset + i].y = start.y + (i * increment_y);
        } else {
            positionTableForward[offset + i].x = start.x + (i * increment_x);
            positionTableForward[offset + i].y = start.y + (i * increment_y);
        }
    }
}

void Leg::calculateDefaultPositionTable()
{
    /*
     * Assign extreme positions. The route is A->B->C->D->A etc.
     * Coordinates are defined on top.
     */
    Position A,B,C,D;
    A.x = REAR_BOTTOM_X;
    A.y = REAR_MAX_Y;
    B.x = REAR_TOP_X;
    B.y = REAR_MIN_Y;
    C.x = FRONT_TOP_X;
    C.y = FRONT_MIN_Y;
    D.x = FRONT_BOTTOM_X;
    D.y = FRONT_MAX_Y;
    
    //A->D
    calculateLinearPathFromResolutionStartStop(0, 15, D, C, true);
    //D->C
    calculateLinearPathFromResolutionStartStop(15, 20, C, B, true);
    //C->B
    calculateLinearPathFromResolutionStartStop(35, 15, B, A, true);
    //B->A
    calculateLinearPathFromResolutionStartStop(50, 350, A, D, true);
    //A->B
    calculateLinearPathFromResolutionStartStop(0, 15, A, B, false);
    //B->C
    calculateLinearPathFromResolutionStartStop(15, 20, B, C, false);
    //C->D
    calculateLinearPathFromResolutionStartStop(35, 15, C, D, false);
    //D->A
    calculateLinearPathFromResolutionStartStop(50, 350, D, A, false);
}

void Leg::doStep(bool reverse)
{
    //gets x and y from position table and passes them to set_state_
    getCoordinatesForTime(reverse);

    //calculate joint coordinates from x and y and passes them to set_state_
    calculateIK();
    
    //publish them
    publishMessages();
    
    //increment timer
    if(reverse)
    {
        ++current_time_reverse %= 400;
    } else {
        ++current_time_forward %= 400;
    }
}

void Leg::calculateIK()
{
    double x,y, l1, l2;
    x = set_state_.coordinates.x;
    y = set_state_.coordinates.y;
    l1 = geometry_.l1;
    l2 = geometry_.l2;
    
    double beta = acos( (pow(x, 2) + pow(y, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2) );
    
    double sigma = asin( (l2 * sin((is_front_leg_ ? 1 : -1) * beta)) / (sqrt(pow(x, 2) + pow(y, 2))) );
    double gamma = 0;
    
    if (x != 0)
    {
        gamma = atan2(y, x);
        gamma -= M_PI/2;
    }
    
    sigma += gamma;
    
    //delimiters
    if (beta < KNEE_MIN)
        beta = KNEE_MIN;
    else if (beta > KNEE_MAX)
        beta = KNEE_MAX;
    
    if (sigma < HIP_MIN)
        sigma = HIP_MIN;
    else if (sigma > HIP_MAX)
        sigma = HIP_MAX;
    
    set_state_.angles.knee = beta;
    set_state_.angles.hip = sigma;
}

void Leg::publishMessages()
{
    std_msgs::Float64 msg;
    
    msg.data = set_state_.angles.hip;
    hip_pub_.publish(msg);
    msg.data = set_state_.angles.knee;
    knee_pub_.publish(msg);
    
    if (!is_front_leg_)
    {
        ankle_pub_.publish(msg);
    }
}

void Leg::getCoordinatesForTime(bool reverse)
{
    if(reverse)
    {
        set_state_.coordinates.x = positionTableReverse[current_time_reverse].x;
        set_state_.coordinates.y = positionTableReverse[current_time_reverse].y;
    } else {
        set_state_.coordinates.x = positionTableForward[current_time_forward].x;
        set_state_.coordinates.y = positionTableForward[current_time_forward].y;
    }
}

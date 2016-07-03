//
//  RotateLeg.cpp
//  ros_ios
//
//  Created by Mikołaj-iMac on 14.12.2015.
//  Copyright © 2015 Ronan Chauvin. All rights reserved.
//

#include <RotateLeg.h>

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

//tylna prawa
#define START_POS_TP_X -0.05
#define START_POS_TP_Y 0.32
#define START_POS_TP_Z 0.0
#define UP_POS_TP_X -0.05
#define UP_POS_TP_Y 0.24
#define UP_POS_TP_Z 0.0
#define UP_ROT_TP_X -0.05
#define UP_ROT_TP_Y 0.24
#define UP_ROT_TP_Z -0.05
#define DOWN_ROT_TP_X -0.05
#define DOWN_ROT_TP_Y 0.32
#define DOWN_ROT_TP_Z -0.05
//tylna lewa
#define START_POS_TL_X -0.1
#define START_POS_TL_Y 0.32
#define START_POS_TL_Z 0.0
#define UP_POS_TL_X -0.1
#define UP_POS_TL_Y 0.28
#define UP_POS_TL_Z 0.0
#define UP_ROT_TL_X -0.1
#define UP_ROT_TL_Y 0.28
#define UP_ROT_TL_Z -0.05
#define DOWN_ROT_TL_X -0.1
#define DOWN_ROT_TL_Y 0.32
#define DOWN_ROT_TL_Z -0.05
//przednia prawa
#define START_POS_PP_X 0.15
#define START_POS_PP_Y 0.32
#define START_POS_PP_Z 0.0
#define UP_POS_PP_X 0.15
#define UP_POS_PP_Y 0.24
#define UP_POS_PP_Z 0.0
#define UP_ROT_PP_X 0.15
#define UP_ROT_PP_Y 0.24
#define UP_ROT_PP_Z -0.05
#define DOWN_ROT_PP_X 0.15
#define DOWN_ROT_PP_Y 0.32
#define DOWN_ROT_PP_Z -0.05
//przednia lewa
#define START_POS_PL_X 0.1
#define START_POS_PL_Y 0.32
#define START_POS_PL_Z 0.0
#define UP_POS_PL_X 0.1
#define UP_POS_PL_Y 0.24
#define UP_POS_PL_Z 0.0
#define UP_ROT_PL_X 0.1
#define UP_ROT_PL_Y 0.24
#define UP_ROT_PL_Z -0.05
#define DOWN_ROT_PL_X 0.1
#define DOWN_ROT_PL_Y 0.32
#define DOWN_ROT_PL_Z -0.05


RotateLeg::RotateLeg(LegType leg_type, int start_time_forward, int start_time_reverse, ros::NodeHandle &nh) :
leg_type_(leg_type), start_time_forward(start_time_forward), start_time_reverse(start_time_reverse), nh_(nh)
{
    //helper value - shortens various expressions
    is_front_leg_ = (leg_type_ == LegTypes::FrontLeft || leg_type_ == LegTypes::FrontRight);
    
    //helper value used in publisher/subscriber topics
    std::string leg_number = boost::lexical_cast<std::string>(int(leg_type_));
    
    current_time_forward = 0;
    current_time_reverse = 0;
    
    //Initialize publishers
    hip_pub_ = nh_.advertise<std_msgs::Float64>("/cheetahwut/hip"+leg_number+"_position_controller/command", 1000);
    knee_pub_ = nh_.advertise<std_msgs::Float64>("/cheetahwut/knee"+leg_number+"_position_controller/command", 1000);
    if (!is_front_leg_)
    {
        ankle_pub_ = nh_.advertise<std_msgs::Float64>("/cheetahwut/ankle"+leg_number+"_position_controller/command", 1000);
    }
    
    hipX = nh.advertise<std_msgs::Float64>("/cheetahwut/hipX"+leg_number+"_position_controller/command", 1000);
    
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

int RotateLeg::GetTimer(bool reverse)
{
    return (reverse) ? current_time_reverse : current_time_forward;
}

LegType RotateLeg::getLegType() const
{
    return leg_type_;
}

void RotateLeg::calculateLinearPathFromResolutionStartStop(int offset, int end, Position start, Position stop, bool reverse)
{
    double l1,l2;
    l1 = geometry_.l1;
    l2 = geometry_.l2;
    
    if(reverse)
    {
        start.z = start.z*(-1);
        stop.z = stop.z*(-1);
    }
    
    double yawStart = asin(start.z/(sqrt(pow(start.x, 2) + pow(start.z, 2))));
    double betaStart = acos( (pow((cos(yawStart)*(sqrt(pow(start.x, 2) + pow(start.z, 2)))), 2) + pow(start.y, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2) );
    
    double sigmaStart = asin( (l2 * sin((is_front_leg_ ? 1 : -1) * betaStart)) / (sqrt(pow((cos(yawStart)*(sqrt(pow(start.x, 2) + pow(start.z, 2)))), 2) + pow(start.y, 2))) );
    double gammaStart = 0;
    
    if ((cos(yawStart)*(sqrt(pow(start.x, 2) + pow(start.z, 2)))) != 0)
    {
        gammaStart = atan2(start.y, (start.x < 0 ? -1 : 1) *(cos(yawStart)*(sqrt(pow(start.x, 2) + pow(start.z, 2)))));
        gammaStart -= M_PI/2;
    }
    
    sigmaStart += gammaStart;
    
    //delimiters
    if (betaStart < KNEE_MIN)
        betaStart = KNEE_MIN;
    else if (betaStart > KNEE_MAX)
        betaStart = KNEE_MAX;
    
    if (sigmaStart < HIP_MIN)
        sigmaStart = HIP_MIN;
    else if (sigmaStart > HIP_MAX)
        sigmaStart = HIP_MAX;
    
    double yawStop = asin(stop.z/(sqrt(pow(stop.x, 2) + pow(stop.z, 2))));
    double betaStop = acos( (pow((cos(yawStop)*(sqrt(pow(stop.x, 2) + pow(stop.z, 2)))), 2) + pow(stop.y, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2) );
    
    double sigmaStop = asin( (l2 * sin((is_front_leg_ ? 1 : -1) * betaStop)) / (sqrt(pow((cos(yawStop)*(sqrt(pow(stop.x, 2) + pow(stop.z, 2)))), 2) + pow(stop.y, 2))) );
    double gammaStop = 0;
    
    if ((cos(yawStop)*(sqrt(pow(stop.x, 2) + pow(stop.z, 2)))) != 0)
    {
        gammaStop = atan2(stop.y, (stop.x < 0 ? -1 : 1) *(cos(yawStop)*(sqrt(pow(stop.x, 2) + pow(stop.z, 2)))));
        gammaStop -= M_PI/2;
    }
    
    sigmaStop += gammaStop;
    
    //delimiters
    if (betaStop < KNEE_MIN)
        betaStop = KNEE_MIN;
    else if (betaStop > KNEE_MAX)
        betaStop = KNEE_MAX;
    
    if (sigmaStop < HIP_MIN)
        sigmaStop = HIP_MIN;
    else if (sigmaStop > HIP_MAX)
        sigmaStop = HIP_MAX;
    
    double increment_beta = (betaStop - betaStart) / (end-offset);
    double increment_sigma = (sigmaStop - sigmaStart) / (end-offset);
    double increment_yaw = (yawStop - yawStart) / (end-offset);
    
    for (int i = 0; i < (end-offset); i++)
    {
        if(reverse)
        {
            positionTableLeft[offset + i].hip = sigmaStart + (i * increment_sigma);
            positionTableLeft[offset + i].knee = betaStart + (i * increment_beta);
            positionTableLeft[offset + i].yaw = yawStart + (i * increment_yaw);
        } else {
            positionTableRight[offset + i].hip = sigmaStart + (i * increment_sigma);
            positionTableRight[offset + i].knee = betaStart + (i * increment_beta);
            positionTableRight[offset + i].yaw = yawStart + (i * increment_yaw);
        }
    }
}

void RotateLeg::calculateDefaultPositionTable()
{
    /*
     * Assign extreme positions. The route is A->B->C->D->A etc.
     * Coordinates are defined on top.
     */
    Position A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P;
    //tylna prawa
    A.x = START_POS_TP_X;
    A.y = START_POS_TP_Y;
    A.z = START_POS_TP_Z;
    B.x = UP_POS_TP_X;
    B.y = UP_POS_TP_Y;
    B.z = UP_POS_TP_Z;
    M.x = UP_ROT_TP_X;
    M.y = UP_ROT_TP_Y;
    M.z = UP_ROT_TP_Z;
    I.x = DOWN_ROT_TP_X;
    I.y = DOWN_ROT_TP_Y;
    I.z = DOWN_ROT_TP_Z;
    //tylna lewa
    C.x = START_POS_TL_X;
    C.y = START_POS_TL_Y;
    C.z = START_POS_TL_Z;
    D.x = UP_POS_TL_X;
    D.y = UP_POS_TL_Y;
    D.z = UP_POS_TL_Z;
    N.x = UP_ROT_TL_X;
    N.y = UP_ROT_TL_Y;
    N.z = UP_ROT_TL_Z;
    J.x = DOWN_ROT_TL_X;
    J.y = DOWN_ROT_TL_Y;
    J.z = DOWN_ROT_TL_Z;
    //przednia prawa
    E.x = START_POS_PP_X;
    E.y = START_POS_PP_Y;
    E.z = START_POS_PP_Z;
    F.x = UP_POS_PP_X;
    F.y = UP_POS_PP_Y;
    F.z = UP_POS_PP_Z;
    O.x = UP_ROT_PP_X;
    O.y = UP_ROT_PP_Y;
    O.z = UP_ROT_PP_Z;
    K.x = DOWN_ROT_PP_X;
    K.y = DOWN_ROT_PP_Y;
    K.z = DOWN_ROT_PP_Z;
    //przednia lewa
    G.x = START_POS_PL_X;
    G.y = START_POS_PL_Y;
    G.z = START_POS_PL_Z;
    H.x = UP_POS_PL_X;
    H.y = UP_POS_PL_Y;
    H.z = UP_POS_PL_Z;
    P.x = UP_ROT_PL_X;
    P.y = UP_ROT_PL_Y;
    P.z = UP_ROT_PL_Z;
    L.x = DOWN_ROT_PL_X;
    L.y = DOWN_ROT_PL_Y;
    L.z = DOWN_ROT_PL_Z;
    
    //wersja w lewo
    //oczekiwanie
    calculateLinearPathFromResolutionStartStop(0, start_time_reverse+80, (leg_type_ == LegTypes::RearRight) ? A : (leg_type_ == LegTypes::FrontRight) ? E : (leg_type_ == LegTypes::RearLeft) ? C : G, (leg_type_ == LegTypes::RearRight) ? A : (leg_type_ == LegTypes::FrontRight) ? E : (leg_type_ == LegTypes::RearLeft) ? C : G, true);
    //podniesiecie nogi
    calculateLinearPathFromResolutionStartStop(start_time_reverse+80, start_time_reverse+110, (leg_type_ == LegTypes::RearRight) ? A : (leg_type_ == LegTypes::FrontRight) ? E : (leg_type_ == LegTypes::RearLeft) ? C : G, (leg_type_ == LegTypes::RearRight) ? B : (leg_type_ == LegTypes::FrontRight) ? F : (leg_type_ == LegTypes::RearLeft) ? D : H, true);
    //obrot nogi
    calculateLinearPathFromResolutionStartStop(start_time_reverse+110, start_time_reverse+180, (leg_type_ == LegTypes::RearRight) ? B : (leg_type_ == LegTypes::FrontRight) ? F : (leg_type_ == LegTypes::RearLeft) ? D : H, (leg_type_ == LegTypes::RearRight) ? M : (leg_type_ == LegTypes::FrontRight) ? O : (leg_type_ == LegTypes::RearLeft) ? N : P, true);
    //opuszczenie nogi
    calculateLinearPathFromResolutionStartStop(start_time_reverse+180, start_time_reverse+230, (leg_type_ == LegTypes::RearRight) ? M : (leg_type_ == LegTypes::FrontRight) ? O : (leg_type_ == LegTypes::RearLeft) ? N : P, (leg_type_ == LegTypes::RearRight) ? I : (leg_type_ == LegTypes::FrontRight) ? K : (leg_type_ == LegTypes::RearLeft) ? J : L, true);
    //wyrównanie innych nóg
    calculateLinearPathFromResolutionStartStop(start_time_reverse+230, 920, (leg_type_ == LegTypes::RearRight) ? I : (leg_type_ == LegTypes::FrontRight) ? K : (leg_type_ == LegTypes::RearLeft) ? J : L, (leg_type_ == LegTypes::RearRight) ? I : (leg_type_ == LegTypes::FrontRight) ? K : (leg_type_ == LegTypes::RearLeft) ? J : L, true);
    //powrót do pozycji 0
    calculateLinearPathFromResolutionStartStop(920, 1000, (leg_type_ == LegTypes::RearRight) ? I : (leg_type_ == LegTypes::FrontRight) ? K : (leg_type_ == LegTypes::RearLeft) ? J : L, (leg_type_ == LegTypes::RearRight) ? A : (leg_type_ == LegTypes::FrontRight) ? E : (leg_type_ == LegTypes::RearLeft) ? C : G, true);
    //wersja w prawo
    //oczekiwanie
    calculateLinearPathFromResolutionStartStop(0, start_time_forward+80, (leg_type_ == LegTypes::RearRight) ? A : (leg_type_ == LegTypes::FrontRight) ? E : (leg_type_ == LegTypes::RearLeft) ? C : G, (leg_type_ == LegTypes::RearRight) ? A : (leg_type_ == LegTypes::FrontRight) ? E : (leg_type_ == LegTypes::RearLeft) ? C : G, false);
    //podniesiecie nogi
    calculateLinearPathFromResolutionStartStop(start_time_forward+80, start_time_forward+110, (leg_type_ == LegTypes::RearRight) ? A : (leg_type_ == LegTypes::FrontRight) ? E : (leg_type_ == LegTypes::RearLeft) ? C : G, (leg_type_ == LegTypes::RearRight) ? B : (leg_type_ == LegTypes::FrontRight) ? F : (leg_type_ == LegTypes::RearLeft) ? D : H, false);
    //obrot nogi
    calculateLinearPathFromResolutionStartStop(start_time_forward+110, start_time_forward+180, (leg_type_ == LegTypes::RearRight) ? B : (leg_type_ == LegTypes::FrontRight) ? F : (leg_type_ == LegTypes::RearLeft) ? D : H, (leg_type_ == LegTypes::RearRight) ? M : (leg_type_ == LegTypes::FrontRight) ? O : (leg_type_ == LegTypes::RearLeft) ? N : P, false);
    //opuszczenie nogi
    calculateLinearPathFromResolutionStartStop(start_time_forward+180, start_time_forward+230, (leg_type_ == LegTypes::RearRight) ? M : (leg_type_ == LegTypes::FrontRight) ? O : (leg_type_ == LegTypes::RearLeft) ? N : P, (leg_type_ == LegTypes::RearRight) ? I : (leg_type_ == LegTypes::FrontRight) ? K : (leg_type_ == LegTypes::RearLeft) ? J : L, false);
    //wyrównanie innych nóg
    calculateLinearPathFromResolutionStartStop(start_time_forward+230, 920, (leg_type_ == LegTypes::RearRight) ? I : (leg_type_ == LegTypes::FrontRight) ? K : (leg_type_ == LegTypes::RearLeft) ? J : L, (leg_type_ == LegTypes::RearRight) ? I : (leg_type_ == LegTypes::FrontRight) ? K : (leg_type_ == LegTypes::RearLeft) ? J : L, false);
    //powrót do pozycji 0
    calculateLinearPathFromResolutionStartStop(920, 1000, (leg_type_ == LegTypes::RearRight) ? I : (leg_type_ == LegTypes::FrontRight) ? K : (leg_type_ == LegTypes::RearLeft) ? J : L, (leg_type_ == LegTypes::RearRight) ? A : (leg_type_ == LegTypes::FrontRight) ? E : (leg_type_ == LegTypes::RearLeft) ? C : G, false);
}

void RotateLeg::doStep(bool reverse)
{
    //gets x and y from position table and passes them to set_state_
    getCoordinatesForTime(reverse);
    
    //publish them
    publishMessages();
    
    //increment timer
    if(reverse)
    {
        ++current_time_reverse %= 1000;
    } else {
        ++current_time_forward %= 1000;
    }
}

void RotateLeg::publishMessages()
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
    
    msg.data = set_state_.angles.yaw;
    hipX.publish(msg);
}

void RotateLeg::getCoordinatesForTime(bool reverse)
{
    if(reverse)
    {
        set_state_.angles.knee = positionTableLeft[current_time_reverse].knee;
        set_state_.angles.hip = positionTableLeft[current_time_reverse].hip;
        set_state_.angles.yaw = positionTableLeft[current_time_reverse].yaw;
    } else {
        set_state_.angles.knee = positionTableRight[current_time_forward].knee;
        set_state_.angles.hip = positionTableRight[current_time_forward].hip;
        set_state_.angles.yaw = positionTableRight[current_time_forward].yaw;
    }
}

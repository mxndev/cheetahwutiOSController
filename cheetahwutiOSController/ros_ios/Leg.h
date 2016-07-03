//
//  Leg.h
//  ros_ios
//
//  Created by Mikołaj-iMac on 14.12.2015.
//  Copyright © 2015 Ronan Chauvin. All rights reserved.
//

#ifndef Leg_h
#define Leg_h

#ifndef LEG_H
#define LEG_H

#include <ros/ros.h>

//#include <gazebo_msgs/ContactsState.h>

namespace LegTypes
{
    enum LegType
    {
        FrontLeft = 1,
        FrontRight,
        RearLeft,
        RearRight,
        Empty
    };
}
typedef LegTypes::LegType LegType;

struct JointPosition
{
    double hip;
    double knee;
    double yaw;
};

struct Position
{
    double x;
    double y;
    double z;
};

struct Parameters
{
    double l1;
    double l2;
};

struct State
{
    JointPosition angles;
    Position coordinates;
};

class Leg
{
public:
    Leg(LegType leg_type, int current_time_forward, int current_time_reverse, ros::NodeHandle &nh);
    
    LegType getLegType() const;
    
    void doStep(bool reverse);
    int GetTimer(bool reverse);
    
private:
    LegType leg_type_;
    bool is_front_leg_;
    Parameters geometry_;
    State current_state_;
    State set_state_;
    int current_time_forward, current_time_reverse;
    Position positionTableForward[400], positionTableReverse[400];
    ros::NodeHandle nh_;
    ros::Publisher hip_pub_;
    ros::Publisher knee_pub_;
    ros::Publisher ankle_pub_;
    
    //Methods
    void calculateDefaultPositionTable();
    void calculateLinearPathFromResolutionStartStop(int offset, int length, Position start, Position stop, bool reverse);
    void getCoordinatesForTime(bool reverse);
    void calculateIK();
    void publishMessages();
};

#endif // LEG_H


#endif /* Leg_h */

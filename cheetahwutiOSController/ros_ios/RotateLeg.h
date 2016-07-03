//
//  RotateLeg.h
//  ros_ios
//
//  Created by Mikołaj-iMac on 14.12.2015.
//  Copyright © 2015 Ronan Chauvin. All rights reserved.
//

#ifndef RotateLeg_h
#define RotateLeg_h

#ifndef ROTATELEG_H
#define ROTATELEG_H

#include <ros/ros.h>
#include "Leg.h"

//#include <gazebo_msgs/ContactsState.h>

class RotateLeg
{
public:
    RotateLeg(LegType leg_type, int current_time_forward, int current_time_reverse, ros::NodeHandle &nh);
    
    LegType getLegType() const;
    
    void doStep(bool reverse);
    int GetTimer(bool reverse);
    
private:
    LegType leg_type_;
    bool is_front_leg_;
    Parameters geometry_;
    State set_state_;
    int current_time_forward, current_time_reverse, start_time_forward, start_time_reverse;
    JointPosition positionTableRight[1000], positionTableLeft[1000];
    ros::NodeHandle nh_;
    ros::Publisher hip_pub_;
    ros::Publisher knee_pub_;
    ros::Publisher ankle_pub_;
    ros::Publisher hipX;
    
    //Methods
    void calculateDefaultPositionTable();
    void calculateLinearPathFromResolutionStartStop(int offset, int length, Position start, Position stop, bool reverse);
    void getCoordinatesForTime(bool reverse);
    void publishMessages();
};

#endif // LEG_H


#endif /* Leg_h */

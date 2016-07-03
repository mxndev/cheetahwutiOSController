//
//  Sterowanie.h
//  ros_ios
//
//  Created by Mikołaj-iMac on 14.12.2015.
//  Copyright © 2015 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#include "Leg.h"
#include "RotateLeg.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>

@interface Sterowanie : UIViewController
{
    ros::NodeHandle n;
    std::vector<Leg*> lineLeg;
    std::vector<RotateLeg*> rotateLeg;
    NSTimer *timerInit, *timerGo, *timerRot;
    BOOL isStop, reverse;
    int counter, sekwencje;
}

@property (nonatomic, retain) IBOutlet UILabel *display;
@property(retain) IBOutlet UIButton *startButtonForward;
@property(retain) IBOutlet UIButton *stopButtonForward;
@property(retain) IBOutlet UIButton *startButtonReverse;
@property(retain) IBOutlet UIButton *stopButtonReverse;
@property(retain) IBOutlet UIButton *startRotateLeft;
@property(retain) IBOutlet UIButton *stopRotateLeft;
@property(retain) IBOutlet UIButton *startRotateRight;
@property(retain) IBOutlet UIButton *stopRotateRight;
@property(retain) IBOutlet UIButton *startRotateLeft90;
@property(retain) IBOutlet UIButton *stopRotateLeft90;
@property(retain) IBOutlet UIButton *startRotateRight90;
@property(retain) IBOutlet UIButton *stopRotateRight90;

-(IBAction)startGoForward:(id)sender;
-(IBAction)stopGoForward:(id)sender;
-(IBAction)startGoBackward:(id)sender;
-(IBAction)stopGoBackward:(id)sender;
-(IBAction)startRotateLeft:(id)sender;
-(IBAction)stopRotateLeft:(id)sender;
-(IBAction)startRotateRight:(id)sender;
-(IBAction)stopRotateRight:(id)sender;
-(IBAction)startRotateLeft90:(id)sender;
-(IBAction)stopRotateLeft90:(id)sender;
-(IBAction)startRotateRight90:(id)sender;
-(IBAction)stopRotateRight90:(id)sender;

@end

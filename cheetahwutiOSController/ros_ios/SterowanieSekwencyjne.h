//
//  SterowanieSekwencyjne.h
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

struct commandSequence
{
    int type; // 0 - do przodu, 1 - do tyłu, 2 - w prawo, 3 - w lewo
    int count; // ilosc sekwencji
};

@interface SterowanieSekwencyjne : UIViewController <UITableViewDelegate,UITableViewDataSource, UIPickerViewDelegate, UIPickerViewDataSource>
{
    NSMutableArray *tableArray;
    NSArray *line, *skret;
    NSString * lineMove, *skretMove;
    int lineValue, counter, statusOfSim; // statusOfSim: 0 - start 1 - stopping 2 - stopped
    double skretValue;
    NSTimer *timerInit;
    dispatch_source_t timerGo, timerRot;
    BOOL ready, reverseMain, timerEnd;
    std::vector<Leg*> lineLeg;
    std::vector<RotateLeg*> rotateLeg;
    ros::NodeHandle n;
    std::vector<commandSequence> cmdSeq;
}
@property(retain) IBOutlet UITableView *tableViewIn;
@property (weak, nonatomic) IBOutlet UIPickerView *picker;
@property (weak, nonatomic) IBOutlet UIPickerView *picker2;
@property(retain) IBOutlet UILabel *countOfSeq;
@property(retain) IBOutlet UILabel *countOfKat;
@property(retain) IBOutlet UILabel *status;
@property(retain) IBOutlet UIStepper *stepper1;
@property(retain) IBOutlet UIStepper *stepper2;
@property(retain) IBOutlet UIButton *addLineButton;
@property(retain) IBOutlet UIButton *addKatButton;
@property(retain) IBOutlet UIButton *startSimButton;
@property(retain) IBOutlet UIButton *stopSimButton;
-(IBAction)addLine:(id)sender;
-(IBAction)addKat:(id)sender;
-(IBAction)stepperSeq:(id)sender;
-(IBAction)stepperKat:(id)sender;
-(IBAction)startSim:(id)sender;
-(IBAction)stopSim:(id)sender;
@end
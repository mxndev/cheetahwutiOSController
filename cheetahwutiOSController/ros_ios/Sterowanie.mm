//
//  Sterowanie.m
//  ros_ios
//
//  Created by Mikołaj-iMac on 14.12.2015.
//  Copyright © 2015 Mikołaj Płachta. All rights reserved.
//

#import "Sterowanie.h"

@interface Sterowanie ()

@end

@implementation Sterowanie

@synthesize display, startButtonForward, stopButtonForward, startButtonReverse, stopButtonReverse, startRotateLeft, stopRotateLeft, startRotateRight, stopRotateRight, startRotateLeft90, stopRotateLeft90, startRotateRight90, stopRotateRight90;

- (void)viewDidLoad {
    [super viewDidLoad];
    counter = 0;
    sekwencje = -1;
    isStop = NO;
    timerInit = [NSTimer scheduledTimerWithTimeInterval:0.1 target:self selector:@selector(caller:) userInfo:nil repeats:YES];
    // Do any additional setup after loading the view.
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

-(void) callerForwardBackward:(NSTimer*)timer{
    //do your action
    
    if(ros::ok())
    {
        for(int i = 0; i < lineLeg.size(); i++)
        {
            lineLeg[i]->doStep(reverse);
        }
    }
    
    if((((lineLeg[0]->GetTimer(reverse) == 50) && (isStop)) && (!reverse)) || (((lineLeg[0]->GetTimer(reverse) == 350) && (isStop)) && (reverse)))
    {
        [timerGo invalidate];
        display.text = @"Zatrzymano";
        isStop = NO;
        [startButtonForward setEnabled:YES];
        [startButtonReverse setEnabled:YES];
        [startRotateLeft setEnabled:YES];
        [startRotateRight setEnabled:YES];
        [startRotateLeft90 setEnabled:YES];
        [startRotateRight90 setEnabled:YES];
        [stopButtonForward setEnabled:NO];
        [stopButtonReverse setEnabled:NO];
        [stopRotateLeft setEnabled:NO];
        [stopRotateRight setEnabled:NO];
        [stopRotateLeft90 setEnabled:NO];
        [stopRotateRight90 setEnabled:NO];
    }
    
}

-(void) callerRotate:(NSTimer*)timer{
    //do your action
    
    if(ros::ok())
    {
        for(int i = 0; i < rotateLeg.size(); i++)
        {
            rotateLeg[i]->doStep(reverse);
        }
    }
    
    if(rotateLeg[0]->GetTimer(reverse) == 0)
    {
        if(((sekwencje != -1) && (sekwencje >= 10)) || ((sekwencje == -1) && (isStop)))
        {
            sekwencje = -1;
            [timerRot invalidate];
            display.text = @"Zatrzymano";
            isStop = NO;
            [startButtonForward setEnabled:YES];
            [startButtonReverse setEnabled:YES];
            [startRotateLeft setEnabled:YES];
            [startRotateRight setEnabled:YES];
            [startRotateLeft90 setEnabled:YES];
            [startRotateRight90 setEnabled:YES];
            [stopButtonForward setEnabled:NO];
            [stopButtonReverse setEnabled:NO];
            [stopRotateLeft setEnabled:NO];
            [stopRotateRight setEnabled:NO];
            [stopRotateLeft90 setEnabled:NO];
            [stopRotateRight90 setEnabled:NO];
        } else {
            if(sekwencje != -1)
            {
                sekwencje++;
            }
        }
    }
    
}

-(void) caller:(NSTimer*)timer{
    //do your action
    if(counter == 0)
    {
        lineLeg.push_back(new Leg(LegTypes::FrontLeft, 50, 350, n));
        rotateLeg.push_back(new RotateLeg(LegTypes::FrontLeft, 690, 0, n));
    } else if(counter == 1) {
        lineLeg.push_back(new Leg(LegTypes::FrontRight, 250, 150, n));
        rotateLeg.push_back(new RotateLeg(LegTypes::FrontRight, 0, 690, n));
    } else if(counter == 2) {
        lineLeg.push_back(new Leg(LegTypes::RearLeft, 150, 250, n));
        rotateLeg.push_back(new RotateLeg(LegTypes::RearLeft, 460, 230, n));
    } else if(counter == 3) {
        lineLeg.push_back(new Leg(LegTypes::RearRight, 350, 50, n));
        rotateLeg.push_back(new RotateLeg(LegTypes::RearRight, 230, 460, n));
    } else if(counter == 4) {
        [timerInit invalidate];
        display.text = @"Inicjalizacja zakończona";
        [startButtonForward setEnabled:YES];
        [startButtonReverse setEnabled:YES];
        [startRotateRight setEnabled:YES];
        [startRotateLeft setEnabled:YES];
        [startRotateRight90 setEnabled:YES];
        [startRotateLeft90 setEnabled:YES];
    }
    counter++;
}

-(IBAction)stopGoForward:(id)sender
{
    isStop = YES;
    display.text = @"Zatrzymywanie";
    [stopButtonForward setEnabled:NO];
}

-(IBAction)startGoForward:(id)sender
{
    reverse = NO;
    [startButtonForward setEnabled:NO];
    [startButtonReverse setEnabled:NO];
    [startRotateLeft setEnabled:NO];
    [startRotateRight setEnabled:NO];
    [startRotateLeft90 setEnabled:NO];
    [startRotateRight90 setEnabled:NO];
    [stopButtonForward setEnabled:YES];
    
    display.text = @"Ruch do przodu";
    
    timerGo = [NSTimer scheduledTimerWithTimeInterval:0.02 target:self selector:@selector(callerForwardBackward:) userInfo:nil repeats:YES];
}

-(IBAction)stopGoBackward:(id)sender
{
    isStop = YES;
    display.text = @"Zatrzymywanie";
    [stopButtonReverse setEnabled:NO];
}

-(IBAction)startGoBackward:(id)sender
{
    reverse = YES;
    [startButtonForward setEnabled:NO];
    [startButtonReverse setEnabled:NO];
    [startRotateLeft setEnabled:NO];
    [startRotateRight setEnabled:NO];
    [startRotateLeft90 setEnabled:NO];
    [startRotateRight90 setEnabled:NO];
    [stopButtonReverse setEnabled:YES];
    
    display.text = @"Ruch do tyłu";
    
    timerGo = [NSTimer scheduledTimerWithTimeInterval:0.02 target:self selector:@selector(callerForwardBackward:) userInfo:nil repeats:YES];
}

-(IBAction)stopRotateLeft:(id)sender
{
    isStop = YES;
    display.text = @"Zatrzymywanie";
    [stopRotateLeft setEnabled:NO];
}

-(IBAction)startRotateLeft:(id)sender
{
    reverse = YES;
    [startButtonForward setEnabled:NO];
    [startButtonReverse setEnabled:NO];
    [startRotateLeft setEnabled:NO];
    [startRotateRight setEnabled:NO];
    [startRotateLeft90 setEnabled:NO];
    [startRotateRight90 setEnabled:NO];
    [stopRotateLeft setEnabled:YES];
    
    display.text = @"Skręcanie w lewo";
    
    timerRot = [NSTimer scheduledTimerWithTimeInterval:0.02 target:self selector:@selector(callerRotate:) userInfo:nil repeats:YES];
}

-(IBAction)stopRotateRight:(id)sender
{
    isStop = YES;
    display.text = @"Zatrzymywanie";
    [stopRotateRight setEnabled:NO];
}

-(IBAction)startRotateRight:(id)sender
{
    reverse = NO;
    [startButtonForward setEnabled:NO];
    [startButtonReverse setEnabled:NO];
    [startRotateLeft setEnabled:NO];
    [startRotateRight setEnabled:NO];
    [startRotateLeft90 setEnabled:NO];
    [startRotateRight90 setEnabled:NO];
    [stopRotateRight setEnabled:YES];
    
    display.text = @"Skręcanie w prawo";
    
    timerRot = [NSTimer scheduledTimerWithTimeInterval:0.02 target:self selector:@selector(callerRotate:) userInfo:nil repeats:YES];
}

-(IBAction)stopRotateLeft90:(id)sender
{
    sekwencje = -1;
    isStop = YES;
    display.text = @"Zatrzymywanie";
    [stopRotateLeft90 setEnabled:NO];
}

-(IBAction)startRotateLeft90:(id)sender
{
    sekwencje = 0;
    reverse = YES;
    [startButtonForward setEnabled:NO];
    [startButtonReverse setEnabled:NO];
    [startRotateLeft setEnabled:NO];
    [startRotateRight setEnabled:NO];
    [startRotateLeft90 setEnabled:NO];
    [startRotateRight90 setEnabled:NO];
    [stopRotateLeft90 setEnabled:YES];
    
    display.text = @"Skręcanie w lewo o 90 stopni";
    
    timerRot = [NSTimer scheduledTimerWithTimeInterval:0.02 target:self selector:@selector(callerRotate:) userInfo:nil repeats:YES];
}

-(IBAction)stopRotateRight90:(id)sender
{
    sekwencje = -1;
    isStop = YES;
    display.text = @"Zatrzymywanie";
    [stopRotateRight90 setEnabled:NO];
}

-(IBAction)startRotateRight90:(id)sender
{
    sekwencje = 0;
    reverse = NO;
    [startButtonForward setEnabled:NO];
    [startButtonReverse setEnabled:NO];
    [startRotateLeft setEnabled:NO];
    [startRotateRight setEnabled:NO];
    [startRotateLeft90 setEnabled:NO];
    [startRotateRight90 setEnabled:NO];
    [stopRotateRight90 setEnabled:YES];
    
    display.text = @"Skręcanie w prawo o 90 stopni";
    
    timerRot = [NSTimer scheduledTimerWithTimeInterval:0.02 target:self selector:@selector(callerRotate:) userInfo:nil repeats:YES];
}

/*
#pragma mark - Navigation

// In a storyboard-based application, you will often want to do a little preparation before navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    // Get the new view controller using [segue destinationViewController].
    // Pass the selected object to the new view controller.
}
*/

@end

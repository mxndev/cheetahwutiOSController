//
//  SterowanieSekwencyjne.m
//  ros_ios
//
//  Created by Mikołaj-iMac on 14.12.2015.
//  Copyright © 2015 Mikołaj Płachta. All rights reserved.
//

#import "SterowanieSekwencyjne.h"

@interface SterowanieSekwencyjne()
-(void) goLine:(BOOL)reverse seq:(int)seqence;
-(void) goRotate:(BOOL)reverse seq:(int)seqence;
@end

@implementation SterowanieSekwencyjne

@synthesize picker, picker2, countOfSeq, countOfKat, stepper1, stepper2, addLineButton, addKatButton, startSimButton, stopSimButton, status;
- (void)viewDidLoad {
    [super viewDidLoad];

    tableArray = [NSMutableArray array];
    
    line = [[NSArray alloc] initWithObjects:@"przodu", @"tyłu", nil];
    skret = [[NSArray alloc] initWithObjects:@"prawo", @"lewo", nil];
    
    [picker selectRow:0 inComponent:0 animated:NO];
    [picker2 selectRow:0 inComponent:0 animated:NO];
    
    stepper1.maximumValue = 30.0;
    stepper2.maximumValue = 360.8;
    stepper2.stepValue = 8.20;
    [addLineButton setEnabled:NO];
    [addKatButton setEnabled:NO];
    [startSimButton setEnabled:NO];
    [stopSimButton setEnabled:NO];
    
    lineMove = @"przodu";
    skretMove = @"prawo";
    lineValue = 0;
    skretValue = 0.0;
    ready = YES;
    timerEnd = NO;
    timerInit = [NSTimer scheduledTimerWithTimeInterval:0.1 target:self selector:@selector(caller:) userInfo:nil repeats:YES];
    statusOfSim = 2;
    // Do any additional setup after loading the view.
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

dispatch_source_t CreateDispatchTimer(uint64_t interval, uint64_t leeway, dispatch_queue_t queue, dispatch_block_t block)
{
    dispatch_source_t timer = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, queue);
    if (timer)
    {
        dispatch_source_set_timer(timer, dispatch_walltime(NULL, 0), interval, leeway);
        dispatch_source_set_event_handler(timer, block);
        dispatch_resume(timer);
    }
    return timer;
}

-(void) callerForwardBackward
{
    //do your action
    
    if(ros::ok())
    {
        for(int i = 0; i < lineLeg.size(); i++)
        {
            lineLeg[i]->doStep(reverseMain);
        }
    }
    
    if(((lineLeg[0]->GetTimer(reverseMain) == 50) && (!reverseMain)) || ((lineLeg[0]->GetTimer(reverseMain) == 350) && (reverseMain)))
    {
        dispatch_source_cancel(timerGo);
        timerEnd = YES;
    }
    
}

-(void) callerRotate
{
    //do your action
    
    if(ros::ok())
    {
        for(int i = 0; i < rotateLeg.size(); i++)
        {
            rotateLeg[i]->doStep(reverseMain);
        }
    }
    
    if(rotateLeg[0]->GetTimer(reverseMain) == 0)
    {
        dispatch_source_cancel(timerRot);
        timerEnd = YES;
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
        status.text = @"Inicjalizacja zakończona";
        ready = YES;
    }
    counter++;
}

-(void) goLine:(BOOL)reverse seq:(int)seqence
{
    timerEnd = YES;
    int i = 0;
    reverseMain = reverse;
    while(true)
    {
        if(timerEnd)
        {
            if(i >= seqence)
            {
                break;
            }
            timerGo = CreateDispatchTimer(0.02 * NSEC_PER_SEC, (0.02 * NSEC_PER_SEC) / 10, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
                [self callerForwardBackward];
            });
            timerEnd = NO;
            i++;
        }
    }
}

-(void) goRotate:(BOOL)reverse seq:(int)seqence
{
    timerEnd = YES;
    int i = 0;
    reverseMain = reverse;
    while(true)
    {
        if(timerEnd)
        {
            if(i >= seqence)
            {
                break;
            }
            timerRot = CreateDispatchTimer(0.02 * NSEC_PER_SEC, (0.02 * NSEC_PER_SEC) / 10, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
                [self callerRotate];
            });
            timerEnd = NO;
            i++;
        }
    }
}

-(void) runSim
{
    for(int i = 0; i < cmdSeq.size(); i++)
    {
        if(cmdSeq[i].type == 0)
        {
            [self goLine:NO seq:cmdSeq[i].count];
        } else if(cmdSeq[i].type == 1) {
            [self goLine:YES seq:cmdSeq[i].count];
        } else if(cmdSeq[i].type == 2) {
            [self goRotate:NO seq:cmdSeq[i].count];
        } else if(cmdSeq[i].type == 3) {
            [self goRotate:YES seq:cmdSeq[i].count];
        }
        if(statusOfSim == 1)
        {
            dispatch_async(dispatch_get_main_queue(), ^{
                [self.tableViewIn selectRowAtIndexPath:[NSIndexPath indexPathForRow:cmdSeq.size() inSection:0] animated:NO scrollPosition:0];
                [startSimButton setEnabled:YES];
                if(lineValue > 0.0)
                {
                    [addLineButton setEnabled:YES];
                } else {
                    [addLineButton setEnabled:NO];
                }
                if(skretValue > 0.0)
                {
                    [addKatButton setEnabled:YES];
                } else {
                    [addKatButton setEnabled:NO];
                }
                [stepper1 setEnabled:YES];
                [stepper2 setEnabled:YES];
                status.text = @"Symulacja przerwana";
            });
            statusOfSim = 2;
            break;
        }
        dispatch_async(dispatch_get_main_queue(), ^{
            [self.tableViewIn selectRowAtIndexPath:[NSIndexPath indexPathForRow:(i+1) inSection:0] animated:NO scrollPosition:0];
        });
    }
    if(statusOfSim != 2)
    {
        dispatch_async(dispatch_get_main_queue(), ^{
            [self.tableViewIn selectRowAtIndexPath:[NSIndexPath indexPathForRow:cmdSeq.size() inSection:0] animated:NO scrollPosition:0];
            [startSimButton setEnabled:YES];
            if(lineValue > 0.0)
            {
                [addLineButton setEnabled:YES];
            } else {
                [addLineButton setEnabled:NO];
            }
            if(skretValue > 0.0)
            {
                [addKatButton setEnabled:YES];
            } else {
                [addKatButton setEnabled:NO];
            }
            [stopSimButton setEnabled:NO];
            [stepper1 setEnabled:YES];
            [stepper2 setEnabled:YES];
            status.text = @"Symulacja zakończona";
        });
        statusOfSim = 2;
    }
}

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView {
    return 1;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
    self.tableViewIn = tableView;
    return tableArray.count;
}

- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath {
    static NSString *CellIdentifier = @"Cell";
    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:CellIdentifier];
    if (cell == nil) {
        cell = [[UITableViewCell alloc] initWithStyle:UITableViewCellStyleDefault  reuseIdentifier:CellIdentifier];
    }
    
    NSString *item = [tableArray objectAtIndex:indexPath.row];
    cell.textLabel.text = item;
    return cell;
}

-(void)tableView:(UITableView *)tableView commitEditingStyle:(UITableViewCellEditingStyle)editingStyle forRowAtIndexPath:(NSIndexPath *)indexPath {
    if (editingStyle == UITableViewCellEditingStyleDelete) {
        if(statusOfSim == 2)
        {
            [tableArray removeObjectAtIndex:indexPath.row];
            [tableView deleteRowsAtIndexPaths:@[indexPath] withRowAnimation:UITableViewRowAnimationFade];
            cmdSeq.erase(cmdSeq.begin() + indexPath.row);
            if((tableArray.count > 0) && (ready))
            {
                [startSimButton setEnabled:YES];
            } else {
                [startSimButton setEnabled:NO];
            }
        } else {
            UIAlertView * alert = [[UIAlertView alloc] initWithTitle:@"Błąd!" message:@"Nie możesz usunąć sekwencji, kiedy trwa symulacja." delegate:self cancelButtonTitle:@"Ok" otherButtonTitles:nil];
            [alert show];
        }
    }
}

- (NSInteger)numberOfComponentsInPickerView:(UIPickerView *)pickerView {
    return 1;
}

-(void)pickerView:(UIPickerView *)pickerView didSelectRow:(NSInteger)row inComponent:(NSInteger)component
{
    if(pickerView == picker)
    {
        lineMove = [line objectAtIndex:row];
    } else {
        skretMove = [skret objectAtIndex:row];
    }
}

-(NSInteger)pickerView:(UIPickerView *)pickerView numberOfRowsInComponent:(NSInteger)component;
{
    if(pickerView == picker)
    {
        return [line count];
    } else {
        return [skret count];
    }
}

-(NSString *)pickerView:(UIPickerView *)pickerView titleForRow:(NSInteger)row forComponent:(NSInteger)component;
{
    if(pickerView == picker)
    {
        return [line objectAtIndex:row];
    } else {
        return [skret objectAtIndex:row];
    }
}


-(IBAction)addLine:(id)sender
{
    [tableArray addObject:[NSString stringWithFormat:@"Ruch do %@ - ilość sekwencji: %d", lineMove, (int)lineValue]];
    [self.tableViewIn reloadData];
    if((tableArray.count > 0) && (ready))
    {
        [startSimButton setEnabled:YES];
    } else {
        [startSimButton setEnabled:NO];
    }
    if([lineMove isEqual: @"przodu"])
    {
        cmdSeq.push_back((commandSequence){0, lineValue});
    } else if([lineMove isEqual: @"tyłu"]) {
        cmdSeq.push_back((commandSequence){1, lineValue});
    }
}

-(IBAction)addKat:(id)sender
{
    [tableArray addObject:[NSString stringWithFormat:@"Skręcaj w %@ - ilość stopni: %.1f", skretMove, (double)skretValue]];
    [self.tableViewIn reloadData];
    if((tableArray.count > 0) && (ready))
    {
        [startSimButton setEnabled:YES];
    } else {
        [startSimButton setEnabled:NO];
    }
    if([skretMove isEqual: @"prawo"])
    {
        cmdSeq.push_back((commandSequence){2, (skretValue/8.2)});
    } else if([skretMove isEqual: @"lewo"]) {
        cmdSeq.push_back((commandSequence){3, (skretValue/8.2)});
    }
}

-(IBAction)stepperSeq:(UIStepper *)sender
{
    lineValue = [sender value];
    if(lineValue > 0.0)
    {
        [addLineButton setEnabled:YES];
    } else {
        [addLineButton setEnabled:NO]; 
    }
    [countOfSeq setText:[NSString stringWithFormat:@"%d", (int)lineValue]];
}

-(IBAction)stepperKat:(UIStepper *)sender
{
    skretValue = [sender value];
    if(skretValue > 0.0)
    {
        [addKatButton setEnabled:YES];
    } else {
        [addKatButton setEnabled:NO];
    }
    [countOfKat setText:[NSString stringWithFormat:@"%.1f", (double)skretValue]];
}

-(IBAction)startSim:(UIStepper *)sender
{
    [self.tableViewIn selectRowAtIndexPath:[NSIndexPath indexPathForRow:0 inSection:0] animated:NO scrollPosition:0];
    status.text = @"Symulacja rozpoczęta";
    statusOfSim = 0;
    [startSimButton setEnabled:NO];
    [addLineButton setEnabled:NO];
    [addKatButton setEnabled:NO];
    [stopSimButton setEnabled:YES];
    [stepper1 setEnabled:NO];
    [stepper2 setEnabled:NO];
    dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
        [self runSim];
    });
}

-(IBAction)stopSim:(UIStepper *)sender
{
    status.text = @"Przerywanie symulacji";
    [stopSimButton setEnabled:NO];
    statusOfSim = NO;
    statusOfSim = 1;
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

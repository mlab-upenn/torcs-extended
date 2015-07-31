/***************************************************************************

file                 : autonet.cpp
created              : Sat Nov 13 23:19:31 EST 2010
copyright            : (C) 2002 Utsav

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>
#include "serial.h"
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
//#include <sys/time.h>
#include "driver.h"
#include <errno.h>
#include "timer.h"
#include <iostream>
using namespace std;

#define NBBOTS 2
static Driver *driver[NBBOTS];
static tTrack *curTrack;
 int serialfd;
int flag = 0;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newrace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt);


static char *botname[NBBOTS] = { "autonet","autonet2" };
static tCarElt *car,*car2;
static float accel = 0, accel2 = 0;
static float brake[4] = {0}, brake2[4]={0};
static float clutch = 0,clutch2 = 0;
static float angle = 0 , angle2 = 0;
static int gear = 0 , gear2 = 0,radarAngle=5;
//uint16_t distance;
 int dist1 = 0, dist2 = 0;
 int dist_cars = 0,start_dist = 0;
 double x1 = 0,x2 = 0,Y1 = 0,Y2 = 0,carAngle=0; 
int print_count = 0;
int check_flag = 0;

double pos1,pos2;
static timer_t timerid;
volatile static bool signalStopSendData = false;
volatile static bool serialDataStopped = true;

/*
 * Module entry point
 */
extern "C" int autonet(tModInfo *modInfo)
{
    int i;

    //cout<<"Foo 1 " << endl;

    memset(modInfo, 0, 10*sizeof(tModInfo));

   
    #if 1
    for (i = 0; i < NBBOTS; i++) {

     modInfo[i].name    = botname[i];  /* name of the module (short) */
     modInfo[i].desc    = "";          /* description of the module (can be long) */
     modInfo[i].fctInit = InitFuncPt;  /* init function */
     modInfo[i].gfId    = ROB_IDENT;	  /* supported framework version */
     modInfo[i].index   = i;           /* indices from 0 to 9 */

    }
    #endif

      // Old code
    #if 0
    modInfo->name    = "autonet";		/* name of the module (short) */
    modInfo->desc    = "";	            /* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    #endif
 
    return 0;
}

/* Module interface initialization. */
static int InitFuncPt(int index, void *pt)
{
     //cout<<" Init FuncPt for index "<< index <<endl;
    tRobotItf *itf  = (tRobotItf *)pt;

     // New code
     #if 1
     /* create robot instance for index */
    driver[index] = new Driver(index);
    itf->rbNewTrack = initTrack; /* Give the robot the track view called */
    itf->rbNewRace  = newrace;   /* Start a new race */
    itf->rbDrive    = drive;     /* Drive during race */
    itf->rbEndRace  = endrace;   /* End of the current race */
    itf->rbShutdown = shutdown;  /* Called before the module is unloaded */
    itf->index      = index;     /* Index used if multiple interfaces */

    // Old code

    #endif

    #if 0 
    itf->rbNewTrack = initTrack; /* Give the robot the track view called */
                                 /* for every track change or new race */
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    #endif 
    return 0;
}

/* Called for every track change or new race. */
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
    // New code
   
    driver[index]->initTrack(track, carHandle, carParmHandle, s);
    curTrack = track;
    cout<<" Init Track for index "<< index <<endl;

    // Old Code
    #if 0
    *carParmHandle = NULL;
    
    #endif
}

int virtual_sensor2(double pos1,double pos2)
{ 
   if  (((pos1 - 2.0) < pos2) && ((pos1 + 2.0) > pos2))
       return 1;
   
   else
      return 0;

}

uint16_t mod_virtual_sensor(double x1,double y1,double x2,double y2)
{
  double delta_y,delta_x,theta;
  double temp ;
  delta_x = x1 - x2;
  delta_y = y1 - y2; 
  temp = (delta_y / delta_x);
  theta = atan(temp);
  theta = theta * 57.2957 ; // conversion to degrees 
  //  cout<<"Theta value is " << theta << "degrees" << endl;
 // cout<<abs(theta)<<"  ";
  if(dist_cars < 100.0) 
  {
    if((abs(theta) > 85 && abs(theta) < 90)  | (abs(theta) >= 0 && abs(theta) < 5) ) 
     {
      //cout<<"D" ;//<< //endl;
  
      return 1;
     }
    else
     {
       //cout<<"N";//<<endl;
      //cout<<" Not Detectable " << endl;
      return 0;
     }
  }
  else
  {
   //cout<<"O";
    return 0;
  }
}

int virtual_sensor(double x1,double Y1,double x2,double y2, double carAngle)
{
  double delta_y,delta_x,theta;
  double temp,theta1 ;
  delta_x = x1 - x2;
  delta_y = Y1 - y2; 
  temp = (delta_y / delta_x);
  theta = atan(temp);
  theta = theta * 57.2957 ; // conversion to degrees 
  //cout<<"Theta value is " << theta << "degrees" << endl;

  theta1=(abs(carAngle*57.2957)%90);
  //cout<<"*"<<abs(abs(theta)-theta1)<<"%"<<(90-(abs(abs(theta)-theta1)));
  #if 0 
  if(dist_cars > 100.0) 
  {
  if((abs(theta) > 85 && abs(theta) < 90)  | (abs(theta) >= 0 && abs(theta) < 5) )
     return 1;
  else
       return 0;
  }
  else
      return 0;
  #endif
  //cout<<abs(abs(theta)-theta1ud);
  #if 1
  if((abs(abs(theta)-theta1)<radarAngle) | ((90-abs(abs(theta)-theta1))<radarAngle))
 	return 1;
  else
	return 0;
  #endif
}
void serialData(int signum)
{
    uint8_t a[512]; // Large buffer to read in all pending bytes from serial
    uint8_t calcChkSum = 0;
    uint8_t b[19];  // 	Changed from 12 to 15 to send distance as 2 bytes
    int i;
    //cout<<"Signal Number is " << signum << endl;

   
   // print_count++;
    #if 1
    if(signalStopSendData)
    {
         
        serialDataStopped = true;
        return;
    }
  
    if(serialPortRead(serialfd, a, 1) == 1) {
        if(a[0] == 0xAA) {
            if(serialPortRead(serialfd, a, 1) == 1) {
                if(a[0] == 0xCC) {
                    calcChkSum = 0xAA ^ 0xCC;
                    if(serialPortRead(serialfd, a, 512) >= 9) {
                        for(i = 0; i < 9; i++) {
                            calcChkSum = calcChkSum ^ a[i];
                        }

                        if(calcChkSum == a[9]) {
                            accel = a[0]/100.0;
                            brake[0] = a[1]/100.0;
                            brake[1] = a[2]/100.0;
                            brake[2] = a[3]/100.0;
                            brake[3] = a[4]/100.0;
                            angle = (int8_t)a[5]/50.0;
                            gear = (int8_t)a[6];
                            clutch = a[7]/100.0;
                           // radarAngle = 10;
                            radarAngle=a[8]; 

    
                       }
                    }
                }
            }
        }
    }

    #endif

 //   cout<< radarAngle;
    b[0] = 0xAA;
    b[1] = 0xCC;
    b[2] = car->_speed_x;
    uint16_t rpm = car->_enginerpm;
    b[3] = (rpm & 0xFF00) >> 8;
    b[4] = rpm & 0xFF;
 
    b[5] = car->_wheelSpinVel(FRNT_LFT) * car->_wheelRadius(FRNT_LFT);
    b[6] = car->_wheelSpinVel(FRNT_RGT) * car->_wheelRadius(FRNT_RGT);
    b[7] = car->_wheelSpinVel(REAR_LFT) * car->_wheelRadius(REAR_LFT);
    b[8] = car->_wheelSpinVel(REAR_RGT) * car->_wheelRadius(REAR_RGT);

    b[9] = car->_yaw_rate * 10;

    

    if(dist2 >=  dist1)
      dist_cars = abs(dist2-dist1);
    else
      dist_cars = (((int)(curTrack->length) - abs(dist2-dist1)));

  // Code for second type of virtual sensor using position and width(linear)

  #if 1
  if(dist_cars < 160) 
   {
      if(virtual_sensor2(pos1,pos2)) 
      {
      // cout<<"Yeah  : Detectable " << endl;
   
      }
   else
      {
       dist_cars = 2500.0;
      // cout<<"Oh no : Not detectable " << endl;
      }
   }
   else 
    {
       dist_cars = 2500.0;
       //cout<<"No issues, car not in range " << endl;
    } 
   #endif

    //cout<< dist_cars << endl;

   // Code for modified virtual sensor taking into consideration angles
  
   #if 0
   if(dist_cars < 160) 
   {
     if(!(virtual_sensor(x1,Y1,x2,Y2,carAngle)))
       dist_cars = 2500.0;
   }
   else
       dist_cars = 2500.0;  
   #endif

    uint16_t distance = dist_cars;
    b[10] =(uint8_t)((distance & 0xFF00) >> 8); // Lower 8 bytes of distance 
    b[11] = (uint8_t)(distance & 0xFF);         // Higher 8 bytes of distance 
    
    
    //cout<<"Distance from start is " << dist1 << endl;
    b[12] =(((int)(dist1) & 0xFF00) >> 8); // Lower 8 bytes of start distance of Car1 
    b[13] = ((int)(dist1)  & 0xFF);         // Higher 8 bytes of start distance of Car1 
    //b[12] =(uint8_t)(((uint8_t)(car->_trkPos.seg->lgfromstart) & 0xFF00) >> 8); // Lower 8 bytes of start distance of Car1 
    //b[13] = (uint8_t)((uint8_t)(car->_trkPos.seg->lgfromstart) & 0xFF);         // Higher 8 bytes of start distance of Car1 
    
    #if 0
    b[14] =(((int)(x1) & 0xFF00) >> 8); // Lower 8 bytes of X position of Car1 
    b[15] = ((int)(x1)  & 0xFF);         // Higher 8 bytes of X position of Car1 
    b[16] =(((int)(Y1) & 0xFF00) >> 8); // Lower 8 bytes of Y position of Car1 
    b[17] = ((int)(Y1)  & 0xFF);         // Higher 8 bytes of Y position of Car1 
    

    
    b[18] =(((int)(x2) & 0xFF00) >> 8); // Lower 8 bytes of X position of Car2 
    b[19] = ((int)(x2)  & 0xFF);         // Higher 8 bytes of X position of Car2
    b[20] =(((int)(Y2) & 0xFF00) >> 8); // Lower 8 bytes of Y position of Car2
    b[21] = ((int)(Y2)  & 0xFF);         // Higher 8 bytes of Y position of Car2
    #endif

   b[14] = (int)(x1/4);
   b[15] = (int)(Y1/2);
   b[16] = (int)(x2/4);
   b[17] = (int)(Y2/2);
 
   #if 0
   if(car2 != NULL)
      b[18] = car2->_speed_x;
   else
      b[18] = 0;
    calcChkSum = 0;
   #endif

    for(i = 0; i <= 17; i++){
    //for(i = 0; i <= 13; i++){   // Changed from 10 to 13
        calcChkSum = calcChkSum ^ b[i];
    }
   
    b[18] = calcChkSum;  

    serialPortWrite(serialfd, b, 19);  // Changed from 12 to 15 for sending distance as 10 bytes
}


/* Start a new race. */
static void newrace(int index, tCarElt* car_local, tSituation *s)
{

   
   cout<<" New Race for index "<< index <<endl;
   #if 1
   if(index == 0) 
    {   // New code
    
    //fcntl(serialfd,F_SETFL,O_NONBLOCK);
    serialfd = serialPortOpen("/dev/ttyUSB0", 115200);
    car = car_local;
    //timerid = timerInit(serialData, 10000000000);
    timerid = timerInit(serialData, 2000000);
    serialDataStopped = false;
    enableTimerSignal(); 
   }
   #endif

   #if 0
   else if(index == 1) {   // New code
   //  cout<<"New Race : Car2 local value is " << car_local;
    serialfd = serialPortOpen("/dev/ttyUSB0", 115200);
    car2 = car_local;
    
    //timerid = timerInit(serialData, 2000000);
    //serialDataStopped = false;
    //enableTimerSignal();
   }
   #endif
  #if 1
  else   {          // New code
     car2 = car_local;
     driver[index]->newRace(car2, s);
  
  }
  #endif

}

/* Drive during race. */
static void drive(int index, tCarElt* car, tSituation *s)
{

     //float angle;
     // static double x1,x2,y1,y2;   
     memset((void *)&car->ctrl, 0, sizeof(tCarCtrl));

   //cout<<" Drive for index "<< index <<endl;
    #if 1
   if(index == 0) 
   {
    
     // set the values
     car->_steerCmd = angle;
     car->_gearCmd = gear;
     car->_accelCmd = accel;
     car->_individualBrakes = true;
     car->_individualBrakeCmd[0] = brake[0];
     car->_individualBrakeCmd[1] = brake[1];
     car->_individualBrakeCmd[2] = brake[2];
     car->_individualBrakeCmd[3] = brake[3];
     car->_brakeCmd = (brake[0] + brake[1] + brake[2] + brake[3])/4.0; // Just for display in TORCS
     car->_clutchCmd = clutch;
     //cout<<"before dist is " << endl;
    
     dist1 = car->_distFromStartLine;//_trkPos.seg->lgfromstart;
     x1 =(double)car->_pos_X;
     Y1 = car->_pos_Y;
     carAngle=car->_yaw;
     //cout<<x1<<"  -  "<<Y1<<endl;
     pos1 =(double)car->_trkPos.toMiddle;
     //cout<<(abs(carAngle*57.2957)%180);
     //cout<<"Car Yaw angle is " << car->_yaw;
    // angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
     //NORM_PI_PI(angle);
     //cout<< angle * 57.2957 <<" ";
    //cout<<"Dist from centre is " <<car->_trkPos.toMiddle<<endl;
   }
   #endif

   #if 0
   else if(index == 1) 
   {
    // cout<<"Autonet1 drive " <<endl;
     // set the values
     car2->_steerCmd = angle2;
     car2->_gearCmd = gear2;
     car2->_accelCmd = accel2;
     car2->_individualBrakes = true;
     car2->_individualBrakeCmd[0] = brake2[0];
     car2->_individualBrakeCmd[1] = brake2[1];
     car2->_individualBrakeCmd[2] = brake2[2];
     car2->_individualBrakeCmd[3] = brake2[3];
     car2->_brakeCmd = (brake2[0] + brake2[1] + brake2[2] + brake2[3])/4.0; // Just for display in TORCS
     car2->_clutchCmd = clutch2;
     //dist2 = 30;
     dist2 = car2->_trkPos.seg->lgfromstart;
     Cout<<"Dist1 value is " <<dist1;
     
   }
   #endif
   #if 1
   else if(index == 1) {
     
     dist2 = car2->_distFromStartLine;//_trkPos.seg->lgfromstart;
    //cout<<"Dist2 value is " <<dist2<<endl;


     //angle = RtTrackSideTgAngleL(&(car2->_trkPos)) - car2->_yaw;
     //NORM_PI_PI(angle);
     //cout<< angle<<" ";


     driver[index]->drive(car2, s,index);
    // cout<<"Car2 : X and Y positions are  " << car2->_pos_X << "  " << car2->_pos_Y << endl;
   //cout<<"Distance from start is " << dist2<< endl;
   

   //virtual_sensor(5.0,0.0,10.0,50.0);

   x2 =(double) car2->_pos_X;
   Y2 =(double) car2->_pos_Y;
   pos2 =(double)car2->_trkPos.toMiddle;
 // cout<< car2->_yaw * 57.2957<<"  ";
   //cout<<"Car2 : X and Y positions are  " << car2->_pos_X << "  " << car2->_pos_Y << endl;
  

   //cout<<"Car 1 -  X and Y are " << x1 <<"  " << y1 << endl;
   //cout<<"Car 2  - X and Y are " << x2 <<"  " << y2 << endl;
   //int x = virtual_sensor(x1,y1,x2,y2);
   
   #if 0
  if(mod_virtual_sensor(x1,y1,x2,y2) != 2500)
     cout<<"Yeah  : Detectable " << endl;
   else
     cout<<"Oh no : Not detectable " << endl;
  // virtual_sensor(5.0,0.0,0.0,50.0);
   #endif
  
   
   
   //cout<<"Dist2 from centre is " <<car2->trkPos.toMiddle<<endl;
  
   } 
   #endif

  
   
   #if 0 
   if(dist2 >=  dist1)
      dist_cars = abs(dist2-dist1);
   else
      dist_cars = (((int)(curTrack->length) - abs(dist2-dist1)));  // 
   #endif
   
   //cout<<"Distance between cars is " << dist_cars << endl;
   //if(dist_cars > 250 )
	//dist_cars = 250;
   
   
}

/* End of the current race */
static void endrace(int index, tCarElt *car, tSituation *s)
{
    printf("endRace\n");
}

/* Called before the module is unloaded */
static void shutdown(int index)
{
   //if(index == 0)
  {  // New code
    signalStopSendData = true;
    while(!serialDataStopped);
    disableTimerSignal();
    timerEnd(timerid);
    serialPortClose(serialfd);
    printf("Done with shutdown\n");
   }
   #if 0
   else  {
    free(botname[index]);
    delete driver[index];  
   }
   #endif
}

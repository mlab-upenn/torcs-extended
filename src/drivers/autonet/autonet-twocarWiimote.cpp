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
static int gear = 0 , gear2 = 0;
 int dist1 = 0, dist2 = 0;
 int dist_cars = 0;
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

void serialData(int signum)
{
    uint8_t a[512]; // Large buffer to read in all pending bytes from serial
    uint8_t calcChkSum = 0;
    uint8_t b[20];  // Changed from 12 to 20 for second car  // Changed from 11 to 12 for autoplug 3.0 to include distance metric
    int i;
    //cout<<"Signal Number is " << signum << endl;

    #if 1
    if(signalStopSendData)
    {
         
        serialDataStopped = true;
        return;
    }

    //cout<<"Before Serial " <<endl;
    //cout<<"Length of data received is " << serialPortRead(serialfd, a, 512) << endl;
    if(serialPortRead(serialfd, a, 1) == 1) {
       // cout<<" BLAH " << endl; 
        if(a[0] == 0xAA) {
          // cout<<"Got AA " << endl;
            if(serialPortRead(serialfd, a, 1) == 1) {
                if(a[0] == 0xCC) {
                   // cout<<"Got CC " << endl;
                    calcChkSum = 0xAA ^ 0xCC;
                    if(serialPortRead(serialfd, a, 512) >= 9) {
                        for(i = 0; i < 8; i++) {
                            calcChkSum = calcChkSum ^ a[i];
                        }

                        if(calcChkSum == a[8]) {
                            cout<<"Checksum equal " << endl;
                            accel = a[0]/100.0;
                            brake[0] = a[1]/100.0;
                            brake[1] = a[2]/100.0;
                            brake[2] = a[3]/100.0;
                            brake[3] = a[4]/100.0;
                            angle = (int8_t)a[5]/50.0;
                            gear = (int8_t)a[6];
                            clutch = a[7]/100.0;
                               
                           #if 1 

                            accel2 = a[8]/100.0;
                            brake2[0] = a[9]/100.0;
                            brake2[1] = a[10]/100.0;
                            brake2[2] = a[11]/100.0;
                            brake2[3] = a[12]/100.0;
                            angle2 = (int8_t)a[13]/50.0;
                            gear2 = (int8_t)a[14];
                            clutch2 = a[15]/100.0;


                           #endif
                       }
                     // cout<<"Checksum Not equal " << endl;
                    }
                }
            }
        }
    }

    #endif

    //cout<<"End of Serial " << endl;
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

    
    b[10] = (uint8_t)dist_cars; // dist_cars & 0xff  // Changed from 9 to 10

    //cout<< " First car send " << endl;

    #if 1
    if(car2 == NULL) {
        //cout<<"Car2 is NUll " << endl;
        b[11] = 0x00;b[12] = 0x00;b[13] = 0x00;b[14] = 0x00;b[15] = 0x00;b[16] = 0x00;b[17] = 0x00;b[18] = 0x00;
    }
    #endif
   else
    {
     
    #if 1
    b[11] = (uint8_t)car2->_speed_x;
    
    uint16_t rpm2 = car2->_enginerpm;
    b[12] =(uint8_t)((rpm2 & 0xFF00) >> 8);
    b[13] = (uint8_t)rpm2 & 0xFF;
 
    b[14] = (uint8_t)car2->_wheelSpinVel(FRNT_LFT) * car2->_wheelRadius(FRNT_LFT);
    b[15] = (uint8_t)car2->_wheelSpinVel(FRNT_RGT) * car2->_wheelRadius(FRNT_RGT);
    b[16] = (uint8_t)car2->_wheelSpinVel(REAR_LFT) * car2->_wheelRadius(REAR_LFT);
    b[17] =(uint8_t)car2->_wheelSpinVel(REAR_RGT) * car2->_wheelRadius(REAR_RGT);

    b[18] = (uint8_t)(car2->_yaw_rate * 10);
    #endif
    }

    
    calcChkSum = 0;
 
    for(i = 0; i <= 18; i++){   // Changed from 10 to 18 for second car // Changed from 9 to 10
        calcChkSum = calcChkSum ^ b[i];
    }
   
    b[19] = calcChkSum;  // changed from 11 to 19 for new distance method // Changed - was b[10] earlier // 

    //if(serialPortWrite(serialfd, b, 15) == -1)
      //perror("Error");
    //cout<<"Bytes Written " << serialPortWrite(serialfd, b, 20);
    serialPortWrite(serialfd, b, 20);  // Changed from 1 for distance // Changed size from 11 to 12
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
    timerid = timerInit(serialData, 2000000);
    serialDataStopped = false;
    enableTimerSignal();
   }
   #endif

   #if 1
   else if(index == 1) {   // New code
   //  cout<<"New Race : Car2 local value is " << car_local;
    //serialfd = serialPortOpen("/dev/ttyUSB0", 115200);
    car2 = car_local;
    
    //timerid = timerInit(serialData, 2000000);
    //serialDataStopped = false;
    //enableTimerSignal();
   }
   #endif
  #if 0
  else   {          // New code
     car2 = car_local;
     driver[index]->newRace(car2, s);
  
  }
  #endif

}

/* Drive during race. */
static void drive(int index, tCarElt* car, tSituation *s)
{

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
     dist1 = car->_trkPos.seg->lgfromstart;
     //cout<<"Dist1 value is " <<dist1;
     //cout<<"Distance from start is : " << dist1  << endl;
    // cout<<"Car position is " << car->position_x;
   }
   #endif

   #if 1
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
     //out<<"Dist1 value is " <<dist1;
    // cout<<"Distance from start is : " << dist2  << endl;
   }
   #endif
   #if 0
   else if(index == 1) {
    
     dist2 = car2->_trkPos.seg->lgfromstart;
    //cout<<"Dist2 value is " <<dist2<<endl;
     
     driver[index]->drive(car2, s,index);
     
     //cout<<"Car's position is " << car->_drvPos_y << endl;
   } 
   #endif

  
   #if 1 
   if(dist2 >=  dist1)
      dist_cars = abs(dist2-dist1) / 8;
   else
      dist_cars = (((int)(curTrack->length) - abs(dist2-dist1))) / 8;  // 
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

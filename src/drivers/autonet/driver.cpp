/***************************************************************************

    file                 : driver.cpp
    created              : Wed Oct 10 01:20:19 CET 2012
    copyright            : (C) 2012 Anand Madhusoodanan

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "driver.h"
//#include "opponent.h"
#include <iostream>
using namespace std;

const float Driver::MAX_UNSTUCK_ANGLE = 30.0/180.0*PI;   /* [radians] */
const float Driver::UNSTUCK_TIME_LIMIT = 2.0;            /* [s] */
const float Driver::MAX_UNSTUCK_SPEED = 5.0;   		 /* [m/s] */
const float Driver::MIN_UNSTUCK_DIST = 3.0;    		 /* [m] */
const float Driver::G = 9.81;                 		 /* [m/(s*s)] */
const float Driver::FULL_ACCEL_MARGIN = 1.0;   		/* [m/s] */
const float Driver::SHIFT = 0.9;        		 /* [-] (% of rpmredline) */
const float Driver::SHIFT_MARGIN = 4.0;  		/* [m/s] */
long count = 0;
long dist = 0,car_dist=0;
float accel_values[10] = {0.3,0.2,0.25,0.1,0.38,0.2,0.3,0.47,0.39,0.7};
long accel_cnt = 0,accel_index_cnt=0;
double old_speed_value = 0;

Driver::Driver(int index)
{
    INDEX = index;
}

/* Called for every track change or new race. */
void Driver::initTrack(tTrack* t, void *carHandle, 
                       void **carParmHandle, tSituation *s)
{
    track = t;
    *carParmHandle = NULL;
    //cout<<"Init track for Car index :" << INDEX <<endl;
}
	
/* Start a new race. */
void Driver::newRace(tCarElt* car, tSituation *s)
{
   //cout<<"New Race for Car index :" << INDEX <<endl;
}

/* Drive during race. */
void Driver::drive(tCarElt* car, tSituation *s, int index, int speed, int lanePosition)
{
   //cout<<"New Race for Car index :" << INDEX <<endl;
   double speed_value = 0;
   
   if((speed >= 59 && speed < 170))
    { 
      speed_value = 0.1 +  (((double)speed - 59.0) * (0.01/4.0));
      old_speed_value = speed_value;
    }
   else
       speed_value =  speed_value;//old_speed_value;
 
   float steerangle;
  //  accel_cnt++;
    #if 1
    update(car, s);

    //printf("\n Drive ...%ld \n ", count++);
    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    if (isStuck(car)) {
    	car->ctrl.steer = -angle / car->_steerLock;
    	car->ctrl.gear = -1; // reverse gear
    	car->ctrl.accelCmd = speed_value; // 50% accelerator pedal
    	car->ctrl.brakeCmd = 0.0; // no brakes
    }       
   else {
	if(lanePosition == 0)
	    steerangle = angle - (car->_trkPos.toMiddle)/car->_trkPos.seg->width;
        else if(lanePosition == 1)
	    steerangle = angle - (car->_trkPos.toMiddle-4.5)/car->_trkPos.seg->width;
        else if(lanePosition == 2)
	    steerangle = angle - (car->_trkPos.toMiddle+4.5)/car->_trkPos.seg->width;
        else 
            {  /*cout<<"J";*/  steerangle = angle - (car->_trkPos.toMiddle)/car->_trkPos.seg->width;  }
	car->ctrl.steer = steerangle / car->_steerLock;
	car->ctrl.gear = getGear(car);
	car->ctrl.brakeCmd = getBrake(car);
	if (car->ctrl.brakeCmd == 0.0) {
            car->ctrl.accelCmd = speed_value;
        } 
       else {
	 car->ctrl.accelCmd = 0.0;
        }
   }
   #endif
}

/* End of the current race */
void Driver::endRace(tCarElt *car, tSituation *s)
{
}


/* Update my private data every timestep */
void Driver::update(tCarElt* car, tSituation *s)
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);
}

/* Check if I'm stuck */
bool Driver::isStuck(tCarElt* car)
{
    if (fabs(angle) > MAX_UNSTUCK_ANGLE &&
        car->_speed_x < MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
        if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*angle < 0.0) {
            return true;
        } else {
            stuck++;
            return false;
        }
    } else {
        stuck = 0;
        return false;
    }
}

  /* Compute gear */
int Driver::getGear(tCarElt *car)
{
    if (car->_gear <= 0) 
     return 1;
    float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
    float omega = car->_enginerpmRedLine/gr_up;
    float wr = car->_wheelRadius(2);

    if (omega*wr*SHIFT < car->_speed_x) {
        return car->_gear + 1;
    } 
    else {
        float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
        omega = car->_enginerpmRedLine/gr_down;
        if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) {
            return car->_gear - 1;
        }
    }
    return car->_gear;
}

float Driver::getBrake(tCarElt* car)
{
    tTrackSeg *segptr = car->_trkPos.seg;
    float currentspeedsqr = car->_speed_x*car->_speed_x;
    float mu = segptr->surface->kFriction;
    float maxlookaheaddist = currentspeedsqr/(2.0*mu*G);

    float lookaheaddist = getDistToSegEnd(car);
    float allowedspeed = getAllowedSpeed(segptr);
    if (allowedspeed < car->_speed_x) 
	return 1.0;
    segptr = segptr->next;
    while (lookaheaddist < maxlookaheaddist) {
       allowedspeed = getAllowedSpeed(segptr);
        if (allowedspeed < car->_speed_x) {
         float allowedspeedsqr = allowedspeed*allowedspeed;
         float brakedist = (currentspeedsqr - allowedspeedsqr) / (2.0*mu*G);
        if (brakedist > lookaheaddist) {
                return 1.0;
            }
        }
        lookaheaddist += segptr->length;
        segptr = segptr->next;
    }
    return 0.0;
}
  

/* Compute the allowed speed on a segment */
float Driver::getAllowedSpeed(tTrackSeg *segment)
{
    if (segment->type == TR_STR) {
        return FLT_MAX;
    } else {
        float mu = segment->surface->kFriction;
        return sqrt(mu*G*segment->radius);
    }
}

/* Compute the length to the end of the segment */
float Driver::getDistToSegEnd(tCarElt* car)
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    } else {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
    }
}

  

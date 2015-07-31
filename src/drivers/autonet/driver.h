/***************************************************************************

    file                 : driver.h
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

#ifndef _DRIVER_H_
#define _DRIVER_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>


class Driver {
    public:
        Driver(int index);

        /* callback functions called from TORCS */
        void initTrack(tTrack* t, void *carHandle,
                       void **carParmHandle, tSituation *s);
        void newRace(tCarElt* car, tSituation *s);
        void drive(tCarElt* car, tSituation *s,int index,int speed, int lanePosition);
        void endRace(tCarElt *car, tSituation *s);
  
        tCarElt *getCarPtr() { return car; }
        tTrack *getTrackPtr() { return track; }
        float getSpeed() {  return speed; }

    private:

        
        /* utility functions */
        bool isStuck(tCarElt* car);
        void update(tCarElt* car, tSituation *s);
        float getAllowedSpeed(tTrackSeg *segment);
        float getDistToSegEnd(tCarElt* car);
	float getAccel(tCarElt* car);
	float getBrake(tCarElt* car);
	int getGear(tCarElt *car);

        /* per robot global data */
        int stuck;
        float trackangle;
        float angle;
	float speed;
        tCarElt *car;		/* pointer to tCarElt struct */

        /* data that should stay constant after first initialization */
        int MAX_UNSTUCK_COUNT;
        int INDEX;

        /* class constants */
        static const float MAX_UNSTUCK_ANGLE;
        static const float UNSTUCK_TIME_LIMIT;
        static const float MAX_UNSTUCK_SPEED;
	static const float MIN_UNSTUCK_DIST;
	static const float G;
	static const float FULL_ACCEL_MARGIN;
	static const float SHIFT;
        static const float SHIFT_MARGIN;

        /* track variables */
        tTrack* track;
};
#endif // _DRIVER_H_

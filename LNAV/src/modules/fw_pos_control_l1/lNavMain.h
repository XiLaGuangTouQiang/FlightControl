#ifndef _IO_MAIN_H_
#define _IO_MAIN_H_

#include <string.h>
#include "MODEL/LNAV.h"
#include <stdint.h>
#include "transitionTurn.h"

//#define bool int

typedef struct _navPosition_t
{
    float Latitude;
    float Longitude;
} NavPosition_t;

typedef struct navAngle_t{
    float True;
    float Magnetic;
}NavAngle_t;

typedef struct _lNavStatus
{
    bool lNavArmed;
    bool lNavActive;
} LNavStatus;

typedef struct _lnavMainInput_t
{
    NavPosition_t Position;
    float TrueHeading;
    float GroundSpeed;
    NavAngle_t TrackAngle;
    float InertialAltitude;
    float TrueAirSpeed;
    double position_setpoint[8];
    LNavStatus lnav_status;
} LnavMainInput_t;

typedef struct _lnavTransTurn
{
    GeographicalP prevLegStart;
    GeographicalP cross;
    GeographicalP nextLegEnd;
    TransTurn transTurn;

} LnavTransTurn;

// typedef struct _perfLegs
// {
//     PerfFixLocation preLegFix;
//     char curLegType;
//     float curLegRadium;
//     GeographicalP curLegCenter;
//     PerfFixLocation curLegFix;
// } PerfLegs;

#ifdef  __cplusplus
extern "C" {
#endif

//static LnavMainInput_t LnavMainInput;
void lnav_init(void);
void lnavMain(LnavMainInput_t LnavMainInput,float* roll_body);

#ifdef  __cplusplus
}
#endif

#endif /* _IO_MAIN_H_ */

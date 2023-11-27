#ifndef _TRANSITION_TURN_H_
#define _TRANSITION_TURN_H_

#include "TURN/calPlane.h"

typedef enum TurnDirection_
{
    UNKOWN_TURN = 0,
    LEFT_TURN = 1,
    RIGHT_TURN = 2
} TurnDirection;

typedef struct _transTurn {
    GeographicalP center;
    GeographicalP startPoint;
    GeographicalP endPoint;

    /* Direction from center to startPoint, relative to north */
    float startDir_deg;
    float endDir_deg;

    TurnDirection turnDir;
    float radius_nm;
}  TransTurn;

#ifdef  __cplusplus
extern "C" {
#endif
/*
 * This function calculate transition turn from prev leg to next leg. Radius input here is always > 0.
 */
TransTurn createTransTurn(const GeographicalP prevLegStartP, const GeographicalP crossP, const GeographicalP nextLegEndP, const float radius_nm);
#ifdef  __cplusplus
}
#endif
#endif /* _TRANSITION_TURN_H_ */

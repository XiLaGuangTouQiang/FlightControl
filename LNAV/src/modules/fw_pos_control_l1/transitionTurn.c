#include "transitionTurn.h"
#include "TURN/error.h"
#include "TURN/fmsMath.h"

TransTurn createTransTurn(const GeographicalP prevLegStartP, const GeographicalP crossP, const GeographicalP nextLegEndP, const float radius_nm)
{
    TransTurn res = {0};

    //ASSERT_OR_RETURN(radius_nm > 0.0, res);

    /* Init calculation plane with origin (legs crossing) */
    CalPlane calPlane = {0};
    geog_CalPlane_init(&calPlane, crossP);

    /* Convert geographical points to points on plane */
    const CalPlaneP prevLegStartOnP = geog_geogPToPlaneP(&calPlane, prevLegStartP);
    const CalPlaneP crossPOnP = geog_geogPToPlaneP(&calPlane, crossP);
    const CalPlaneP nextLegEndOnP = geog_geogPToPlaneP(&calPlane, nextLegEndP);

    const double prevLegCrsOnP_deg = geog_AToBDirDegByN(prevLegStartOnP, crossPOnP);
    const double nextLegCrsOnP_deg = geog_AToBDirDegByN(crossPOnP, nextLegEndOnP);

    /* Intersection angle between two legs, normalized to [-180, 180] */
    const double intcAngle_deg = normalizeDirDegToRange360((float)(nextLegCrsOnP_deg - (prevLegCrsOnP_deg + 180.0)), -180.0f, 'K');

    /* This method will fail when intersection angle is 0. And no turn when intc angle is -180 or 180. */
    ASSERT_OR_RETURN((!fmsEqual(intcAngle_deg, 0.0f) && !fmsEqual(intcAngle_deg, -180.0f) && !fmsEqual(intcAngle_deg, 180.0f)), res);

    //res.turnDir = (intcAngle_deg > 0.0f) ? LEFT_TURN : RIGHT_TURN;

    const double crossToCenter_deg = normalizeDirDegStd((float)(nextLegCrsOnP_deg - 0.5 * intcAngle_deg));

    const double crossToCenterDist_nm = ((double)radius_nm) / fmsSin(fmsFAbs(degreeToRad(0.5 * intcAngle_deg)));

    const CalPlaneP centerOnP = geog_pointDirDistOnPlane(crossPOnP, geog_CalPlaneDir_createByN(crossToCenter_deg),
                                                         crossToCenterDist_nm);

    /* Reserved dist is the dist between legs crossing and entering arc point */
    const double reservedDist_nm = fmsSqrt(crossToCenterDist_nm * crossToCenterDist_nm - (double)(radius_nm * radius_nm));
    const CalPlaneP enterArcPOnP = geog_pointDirDistOnPlane(crossPOnP, geog_CalPlaneDir_createByN(prevLegCrsOnP_deg),
                                                            -reservedDist_nm);
    const double startDegOnP = geog_AToBDirDegByN(centerOnP, enterArcPOnP);

    const CalPlaneP leaveArcPOnP = geog_pointDirDistOnPlane(crossPOnP, geog_CalPlaneDir_createByN(nextLegCrsOnP_deg),
                                                            reservedDist_nm);
    const double endDegOnP = geog_AToBDirDegByN(centerOnP, leaveArcPOnP);

    /* Convert back to geographical points */
    res.center = geog_planePToGeogP(&calPlane, centerOnP);
    res.startPoint = geog_planePToGeogP(&calPlane, enterArcPOnP);
    res.endPoint = geog_planePToGeogP(&calPlane, leaveArcPOnP);

    res.startDir_deg = geog_planeDirToGeogDirAt(&calPlane, enterArcPOnP, startDegOnP);
    res.endDir_deg = geog_planeDirToGeogDirAt(&calPlane, leaveArcPOnP, endDegOnP);

    res.radius_nm = radius_nm;

    return res;
}

#include "MODEL/LNAV.h"
//#include "error.h"
#include "lNavMain.h"
#include <stdio.h>

// static inputDataStructure_A653_IN_NAV_STATE InputNAVState;
// static inputDataStructure_A653_IN_LATERAL_COMMANDS InputPerf;
// static inputDataStructure_A653_IN_ADS_AF20_1 InputAF20;

// static A653_OUT_GUIDANCE_OUTPUT_FCS_type OutputGuidanceFCS;
// static A653_OUT_GUIDANCE_OUTPUT_type OutputGuidance;
// static A653_OUT_GUIDANCE_STATE_type outputGuidanceState;

// static LnavMainInput_t LnavMainInput;
// static PerfLegs legs;
static LNavStatus lNavStatus;

void lnav_init(void)
{
	memset(&LNAV_U, 0, sizeof(LNAV_U));
    memset(&LNAV_Y, 0, sizeof(LNAV_Y));
    memset(&LNAV_DW, 0, sizeof(LNAV_DW));

    LNAV_initialize();
}


static bool receiveNavState(LnavMainInput_t LnavMainInput)
{
    LNAV_U.latitude_deg = LnavMainInput.Position.Latitude;
    LNAV_U.longitude_deg = LnavMainInput.Position.Longitude;
    LNAV_U.MagnTrack_deg = LnavMainInput.TrackAngle.Magnetic;
    LNAV_U.CAS_kts = LnavMainInput.TrueAirSpeed;
    return true;
}

static bool receivePerf(LnavMainInput_t LnavMainInput)
{
    /*
    printf("PreLeg LatiMin %f \n", legs.preLegFix.p.latiMin);
    printf("PreLeg LongiMin %f \n", legs.preLegFix.p.longiMin);
    printf("CurLeg type %c \n", legs.curLegType);
    printf("CurLeg Radium %f \n", legs.curLegRadium);
    printf("CurLeg Center LatiMin %f \n", legs.curLegCenter.latiMin);
    printf("CurLeg Center LongiMin %f \n", legs.curLegCenter.longiMin);
    printf("CurLeg Fix LatiMin %f \n", legs.curLegFix.p.latiMin);
    printf("CurLeg Fix LongiMin %f \n \n", legs.curLegFix.p.longiMin);
    */

    // if (myIsnan(legs.preLegFix.p.latiMin) || myIsnan(legs.preLegFix.p.longiMin) || myIsnan(legs.curLegType)
    //     || myIsnan(legs.curLegRadium) || myIsnan(legs.curLegCenter.latiMin) || myIsnan(legs.curLegCenter.longiMin)
    //     || myIsnan(legs.curLegFix.p.latiMin) || myIsnan(legs.curLegFix.p.longiMin))
    // {
    //     return false;
    // }

    // if ((legs.curLegType != 'L') && (legs.curLegType != 'A'))
    // {
    //     return false;
    // }

    // LNAV_U.perfLegs[0] = legs.preLegFix.p.latiMin;
    // LNAV_U.perfLegs[1] = legs.preLegFix.p.longiMin;
    // LNAV_U.perfLegs[2] = (legs.curLegType == 'L')? 1.0 : 0.0;

    // /* Arc radium in model is always > 0.
    //  * Model implicitly assumes arc turnDeg < 180 deg.
    //  * It determines arc dir according to relative position of arc enter, starting point and end point.
    //  * This method will fail when turnDeg >= 180 deg, like PI. TODO
    //  */
    // LNAV_U.perfLegs[3] = fmsFAbs(legs.curLegRadium);
    // LNAV_U.perfLegs[4] = legs.curLegCenter.latiMin;
    // LNAV_U.perfLegs[5] = legs.curLegCenter.longiMin;
    // LNAV_U.perfLegs[6] = legs.curLegFix.p.latiMin;
    // LNAV_U.perfLegs[7] = legs.curLegFix.p.longiMin;

    LNAV_U.perfLegs[0] = LnavMainInput.position_setpoint[0];
	LNAV_U.perfLegs[1] = LnavMainInput.position_setpoint[1];
	LNAV_U.perfLegs[2] = LnavMainInput.position_setpoint[2];
	LNAV_U.perfLegs[3] = LnavMainInput.position_setpoint[3];
	LNAV_U.perfLegs[4] = LnavMainInput.position_setpoint[4];
	LNAV_U.perfLegs[5] = LnavMainInput.position_setpoint[5];
	LNAV_U.perfLegs[6] = LnavMainInput.position_setpoint[6];
	LNAV_U.perfLegs[7] = LnavMainInput.position_setpoint[7];
    return true;
}

/* BP6.2.1.3 ATA 27 FCS: HF_FLIGHT_CONTROL_MODULE_CLASS.xml */
//static const uint8_t LNAV_Arm = 102U;
//static const uint8_t LNAV_Active = 103U;

static bool receiveLNavStatus(LnavMainInput_t LnavMainInput)
{
    // ASSERT_OR_RETURN_FALSE(IO_receive_A653_IN_ADS_AF20_1(&InputAF20) != -1);

    // if (InputAF20.Status_of_Flight_Director_Armed_Lateral_Mode_in_DS15 == IO_RECEIVE_STATUS_VALID)
    // {
    //     lNavStatus.lNavArmed = (InputAF20.Flight_Director_Armed_Lateral_Mode_in_DS15 == LNAV_Arm);
    // }

    // if (InputAF20.Status_of_Flight_Director_Active_Lateral_Mode_in_DS15 == IO_RECEIVE_STATUS_VALID)
    // {
    //     lNavStatus.lNavActive = (InputAF20.Flight_Director_Active_Lateral_Mode_in_DS15 == LNAV_Active);
    // }
    lNavStatus.lNavArmed = LnavMainInput.lnav_status.lNavArmed;
    lNavStatus.lNavActive = LnavMainInput.lnav_status.lNavActive;
    return true;
}

static float calculateRollAngle(void)
{
    LNAV_step();
    float desirableRoll = (float)LNAV_Y.fms_hor_cmd_signal_sel;
    /* printf("desirableRoll %f \n", desirableRoll); */
    return desirableRoll;
}

// static void sendLNAVOutput(float rollAngle)
// {
//     /* Haven't dealt with return code now. TODO */
//     RETURN_CODE_TYPE retCode = NO_ERROR;
//     OutputGuidanceFCS.Status_Of_DS1_GUIDANCE = A664_DS_NORMAL_OPERATION;
//     if (myIsnan(rollAngle))
//     {
//         OutputGuidanceFCS.DS1_GUIDANCE.SSM_in_L121_Horizontal_Command_Signal = SSM_BNR_NO_COMPUTED_DATA;
//         OutputGuidanceFCS.DS1_GUIDANCE.Horizontal_Command_Signal_in_L121_Horizontal_Command_Signal = 0.0f;
//     }
//     else
//     {
//         OutputGuidanceFCS.DS1_GUIDANCE.SSM_in_L121_Horizontal_Command_Signal = SSM_BNR_NORMAL_OPERATION;
//         OutputGuidanceFCS.DS1_GUIDANCE.Horizontal_Command_Signal_in_L121_Horizontal_Command_Signal = rollAngle;
//     }
//     IO_send_A653_OUT_GUIDANCE_OUTPUT_FCS(&OutputGuidanceFCS, &retCode);

//     OutputGuidance.Status_Of_DS1_GUIDANCE = A664_DS_NORMAL_OPERATION;
//     if (myIsnan(rollAngle))
//     {
//         OutputGuidance.DS1_GUIDANCE.SSM_in_L121_Horizontal_Command_Signal = SSM_BNR_NO_COMPUTED_DATA;
//         OutputGuidance.DS1_GUIDANCE.Horizontal_Command_Signal_in_L121_Horizontal_Command_Signal = 0.0f;
//     }
//     else
//     {
//         OutputGuidance.DS1_GUIDANCE.SSM_in_L121_Horizontal_Command_Signal = SSM_BNR_NORMAL_OPERATION;
//         OutputGuidance.DS1_GUIDANCE.Horizontal_Command_Signal_in_L121_Horizontal_Command_Signal = rollAngle;
//     }
//     IO_send_A653_OUT_GUIDANCE_OUTPUT(&OutputGuidance, &retCode);

//     /* printf("Guidance send rollAngle %f \n\n", rollAngle); */
// }

// static void sendLNavStatusToCore(void)
// {
//     RETURN_CODE_TYPE retCode = NO_ERROR;
//     memcpy(outputGuidanceState.DS_Guidance_State.Guidance_State, &lNavStatus, sizeof(LNavStatus));
//     IO_send_A653_OUT_GUIDANCE_STATE(&outputGuidanceState, &retCode);
// }

void lnavMain(LnavMainInput_t LnavMainInput,float* roll_body)
{
    bool navStateValid = receiveNavState(LnavMainInput);
    bool perfPlanValid = receivePerf(LnavMainInput);
    receiveLNavStatus(LnavMainInput);
    //float desirableRoll;
    if (navStateValid && perfPlanValid)
    {
        *roll_body = calculateRollAngle();
    }
    else
    {
        *roll_body = 0.0f;
    }
}


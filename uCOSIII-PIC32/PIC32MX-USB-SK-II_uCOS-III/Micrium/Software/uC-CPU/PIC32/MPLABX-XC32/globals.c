#include "globals.h"

static  CPU_INT16U LeftEncoder_State, RightEncoder_State;
static  CPU_INT16U LeftEncoder_Ticks = 0, RightEncoder_Ticks = 0;
static  CPU_INT16U LeftWheelPercent = 0, RightWheelPercent = 0;
static  CPU_INT16U IdealLeftWheelSpeed = 4, IdealRightWheelSpeed = 4; //inches per second

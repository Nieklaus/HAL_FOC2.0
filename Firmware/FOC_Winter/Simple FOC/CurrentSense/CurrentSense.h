#ifndef __CurrentSense_H__
#define __CurrentSense_H__

#include "main.h"
#include "arm_math.h"
#include "stdio.h"
//#include "FOC_Define.h"
#include "InlineCurrentSense.h"

DQCurrent_s getFOCCurrents(float angle_el);

#endif


#ifndef SIM_HOST_MAIN_H
#define SIM_HOST_MAIN_H

#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

#define __disable_irq() ((void)0)
#define __enable_irq()  ((void)0)

uint32_t HAL_GetTick(void);

#endif

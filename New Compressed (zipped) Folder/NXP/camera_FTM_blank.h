#ifndef CAMERA_FTM_H
#define CAMERA_FTM_H

#include <stdint.h>

int maincam(void);
void init_FTM2(void);
void init_GPIO(void);
void init_PIT(void);
void init_ADC0(void);
void FTM2_IRQHandler(void);
void PIT1_IRQHandler(void);
void ADC0_IRQHandler(void);

#endif
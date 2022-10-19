#include "main.h"

//------------------------------------------------------------------------------
#define LED_PORT1 GPIOC
#define LED_BIT2 GPIO_PIN_2
#define Led1On() HAL_GPIO_WritePin(LED_PORT1, LED_BIT1, GPIO_PIN_SET);
#define Led1Off() HAL_GPIO_WritePin(LED_PORT1, LED_BIT1, GPIO_PIN_RESET);


#define LED_PORT2 GPIOB
#define LED_BIT2 GPIO_PIN_1
#define Led2On() HAL_GPIO_WritePin(LED_PORT2, LED_BIT2, GPIO_PIN_SET);
#define Led2Off() HAL_GPIO_WritePin(LED_PORT2, LED_BIT2, GPIO_PIN_RESET);
//------------------------------------------------------------------------------



//-----------------------------------------------------------------------------
#define _USE_W5300_OPTIMIZE				1

#define W5300_BANK_ADDR                 ((uint32_t)0x64000000)
#define _W5300_DATA(p)                  (*(volatile unsigned short*) (W5300_BANK_ADDR + (p<<1)))


#define RESET_W5300_GPIO_Port			GPIOG
#define RESET_W5300_Pin					GPIO_PIN_11
//-----------------------------------------------------------------------------

                                        
                                        

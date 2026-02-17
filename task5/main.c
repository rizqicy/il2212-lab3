#include <stdio.h>
#include "bsp.h"

/*************************************************************/

/**
 * @brief Main function.
 * 
 * @return int 
 */
int main()
{
    BSP_Init();             /* Initialize all components on the lab-kit. */
    
    while (true) { 
        BSP_ToggleLED(LED_GREEN);
        sleep_ms(1000); 
    }
}
/*-----------------------------------------------------------*/

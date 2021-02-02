

#include <stdio.h>

#include "pico/stdlib.h"
#include "qwiic_ccs811.h"


CCS811 mySensor;

int main(){

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

     // setup stdio. Output device is controlled in CMakeLists.txt file
    stdio_init_all();

    printf("Starting CCS811 Basic Example\n\n");

    if(!mySensor.begin()){
        printf("CCS811 error. Please check wiring. Aborting\n");
        return 0;
    }

    while(1){
        gpio_put(LED_PIN, 1);
        sleep_ms(250);

        //Check to see if data is ready with .dataAvailable()
        if(mySensor.dataAvailable()){
            //If so, have the sensor read and calculate the results.
            //Get them later
            mySensor.readAlgorithmResults();

            printf("CO2[%d] tVOC[%d]\n", mySensor.getCO2(), mySensor.getTVOC() );

        }
        gpio_put(LED_PIN, 0);
        sleep_ms(2000);
  }

}

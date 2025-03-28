#include "daisy_seed.h"
#include "include/max98389.h"

// Use the daisy namespace to prevent having to type
// daisy:: before all libdaisy functions
using namespace daisy;
// Declare a DaisySeed object called hardware
DaisySeed hardware;

max98389 amp;

int main(void)
{
    // Declare a variable to store the state we want to set for the LED.
    bool led_state;
    led_state = true;

    // Configure and Initialize the Daisy Seed
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hardware.Configure();
    hardware.Init();
    hardware.StartLog(false);
    System::Delay(500);
    // setup the configuration
    
    if (!amp.init())
    {
        hardware.PrintLine("ERROR: Failed to initialize MAX98389.\n");
        return -1;
    }

    // Loop forever
    for(;;)
    {
        // Set the onboard LED
        hardware.SetLed(led_state);

        // Toggle the LED state for the next time around.
        led_state = !led_state;
        // Wait 500ms
        System::Delay(1500);
    }
}

#include "main.h"
#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "CAN_processing.h"
#include "TestList.h"

// Hi, this is going to be the test procedure after assembling new steering boards.
// In Hope (found in Drugs), there will be the same file structure as here.
// So when testing the boards, just update the defines for TESTX (1-4), and
// you're golden.
// This will make testing easier and more repeatable.

void AssortedTests() {

#ifdef TEST1 // The first test blinks the LED.
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(1000);
#endif

#ifdef TEST2 // The second test spins the motor arbitrarily.
    setPIDGoalA(30);
     HAL_Delay(1000);
    setPIDGoalA(60);
    HAL_Delay(1000);
    // June 30 2025 - something is weird with the code
    // but the motor doesn't arrive at goalangle... idk.
#endif

#ifdef TEST3 // The third test blinks the onboard LED after receipt of a message.
    // Additionally, it moves the motor to PI/4 and PI/2.
    // And messages are sent in the first place with the button on the nucleo.
    // Nothing changes with this custom board code. Only Hope changes.
    // Namely, the ID is changed to 0x1B4.
#endif

#ifdef TEST4
    // The fourth test sets the motor to PI/4, and then the nucleo polls the
    // board for the motor location.
    setPIDGoalA(45);
#endif

#ifdef TEST5
	// The fifth test has the motor stop.
	// The ID changes to 0x184 on the nucleo
    setPIDGoalAngleA(45);
    HAL_Delay(1000);
    setPIDGoalAngleA(90);
    HAL_Delay(1000);
    // same looping as in the second test.
#endif
}



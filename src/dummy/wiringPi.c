#include <wiringPi.h>
#include <stdio.h>

int wiringPiSetup() 
{
    printf("Setup dummy wiringPi\n");
    return 0;
}

int wiringPiISR(int pin, int mode, void (*function)(void)) 
{
    printf("Set interrupt on pin %d, mode %d\n", pin, mode);
    return 0;
}

void digitalWrite(int pin, int value) 
{
    printf("Writing pin %d to %d\n", pin, value);
}

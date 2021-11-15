#include "mbed.h"
#include "vesc_can.h"

CAN can1(PA_11, PA_12, 1000000);
vesc_can _vesc;
int main()
{
    _vesc.vesc_can_init(&can1);
    while (true) {

    }
}


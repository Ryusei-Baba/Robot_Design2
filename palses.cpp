#include "mbed.h"
#include "QEI.h"

BusIn in (D2, D4, D5, D7);
QEI qei1(D2, D4, NC, 48, QEI::X4_ENCODING);
QEI qei2(D5, D7, NC, 48, QEI::X4_ENCODING);

int main() {
    in.mode(PullUp);
    while(1) {
        printf("%d, %d\r\n", qei1.getPulses(), qei2.getPulses());
        wait(0.5);
    }
}

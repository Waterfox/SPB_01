#ifndef endstops_H
#define endstops_H
 
#include <Arduino.h>
/* endstops class:
 *  manages endstop interrupts and available motion states
 *  the upper endstop is "max"
 *  the lower endstop is "min"
 */
class endstops {
public:
        endstops();
        ~endstops();
        void endstops_init();
        void check_endstops();
        bool enUp;
        bool enDown;
};
 
#endif



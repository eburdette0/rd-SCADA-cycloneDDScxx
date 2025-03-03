#ifndef MACRO_DEFINITIONS_H
#define MACRO_DEFINITIONS_H

//split this off to avoid circular dependencies in queues (physicalCalc)

#define INPUT_FREQ 1000 //hz

#define INPUT_CHANNELS 8
#define OUTPUT_CHANNELS 8
#define RELAY_CHANNELS 12 //5 working pairs, last bit is for OK signal
#define CALC_CHANNELS 16
#define PID_CHANNELS 16
#define DUMMY_CHANNELMIN 64

#define DISPLAY_CHANNELS 24

#define DACbitsFR 16
#define DACHRange 10.0
#define SOARatio 243 // summing op amp ratio in custom MOOG valve input combiner
#define DOUBLEDChan0 0 // mooghi, mooglo is mooghi+1. If not needed, set to large number


#endif //MACRO_DEFINITIONS_H
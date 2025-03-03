#ifndef DEFINITIONS_H
#define DEFINITIONS_H


#include "systemDifferences.h" //has realdata and UEI/labjack control defines

#include "PID/PID_V1.h" //for PID structures
#include "physicalCalculations/physicalCalculations_V1.h"


#include <array>
#include <tuple>
#include <cstdint>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/policies.hpp>

#include <memory>

#include "macro_definitions.h"
// #define INPUT_FREQ 1000 //hz

// #define INPUT_CHANNELS 8
// #define OUTPUT_CHANNELS 8
// #define RELAY_CHANNELS 12 //5 working pairs, last bit is for OK signal
// #define CALC_CHANNELS 16
// #define PID_CHANNELS 16
// #define DUMMY_CHANNELMIN 64

// #define DISPLAY_CHANNELS 16

// #define DACbitsFR 16
// #define DACHRange 10.0
// #define SOARatio 243 // summing op amp ratio in custom MOOG valve input combiner
// #define DOUBLEDChan0 0 // mooghi, mooglo is mooghi+1. If not needed, set to large number


struct StringWrapper {
        std::shared_ptr<std::string> str;

        StringWrapper() = default;
        StringWrapper(const std::string& s) : str(std::make_shared<std::string>(s)) {}
        StringWrapper(std::string&& s) : str(std::make_shared<std::string>(std::move(s))) {}

        // Trivial destructor and assignment operator
        ~StringWrapper() = default;
        StringWrapper& operator=(const StringWrapper&) = default;
        StringWrapper& operator=(StringWrapper&&) = default;
};

namespace dataStructures1
{
        typedef std::tuple<uint64_t,
                std::array<double,INPUT_CHANNELS>,
                std::array<double,OUTPUT_CHANNELS>,
                std::array<bool, RELAY_CHANNELS>, 
                std::array<double,INPUT_CHANNELS+CALC_CHANNELS>,
                physCalc::calculatorParams> calcDataTuple;

        typedef std::tuple<uint64_t,
                std::array<double,INPUT_CHANNELS>,
                std::array<double,OUTPUT_CHANNELS>,
                std::array<bool, RELAY_CHANNELS>, 
                std::array<double,INPUT_CHANNELS+CALC_CHANNELS>> dataTuple;
                
        typedef std::tuple<std::array<PID, PID_CHANNELS>,
                    std::array<uint32_t, PID_CHANNELS>, 
                    std::array<uint32_t, PID_CHANNELS>, 
                    std::array<double, PID_CHANNELS>> pidCoeffTuple;


        // typedef std::tuple<std::array<double, PID_CHANNELS>, 
        //         std::array<double, PID_CHANNELS>, 
        //         std::array<double, PID_CHANNELS>, 
        //         std::array<double, PID_CHANNELS>> pidIOTuple;

        typedef std::tuple<uint32_t, std::array<double, 10>, uint32_t, uint32_t, double, uint32_t> coeffUpdatetype;
        typedef std::tuple<uint32_t, double, double, uint32_t, uint32_t> commandUpdatetype;


        typedef uint64_t logTuple;
        //typedef StringWrapper logTuple;

};

namespace queuedefs{
        namespace blf = boost::lockfree;
        typedef blf::spsc_queue<dataStructures1::calcDataTuple, blf::capacity<1000>> calcDataQueueFastdef;
        typedef blf::spsc_queue<dataStructures1::dataTuple, blf::capacity<1000>> dataQueueFastdef;
        typedef blf::spsc_queue<dataStructures1::dataTuple, blf::capacity<100>> dataQueueSlowdef;
        typedef blf::spsc_queue<dataStructures1::pidCoeffTuple, blf::capacity<100>> pidCoeffQueuedef;
        typedef blf::spsc_queue<dataStructures1::coeffUpdatetype, blf::capacity<100>> pidCoeffUpdateQueuedef;
        typedef blf::spsc_queue<dataStructures1::commandUpdatetype, blf::capacity<100>> pidCommandUpdateQueuedef;
        //typedef blf::spsc_queue<dataStructures1::pidIOTuple, blf::capacity<100>> pidIOQueuedef;
};

//crude logging primatives
class logBundle{
        uint64_t errNo;
        uint64_t val;
};

enum logMessages{
        unknown,
        mainStarted,
        mainStopped,
        hardwareStarted,
        hardwareStopped,
        networkStarted,
        networkStopped,
        displayStarted,
        displayStopped
};


#endif //DEFINITIONS_H
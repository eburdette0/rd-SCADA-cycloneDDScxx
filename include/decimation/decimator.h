#ifndef DECIMATOR_H
#define DECIMATOR_H

#include "definitions.h"
#include "FIR/coeffHeaders/10xCoeffs.h"
#include "FIR/FIR_ds.h"

#include <atomic>
#include <thread>
#include <iostream>
#include <functional>

#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/queue.hpp>

namespace blf = boost::lockfree;

class decimator {
public:
    decimator(
            std::array<double, INPUT_CHANNELS> scaling,
            std::array<double, INPUT_CHANNELS> offsets,
            queuedefs::calcDataQueueFastdef& queue1000,
            queuedefs::dataQueueSlowdef& queue100,
            queuedefs::dataQueueSlowdef& queue10,
            queuedefs::dataQueueSlowdef& queue1);

    void start();
    void stop();

private:

    void monitor();
    void consume(dataStructures1::calcDataTuple value);
    std::array<double, INPUT_CHANNELS> scaling;
    std::array<double, INPUT_CHANNELS> offsets;

    queuedefs::calcDataQueueFastdef& q1000;
    queuedefs::dataQueueSlowdef& q100;
    queuedefs::dataQueueSlowdef& q10;
    queuedefs::dataQueueSlowdef& q1;
    std::atomic<bool> running;
    std::thread monitorThread;


    double taps[coeffs::Order1]{};
    FIR_circ Dec100[INPUT_CHANNELS];
    FIR_circ Dec10[INPUT_CHANNELS];
    FIR_circ Dec1[INPUT_CHANNELS];

    uint32_t it_dec1000 = 0;
    uint32_t it_dec100 = 0;
    uint32_t it_dec10 = 0;

    double aiDec100[INPUT_CHANNELS]{};
    double aiDec10[INPUT_CHANNELS]{};
    double aiDec1[INPUT_CHANNELS]{};
    double intermed1000[INPUT_CHANNELS][10]{}; //factor of 10 decimation
    double intermed100[INPUT_CHANNELS][10]{};
    double intermed10[INPUT_CHANNELS][10]{};


    double scaledVals1000[INPUT_CHANNELS+CALC_CHANNELS]{};
    double scaledVals100[INPUT_CHANNELS+CALC_CHANNELS]{};
    double scaledVals10[INPUT_CHANNELS+CALC_CHANNELS]{};
    double scaledVals1[INPUT_CHANNELS+CALC_CHANNELS]{};

    dataStructures1::dataTuple OutTuple100;
    dataStructures1::dataTuple OutTuple10;
    dataStructures1::dataTuple OutTuple1;

    std::array<double, INPUT_CHANNELS> aiEmit1000{}; std::array<double, INPUT_CHANNELS+CALC_CHANNELS> calcEmit1000{};
    std::array<double, INPUT_CHANNELS> aiEmit100{}; std::array<double, INPUT_CHANNELS+CALC_CHANNELS> calcEmit100{};
    std::array<double, INPUT_CHANNELS> aiEmit10{}; std::array<double, INPUT_CHANNELS+CALC_CHANNELS> calcEmit10{};
    std::array<double, INPUT_CHANNELS> aiEmit1{}; std::array<double, INPUT_CHANNELS+CALC_CHANNELS> calcEmit1{};

};

#endif
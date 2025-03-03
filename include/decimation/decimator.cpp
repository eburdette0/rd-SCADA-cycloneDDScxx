#include "decimator.h"

#include "definitions.h"
#include "FIR/coeffHeaders/10xCoeffs.h"
#include "FIR/FIR_ds.h"
#include "physicalCalculations/physicalCalculations_V1.h"

#include <atomic>
#include <thread>
#include <iostream>

#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/queue.hpp>


namespace blf = boost::lockfree;


decimator::decimator(
    std::array<double, INPUT_CHANNELS> Ext_scaling,
    std::array<double, INPUT_CHANNELS> Ext_offsets,
    queuedefs::calcDataQueueFastdef& queue1000,
    queuedefs::dataQueueSlowdef& queue100,
    queuedefs::dataQueueSlowdef& queue10,
    queuedefs::dataQueueSlowdef& queue1)
    : q1000(queue1000),
    q100(queue100),
    q10(queue10),
    q1(queue1),
    running(false),
    scaling(Ext_scaling),
    offsets(Ext_offsets)
{
    double taps[coeffs::Order1]{};

}

//fill the FIR filters and start the queue monitor
void decimator::start() {
    running = true;
    //FIR startup
    scaleTaps(taps, coeffs::CoeffData1, coeffs::Order1, 16); //data from integer coefficient header
    for (int i=0; i<INPUT_CHANNELS; i++){
        Dec100[i].fillTaps(taps, coeffs::Order1);
        Dec10[i].fillTaps(taps, coeffs::Order1);
        Dec1[i].fillTaps(taps, coeffs::Order1);
    }

    //start monitoring the 1000Hz queue
    monitorThread = std::thread(&decimator::monitor, this);
}

void decimator::stop() {
    running = false;
    if (monitorThread.joinable()) {
        monitorThread.join();
    }
}

//monitor the 1000Hz queue and decimate when data is available
void decimator::monitor() {
    while (running) {
        if(!q1000.consume_one([this](dataStructures1::calcDataTuple d){this->consume(d);})){
            //handle error
        };
        
        if (q1000.read_available()==0){;
            std::this_thread::sleep_for(std::chrono::microseconds((1000000/INPUT_FREQ)*1/5));
        }
        

    }
}

//process elements and send them out on 3 decimated queues
void decimator::consume(dataStructures1::calcDataTuple value) {
    
    //decimation
    for (int i = 0; i < INPUT_CHANNELS; i++) {

        intermed1000[i][it_dec1000] = std::get<1>(value)[i];//fill 10x decimation array
        if (it_dec1000 + 1 == 10){
            Dec100[i].filter(intermed1000[i], &aiDec100[i], 10);
            intermed100[i][it_dec100] = aiDec100[i]; //fill 10x decimation array with filtered data
        }
        if (it_dec100 + 1 == 10){
            Dec10[i].filter(intermed100[i], &aiDec10[i], 10);
            intermed10[i][it_dec10] = aiDec10[i];
        }
        if (it_dec10 + 1 == 10){
            //Dec1[i].filter(intermed10[i], &aiDec1[i], 10);
            //intermed1[i][it_dec1] = aiDec1[i];
        }
    }

    //sucessive decimated iterators
    it_dec1000 = (it_dec1000 +  1) %  10;
    it_dec100 = (it_dec1000 ==  0) ? (it_dec100 +  1) %  10 : it_dec100;
    it_dec10 = (it_dec100 ==  0) ? (it_dec10 +  1) %  10 : it_dec10;


    //scale and transfer data
    if (it_dec1000 == 0){
        physCalc::aiToCalc(aiDec100, scaling.data(), offsets.data(), scaledVals100,  std::get<5>(value));
        physCalc::copyScaledAi(aiDec100, aiEmit100.data());
        physCalc::copyFullCalcs(scaledVals100, calcEmit100.data());
        q100.push(std::make_tuple(std::get<0>(value), aiEmit100, std::get<2>(value), std::get<3>(value), calcEmit100));
    }

    if ((it_dec1000 + it_dec100) == 0){
        physCalc::aiToCalc(aiDec10, scaling.data(), offsets.data(), scaledVals10,  std::get<5>(value));
        physCalc::copyScaledAi(aiDec10, aiEmit10.data());
        physCalc::copyFullCalcs(scaledVals10, calcEmit10.data());
        q10.push(std::make_tuple(std::get<0>(value), aiEmit10, std::get<2>(value), std::get<3>(value), calcEmit10));
    }

    if ((it_dec1000 + it_dec100 + it_dec10) == 0){
        // calcAndScaling(aiDec1, scaling.data(), offsets.data(), scaledVals1, aiEmit1.data(), calcEmit1.data(), std::get<5>(value));
        q1.push(std::make_tuple(std::get<0>(value), aiEmit10, std::get<2>(value), std::get<3>(value), calcEmit10));
    }

}
#ifndef GETDATA_H
#define GETDATA_H

#include "definitions.h" //has channel count definitions


#ifdef REALDATA
    #ifdef HARDWARE_UEI
        #include "PDNA.h"
        #include "UeiPacUtils/UeiPacUtils.h"
    #endif
    #ifdef HARDWARE_LABJACK
        #include "LabJackUD.h"
    #endif
#else
#include "rigSim/rigSim.h"
#endif

//#include "rapidjson/document.h" //for parsing calibration values

#include "PID/PID_V1.h"
#include "FIR/coeffHeaders/10xCoeffs.h"
#include "FIR/FIR_ds.h"
#include "physicalCalculations/physicalCalculations_V1.h"

#include <chrono>
#include <thread>
#include <filesystem>
#include <cstdint>

#include <boost/signals2.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/policies.hpp>

namespace blf = boost::lockfree;



void calcAndScaling(double*, double*, double*, double*, double*, double*, physCalc::calculatorParams);

class GetData {
public:
    explicit GetData(std::string filename,
    queuedefs::calcDataQueueFastdef& dataQueue,
    queuedefs::pidCoeffQueuedef& pidCoeffQueue,
    queuedefs::pidCoeffUpdateQueuedef& pidCoeffUpdateQueue,
    queuedefs::pidCommandUpdateQueuedef& pidCommandUpdateQueue);
    
    ~GetData();

//signals
    
    boost::signals2::signal<void()> update_data; //now thread safe
    boost::signals2::signal<void()> finished;


//slots
    void start();
    void stop_data();
    void quit();

    void setpointUpdate(std::tuple<uint32_t, double>); //uint32_t ch, double SP);
    void manualOPUpdate(std::tuple<uint32_t, double>); //uint32_t ch, double mOP);
    void autoCtrlUpdate(std::tuple<uint32_t, uint32_t>); //uint32_t ch, uint32_t autoCtrl);
    void lockUpdate(std::tuple<uint32_t, uint32_t>); //uint32_t ch, uint32_t locked);
    void controlUpdate(dataStructures1::commandUpdatetype);
    
    void inputMapUpdate(uint32_t, uint32_t);
    void outputMapUpdate(uint32_t, uint32_t);
    void voltsConversionUpdate(uint32_t, double);
    void coeffUpdate(dataStructures1::coeffUpdatetype);

    void doNothing();
    void timedUpdate();

    bool stop_running = false;

    std::thread timer_thread;
    bool timer_fire = false;
    uint64_t timer_loops = 0;

    std::array<double, INPUT_CHANNELS> aiScaling{};
    std::array<double, INPUT_CHANNELS> aiOffsets{};    

private:

    double ai_cal_scaling[INPUT_CHANNELS] {};
    double ai_cal_offsets[INPUT_CHANNELS] {};

    dataStructures1::calcDataTuple dataOutStage {};
    dataStructures1::pidCoeffTuple pidCoeffOut {};

    queuedefs::calcDataQueueFastdef& dq;
    queuedefs::pidCoeffQueuedef& pCq;
    queuedefs::pidCoeffUpdateQueuedef& coeq;
    queuedefs::pidCommandUpdateQueuedef& comq;

    uint64_t it_count = 0;
    int ret = 0;
    uint32_t m_count = 0;



    const float frequency = INPUT_FREQ;        //Hz
    const double timestep = 1.0/frequency; // seconds



    #ifdef REALDATA
    PDNA_PARAMS aiparams = { 0, INPUT_CHANNELS, {0,1,2,3,4,5,6,7}, frequency};
    PDNA_PARAMS aoparams = { 1, OUTPUT_CHANNELS, {0,1,2,3,4,5,6,7}, frequency};
    PDNA_PARAMS roparams = { 2, 1, {0}, frequency};
    DQSETCLK clkSet;
    int hd0 = 0;
    uint32 errchan = 0;
    uint32 newdata = 0;
    uint32 pgadata[16] {};
    std::array<uint32, DQ_AI217_CHAN> data1{};
    std::array<double, DQ_AI217_CHAN> fdata{};
    std::array<double, OUTPUT_CHANNELS> outData{};
    std::array<bool, RELAY_CHANNELS> relayData{};

    #else
    Machine Rig1;

    std::array<uint32_t, INPUT_CHANNELS> data1{};
    std::array<double, INPUT_CHANNELS> fdata{};
    std::array<double, OUTPUT_CHANNELS> outData{};
    std::array<bool, RELAY_CHANNELS> relayData{};

    #endif

    std::array<double, CALC_CHANNELS> calcData{};
    physCalc::calculatorParams calcParams;

    //static const int pidChannels = 16;


    std::array<double, PID_CHANNELS> PIDSetpoint{}; // initializes to all zeros
    std::array<double, PID_CHANNELS> PIDInput{};
    std::array<double, PID_CHANNELS> PIDOutput{};
    std::array<double, PID_CHANNELS> PIDManualOP{};

    PID PIDch[PID_CHANNELS] {};
    PIDcoeff PIDcoeffIn[PID_CHANNELS] {};
    std::array<uint32_t, PID_CHANNELS> inputMap {};
    std::array<uint32_t, PID_CHANNELS> outputMap {};
    std::array<double, PID_CHANNELS> voltsConversion {};
    uint32_t LoadSafe = 0; //for external PID control

    uint32_t it_dec1000 = 0;
    uint32_t it_dec100 = 0;
    uint32_t it_dec10 = 0;
    
    double scaledVals1000[INPUT_CHANNELS+CALC_CHANNELS] {};

    std::array<double, INPUT_CHANNELS> aiEmit1000{}; std::array<double, INPUT_CHANNELS+CALC_CHANNELS> calcEmit1000{};

    double data_array[INPUT_CHANNELS] {};


};

#endif // GETDATA_H
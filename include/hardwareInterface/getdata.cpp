#include "getdata.h"

#include <cstdint>
#include <stdio.h>
#include <signal.h>
#include <sched.h>
//#include <netinet/in.h>
#include <unistd.h>
#include <sys/time.h>
#include <chrono>
#include <cmath>

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


#include "PID/PID_V1.h"
#include "FIR/coeffHeaders/10xCoeffs.h"
#include "FIR/FIR_ds.h"
#include "physicalCalculations/physicalCalculations_V1.h"


#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/policies.hpp>
#include <boost/signals2.hpp>

#include "rapidjson/document.h" //for parsing calibration values
#include <filesystem>
#include <fstream>
#include <string>


#include <iostream>

using namespace std;
namespace blf = boost::lockfree;


//params defined in header

struct timeval tv1, tv2;
double duration = 0.0;

// Add specified amount of ns to timespec (UEI)
static inline void timespec_add_ns(struct timespec *a, unsigned int ns)
{
#define NSECS_PER_SEC 1000000000L
    ns += a->tv_nsec;
    while (ns >= NSECS_PER_SEC)
    {
        ns -= NSECS_PER_SEC;
        a->tv_sec++;
    }
    a->tv_nsec = ns;
}

//works reasonably well on windows
static void sleep_for(uint64_t interval)
{
    static constexpr std::chrono::duration<double> MinSleepDuration(0);
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration<int64_t, std::nano>(std::chrono::high_resolution_clock::now() - start).count() < interval) {
        std::this_thread::sleep_for(MinSleepDuration);
    }
}

void timerFunction(boost::signals2::signal<void()> &signal, uint64_t interval, bool &timer_fire,  bool &stop_running, uint64_t &timerloops)
{
    std::cout << "Starting Timer" << std::endl;
    std::cout << "Thread: " << std::this_thread::get_id() << std::endl;

    struct sched_param schedp;
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = 30;
    sched_setscheduler(0, SCHED_FIFO, &schedp);
    struct timeval tv1t, tv2t;

    struct timespec next;
    long long periodns;
    // Use posix timer to time refresh loop at the desired frequency
    periodns = (long long) interval*1000;

    
    uint64_t i=0;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while (true)
    {
        if (i==0){gettimeofday(&tv1, NULL);}
        if (timer_fire){
            signal(); //fires signal
            i+=1;
            timerloops=i;
        }
        
        
        //linux
        timespec_add_ns(&next, periodns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        //std::this_thread::sleep_for(std::chrono::nanoseconds(periodns));
        //sleep_for(periodns);


        if (stop_running){
            signal(); //fire last cycle which cleans up in the classes' main thread (necessary?)
            break;}
    }
    gettimeofday(&tv2, NULL);
    duration = ((tv2.tv_sec-tv1.tv_sec) + (tv2.tv_usec-tv1.tv_usec)/1000000.0);
    printf("\nTimer Executed %" PRIu64 " iterations in %f s (%f updates per sec.)\n", i, duration, i/duration);
}



uint32_t RELAY_Bool_to_uint32(std::array<bool, RELAY_CHANNELS> input){
    uint32_t temp = 0;
    for (int i=0; i<RELAY_CHANNELS; i++){
        temp |= (uint32_t)input[i] << i;
    }
    return temp;
}


void calculate_split_value(double* vIn, double vOutHR, uint32_t bitsFR, uint32_t SOAratio, double* vOutHi, double* vOutLo){
    int32_t intHR=(1<<(bitsFR-1)); // ADC signed integer size
    int32_t vInt=(*vIn/vOutHR)*intHR; // scale to integer size 2^bitsFR
    double _vouthi = vInt/(double)intHR*vOutHR; // hi integer part converted back to double
    double _voutlo = (*vIn -_vouthi) * (double)SOAratio; // low integer part. DAC will automatically bitcrush
    *vOutHi = fmin(fmax(_vouthi,-vOutHR),vOutHR); // clamp to +-voutHR
    *vOutLo = fmin(fmax(_voutlo,-vOutHR),vOutHR);
    //could send values larger than vOutHR but they will be clamped automatically in the DAC
}



GetData::GetData(std::string filename,
    queuedefs::calcDataQueueFastdef& dataQueue,
    queuedefs::pidCoeffQueuedef& pidCoeffQueue,
    queuedefs::pidCoeffUpdateQueuedef& pidCoeffUpdateQueue,
    queuedefs::pidCommandUpdateQueuedef& pidCommandUpdateQueue
        ) : dq(dataQueue), pCq(pidCoeffQueue), coeq(pidCoeffUpdateQueue), comq(pidCommandUpdateQueue)
{
    timer_thread = std::thread(timerFunction, std::ref(update_data), (1000000/INPUT_FREQ), std::ref(timer_fire), std::ref(stop_running),std::ref(timer_loops));
    update_data.connect([this]() { this->timedUpdate(); });
    m_count = 1;
    stop_running = false;


    {//json file scope

    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        // Handle file open error
        printf("RigConfig File Error!!\n");
        assert(ifs.is_open());
    }
    std::string content( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );


    rapidjson::Document document1;
    document1.Parse(content.c_str());
    
    std::cout<< "getData_jsonRead" << std::endl;

    // json document1;
    // document1 = json::parse(content);
    for (int i=0; i<INPUT_CHANNELS; i++){
        std::string chName = std::string("Ch") + (i<10? "0":"") + std::to_string(i);
        assert(document1["Channels"]["phys"]["ai"][chName.c_str()].HasMember("scaling"));
        assert(document1["Channels"]["phys"]["ai"][chName.c_str()].HasMember("offset"));

        ai_cal_scaling[i] = document1["Channels"]["phys"]["ai"][chName.c_str()]["scaling"].GetDouble();
        aiScaling[i] = ai_cal_scaling[i];
        ai_cal_offsets[i] = document1["Channels"]["phys"]["ai"][chName.c_str()]["offset"].GetDouble();
        aiOffsets[i] = ai_cal_offsets[i];
    }
    printf("filled scaling\n");

    for (int i=0; i<PID_CHANNELS; i++){
        std::string chName = std::string("Ch") + (i<10? "0":"") + std::to_string(i);

        PIDcoeffIn[i].kP = document1["Channels"]["soft"]["PID"][chName.c_str()]["kP"].GetDouble();
        PIDcoeffIn[i].kI = document1["Channels"]["soft"]["PID"][chName.c_str()]["kI"].GetDouble();
        PIDcoeffIn[i].kD = document1["Channels"]["soft"]["PID"][chName.c_str()]["kD"].GetDouble();
        PIDcoeffIn[i].bias= document1["Channels"]["soft"]["PID"][chName.c_str()]["bias"].GetDouble();

        PIDcoeffIn[i].outputMax = document1["Channels"]["soft"]["PID"][chName.c_str()]["outputMax"].GetDouble();
        PIDcoeffIn[i].outputMin = document1["Channels"]["soft"]["PID"][chName.c_str()]["outputMin"].GetDouble();
        PIDcoeffIn[i].windupMax = document1["Channels"]["soft"]["PID"][chName.c_str()]["windupMax"].GetDouble();
        PIDcoeffIn[i].windupMin = document1["Channels"]["soft"]["PID"][chName.c_str()]["windupMin"].GetDouble();
        PIDcoeffIn[i].deadbandMax = document1["Channels"]["soft"]["PID"][chName.c_str()]["deadbandMax"].GetDouble();
        PIDcoeffIn[i].deadbandMin = document1["Channels"]["soft"]["PID"][chName.c_str()]["deadbandMin"].GetDouble();

        inputMap[i] = document1["Channels"]["soft"]["PID"][chName.c_str()]["inputMap"].GetUint();
        outputMap[i] = document1["Channels"]["soft"]["PID"][chName.c_str()]["outputMap"].GetUint();
        voltsConversion[i] = document1["Channels"]["soft"]["PID"][chName.c_str()]["voltsConversion"].GetDouble();
    }
    printf("filled PID\n");


    calcParams.lvdt_Ch = document1["MachineCorrections"]["LVDT_Ch"].GetUint();
    calcParams.axialStress_Ch = document1["MachineCorrections"]["axialStress_Ch"].GetUint();
    calcParams.confiningP_Ch = document1["MachineCorrections"]["confiningP_Ch"].GetUint();

    
    calcParams.sealDeadband =  document1["MachineCorrections"]["Deadband"].GetDouble();
    calcParams.sealStiffness = document1["MachineCorrections"]["SealStiffness"].GetDouble();

    calcParams.fractionalStiffness = document1["MachineCorrections"]["DrivingStiffness"].GetDouble();
    calcParams.extPistonRadius = document1["MachineCorrections"]["ExtPistonRad"].GetDouble();

    calcParams.faultAngle = document1["SampleParams"]["Angle"].GetDouble();
    calcParams.sampleRadius = document1["SampleParams"]["Radius"].GetDouble();

    }//json file scope

}

GetData::~GetData()
{
    timer_fire=false;
    if (timer_thread.joinable())
    {
        timer_thread.join();
    }
}



void GetData::start()
{

    std::cout << this << " Starting Data" <<std::endl;

    for (int i = 0; i < PID_CHANNELS; i++)
    {
        PIDch[i].begin(&PIDInput[i],
                       &PIDOutput[i],
                       &PIDSetpoint[i],
                       &PIDManualOP[i],
                       PIDcoeffIn[i].kP,
                       PIDcoeffIn[i].kI,
                       PIDcoeffIn[i].kD,
                       PIDcoeffIn[i].bias,
                       timestep);

       PIDch[i].setOutputLimits(PIDcoeffIn[i].outputMin, PIDcoeffIn[i].outputMax);
       PIDch[i].setWindUpLimits(PIDcoeffIn[i].windupMin, PIDcoeffIn[i].windupMax);
       PIDch[i].setDeadBand(PIDcoeffIn[i].deadbandMin,PIDcoeffIn[i].deadbandMax);
    }


    #ifdef REALDATA

        #ifdef HARDWARE_UEI
            DqInitDAQLib();
    
            // open communication with IOM
            if ((ret = DqOpenIOM("127.0.0.1", DQ_UDP_DAQ_PORT, 2000, &hd0, NULL)) < 0) {
                printf("Error %d In Initializing Communication with IOM\n", ret);
                stop_running = true;
            }

            // Configure gains and input mode here:
            // Or the channel number with DQ_LNCL_DIFF to acquire in differential mode
            // Or the channel number with DQ_LNCL_GAIN(G) to select the gain
            // The AI-217 is differential only so you must specify DQ_LNCL_DIFF
            for(int i=0; i<aiparams.numChannels; i++)
            {
                aiparams.channels[i] |= DQ_LNCL_GAIN(DQ_AI218_GAIN_1) | DQ_LNCL_DIFF;
            }

            // Initial read to program channel list, this is required before programming
            // clocks
            Chk4Err(DqAdv217Read(hd0, aiparams.device, aiparams.numChannels, (uint32*)aiparams.channels, data1.data(), fdata.data()), quit());

            // Read PGA status
            // errchan is a bitfield indicating which PGA channel have errors
            // new data is a bitfield indicating which PGA channel have new data since last read
            Chk4Err(DqAdv217GetPgaStatus(hd0, aiparams.device, &errchan, &newdata, pgadata), quit());

            for (int i = 0; i < DQ_AI217_CHAN; i++) {
                if((newdata >> i) & 0x01) {
                    printf("channel %d has new data\n", i);
                } else {
                    printf("channel %d has no new data\n", i);
                }
                if((errchan >> i) & 0x01) {
                    printf("channel %d is in error:  pgadata = 0x%x\n", i, pgadata[i]);
                    // for PGA error bit field definitions search powerdna.h for DQ_AI217_PGAERR
                } else {
                    printf("channel %d has no PGA error\n", i);
                }
            }

            //AO
            if ((ret = DqAdv3xxWrite(hd0, aoparams.device, aoparams.numChannels, (uint32*)aoparams.channels, 0, NULL, fdata.data())) < 0) {
                printf("Error %d in DqAdv302Write()\n", ret);
            }

            //RO
            // Set hysteresis (low is 20%, high is 80%) on DIO-401/4/5/6
            ret = DqAdv40xSetHyst(hd0, roparams.device, 0xC8, 0x190);
            if(ret < 0) {
                printf("Error %d in setting up hysteresis\n", ret);
            }

            //AI
            // Calling DqAdv217Read() only read what's in the layer's input FIFO.
            // Calling DqAdv217Read faster than the speed at which the layer updates
            // its FIFO will return the same values multiple times.
            //
            // Calling DqCmdSetClock() specifies how often the layer transfers data from
            // the AD converters to its input FIFO.
            //
            //
            // Program scan clock (aka CL clock) to acquire 1000.0 scan/s.
            // The function DqCmdSetClock can program clocks on several layers at once.
            // Here we only program one layer
            uint32_t clkEntries;
            float actualClkRate, clkRate;
            clkEntries = 1;
            clkSet.dev = aiparams.device | DQ_LASTDEV; // last (ans only) device in the list
            clkSet.ss = DQ_SS0IN; // program input subsystem clock
            clkSet.clocksel = DQ_LN_CLKID_CVIN; // Program the scan clock, convert clock will be set automatically
            clkRate = (float)aiparams.frequency;
            memcpy((void*)&clkSet.frq, (void*)&clkRate, sizeof(clkSet.frq));
            Chk4Err(DqCmdSetClock(hd0, &clkSet, &actualClkRate, &clkEntries), quit());

            // Use -p option with any value to turn on BIT
            if(aiparams.numArbParams > 0) {
                // Connect 5V test signal to all input channels
                Chk4Err(DqAdv218SetBITMux(hd0, aiparams.device, DQ_AI218_BIT_ALL_CHAN, DQ_AI218_BIT_5V_IN), quit());
            } else {
                // Connect input signal to all channel
                Chk4Err(DqAdv218SetBITMux(hd0, aiparams.device, DQ_AI218_BIT_ALL_CHAN, DQ_AI218_BIT_OFF), quit());
            }

            //AO
            // Program scan clock (aka CL clock) to generate 1000.0 samples/s.
            // The function DqCmdSetClock can program clocks on several layers at once.
            // Here we only program one layer
            clkEntries = 1;
            clkSet.dev = aoparams.device | DQ_LASTDEV; // last (ans only) device in the list 
            clkSet.ss = DQ_SS0OUT; // program output subsystem clock
            clkSet.clocksel = DQ_LN_CLKID_CVOUT; // Program the convert clock
            clkRate = (float)aoparams.frequency*aoparams.numChannels; 
            memcpy((void*)&clkSet.frq, (void*)&clkRate, sizeof(clkSet.frq));
            if((ret = DqCmdSetClock(hd0, &clkSet, &actualClkRate, &clkEntries)) < 0) {
                printf("Error %d in aoDqCmdSetClock()\n", ret);
            }
        #endif            
        #ifdef HARDWARE_LABJACK
        #endif
    
    #else //simulated!!

        Rig1.updateState(fdata.data(), outData.data(), relayData.data(), timestep);


        /*fictitious startup to test!*/
        PIDSetpoint[0] = 0.01;
        PIDch[0].unclampOutput();
        PIDch[0].controlModeAuto();

        PIDSetpoint[1] = 10;
        PIDch[1].unclampOutput();
        PIDch[1].controlModeAuto();

        PIDSetpoint[2] = 0.001;
        PIDch[2].unclampOutput();
        PIDch[2].controlModeAuto();

    #endif


    std::cout << "Finished GetData Startup" << std::endl;
    std::cout << "Thread: " << std::this_thread::get_id() << std::endl;

    timer_fire=true;

    gettimeofday(&tv1, NULL);
}

void GetData::quit()
{
    std::cout << this << "Quitting data";

    #ifdef REALDATA
        #ifdef HARDWARE_UEI
            if (hd0) {
                DqCloseIOM(hd0);
            }
            DqCleanUpDAQLib();
        #endif
        #ifdef HARDWARE_LABJACK
        #endif
    #else

    #endif

    timer_fire=false;
    finished();
}

void GetData::stop_data()
{
    (*this).stop_running = true;
}


void GetData::setpointUpdate(std::tuple<uint32_t, double> data){
    uint32_t ch = std::get<0>(data);
    double SP = std::get<1>(data);
    PIDSetpoint[ch]=SP;
}


void GetData::manualOPUpdate(std::tuple<uint32_t, double> data){
    uint32_t ch = std::get<0>(data);
    double mOP = std::get<1>(data);
    PIDManualOP[ch]=mOP;
}

void GetData::autoCtrlUpdate(std::tuple<uint32_t, uint32_t> data){
    uint32_t ch = std::get<0>(data);
    uint32_t autoCtrl = std::get<1>(data);    if (autoCtrl >= 1){
        PIDch[ch].controlModeAuto();
    }else{ //zero
        PIDch[ch].controlModeMan();
    }
}

void GetData::lockUpdate(std::tuple<uint32_t, uint32_t> data){
    uint32_t ch = std::get<0>(data);
    uint32_t locked = std::get<1>(data);
    if(locked == 0){
        PIDch[ch].unclampOutput();
    }else{
        PIDch[ch].clampOutput();
    }
}

void GetData::controlUpdate(dataStructures1::commandUpdatetype data){
    uint32_t ch = std::get<0>(data);
    setpointUpdate(std::make_tuple(ch, std::get<1>(data)));
    manualOPUpdate(std::make_tuple(ch, std::get<2>(data)));
    autoCtrlUpdate(std::make_tuple(ch, std::get<3>(data)));
    lockUpdate(std::make_tuple(ch, std::get<4>(data)));
}


void GetData::inputMapUpdate(uint32_t Ch, uint32_t inputCh){
    inputMap[Ch]=inputCh;
}
void GetData::outputMapUpdate(uint32_t Ch, uint32_t outputCh){
    for (int i = 0; i<PID_CHANNELS; i++){
        if (outputMap[i]==outputCh && DUMMY_CHANNELMIN>outputCh){
            return; //don't allow multiple outputs to the same channel. Must set to dummy channel first.
        }
        if (outputCh==DOUBLEDChan0+1){
            return; //don't allow outputs to the second channel of a doubled channel 
        }
    }
    PIDch[Ch].clampOutput();
    outputMap[Ch]=outputCh;
    PIDch[Ch].resetState(); //reset PID state because mistakes here are dangerous
}

void GetData::voltsConversionUpdate(uint32_t Ch, double conversion){
    voltsConversion[Ch]=conversion;
}

void GetData::coeffUpdate(dataStructures1::coeffUpdatetype data){
    uint32_t ch = std::get<0>(data);
    double* coeffs = std::get<1>(data).data();

    switch (std::get<5>(data)){
        case 0:
            break;

        case 1: //Coefficients updated
            PIDch[ch].setOutputLimits(coeffs[4],coeffs[5]);
            PIDch[ch].setWindUpLimits(coeffs[6],coeffs[7]);
            PIDch[ch].setDeadBand(coeffs[8],coeffs[9]);

            //resets the PID state
            PIDch[ch].setCoefficients(coeffs[0],coeffs[1],coeffs[2],coeffs[3]);
            PIDch[ch].clampOutput();
            break;

        case 2: //inputMap
            inputMapUpdate(ch, std::get<2>(data));
            break;

        case 3: //outputMap
            outputMapUpdate(ch, std::get<3>(data));
            break;

        case 4: //voltsConversion
            voltsConversionUpdate(ch, std::get<4>(data));
            break;

        case 5: //All
            PIDch[ch].setOutputLimits(coeffs[4],coeffs[5]);
            PIDch[ch].setWindUpLimits(coeffs[6],coeffs[7]);
            PIDch[ch].setDeadBand(coeffs[8],coeffs[9]);
            PIDch[ch].setCoefficients(coeffs[0],coeffs[1],coeffs[2],coeffs[3]);
            PIDch[ch].clampOutput();
            inputMapUpdate(ch, std::get<2>(data));
            outputMapUpdate(ch, std::get<3>(data));
            voltsConversionUpdate(ch, std::get<4>(data));
            break;

        default:
            break;
    }


}



void GetData::doNothing(){
    if (stop_running){
        gettimeofday(&tv2, NULL);
        duration = ((tv2.tv_sec-tv1.tv_sec) + (tv2.tv_usec-tv1.tv_usec)/1000000.0);
        printf("\nExecuted %" PRIu64 " iterations in %f s (%f updates per sec.)\n", it_count, duration, it_count/duration);

        quit();
        return;
    }
}

void GetData::timedUpdate()
{
    if (stop_running){
        gettimeofday(&tv2, NULL);
        duration = ((tv2.tv_sec-tv1.tv_sec) + (tv2.tv_usec-tv1.tv_usec)/1000000.0);
        printf("\nExecuted %" PRIu64 " iterations in %f s (%f updates per sec.)\n", it_count, duration, it_count/duration);

        quit();
        return;
    }


    #ifdef REALDATA
        #ifdef HARDWARE_UEI
        Chk4Err(DqAdv217Read(hd0, aiparams.device, aiparams.numChannels, (uint32*)aiparams.channels, data1.data(), fdata.data()), quit());
        Chk4Err(DqAdv3xxWrite(hd0, aoparams.device, aoparams.numChannels, (uint32*)aoparams.channels, 0, NULL, outData.data()), quit());

        uint32_t datain = 0;
        uint32_t dataout = RELAY_Bool_to_uint32(relayData);
        if ((ret = DqAdv40xWrite(hd0, roparams.device, dataout)) < 0) {
            printf("Error %d in DqAdv40xWrite()\n", ret);
        }

        if ((ret = DqAdv40xRead(hd0, roparams.device, &datain)) < 0) {
            printf("Error %d in DqAdv40xRead()\n", ret);
        }
        //printf("->%08x, <-%08x\n", dataout, datain);
        #endif
        #ifdef HARDWARE_LABJACK
        #endif

    #else
    Rig1.updateState(fdata.data(), outData.data(), relayData.data(), timestep);
    #endif
    
    if (it_count==10){
        scaledVals1000[calcParams.lvdt_Ch] == aiEmit1000[calcParams.lvdt_Ch]; //still no way to reset this on the fly, should be a config param
    }

    physCalc::aiToCalc(fdata.data(), ai_cal_scaling, ai_cal_offsets, scaledVals1000,  calcParams);
    physCalc::copyScaledAi(fdata.data(), aiEmit1000.data());
    physCalc::copyFullCalcs(scaledVals1000, calcEmit1000.data());

    //map in/outputs to PID
    for (int i = 0; i<PID_CHANNELS; i++){
        if (inputMap[i]<INPUT_CHANNELS+CALC_CHANNELS){
            PIDInput[i]=scaledVals1000[inputMap[i]];
        } else {
            PIDInput[i]=0.0;
        }

        PIDch[i].compute(); //PID calculation

        //Connect PID Output to real output
        //outData[0] = PIDOutput[0]; //load control
        //outData[0] = PIDOutput[2]; //displacement control
        if (outputMap[i]<OUTPUT_CHANNELS){
            outData[outputMap[i]] = PIDOutput[i]*voltsConversion[i];
        }
        else if (outputMap[i]>OUTPUT_CHANNELS && outputMap[i] < OUTPUT_CHANNELS+RELAY_CHANNELS-1){
            uint32_t relayindexHigh = outputMap[i]-OUTPUT_CHANNELS-1+1;
            uint32_t relayindexLow = outputMap[i]-OUTPUT_CHANNELS-1;
            if (PIDOutput[i]>=1.0){ //pressure relays
                relayData[relayindexHigh] = 1;
            }else{
                relayData[relayindexHigh] = 0;
            }
            if (PIDOutput[i]<=-1.0){
                relayData[relayindexLow] = 1;
            }else{
                relayData[relayindexLow] = 0;
            }
        }


        if (outputMap[i]==DOUBLEDChan0) //logic for split output and enable relay to loading ram MOOG controller
        {
            double vOutHi, vOutLo, vin;
            vin = PIDOutput[i]*voltsConversion[i];
            calculate_split_value(&vin, DACHRange, DACbitsFR, SOARatio, &vOutHi, &vOutLo);
            outData[outputMap[i]] = vOutHi;
            outData[outputMap[i]+1] = vOutLo;

            if (PIDch[i].clamped() == 0){ LoadSafe = 1;}//set OK bit
            else{LoadSafe=0;}
            relayData[RELAY_CHANNELS-1] = LoadSafe; //set enable relay for MOOG controller
        }
        
    }
        

    //sucessive decimated iterators
    it_dec1000 = (it_dec1000 +  1) %  10;
    it_dec100 = (it_dec1000 ==  0) ? (it_dec100 +  1) %  10 : it_dec100;
    it_dec10 = (it_dec100 ==  0) ? (it_dec10 +  1) %  10 : it_dec10;


    //printf("\n");
    it_count +=1;

    std::get<0>(dataOutStage)=it_count;
    std::get<1>(dataOutStage)=aiEmit1000;
    std::get<2>(dataOutStage)=outData;
    std::get<3>(dataOutStage)=relayData;
    std::get<4>(dataOutStage)=calcEmit1000;
    std::get<5>(dataOutStage)=calcParams;

    dq.push(dataOutStage);




    if ((it_dec1000 + it_dec100) == 0){
        for (int i = 0 ; i<PID_CHANNELS; i++){
            std::get<0>(pidCoeffOut)[i]=PIDch[i]; //copy? probably not very efficient
        }
        //std::get<0>(pidCoeffOut)=; //would be a copy
        std::get<1>(pidCoeffOut)=inputMap;
        std::get<2>(pidCoeffOut)=outputMap;
        std::get<3>(pidCoeffOut)=voltsConversion;

        pCq.push(pidCoeffOut);

        //these all exist in PID structure passed for coefficients     
        // // Assign values to the elements of the pidIOOut tuple
        // std::get<0>(pidIOOut) = PIDInput;
        // std::get<1>(pidIOOut) = PIDOutput;
        // std::get<2>(pidIOOut) = PIDSetpoint;
        // std::get<3>(pidIOOut) = PIDManualOP;

    }    


    if (!coeq.consume_one([this](dataStructures1::coeffUpdatetype d){this->coeffUpdate(d);})){};

    if (!comq.consume_one([this](dataStructures1::commandUpdatetype d){this->controlUpdate(d);})){};

    m_count+=1;

}

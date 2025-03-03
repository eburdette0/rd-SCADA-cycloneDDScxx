#include "networkData.h"

#include <cstdint>

#include "definitions.h"
#include "hardwareInterface/getdata.h"
#include "PID/PID_V1.h"

#include "dds/dds.hpp"
#include "../include/DDSsources/networkDataStructures.hpp"

#include <stdio.h>
#include <vector>

#include "rapidjson/document.h" //for parsing calibration values

#include <fstream>
#include <string>

#include <sstream>


std::vector<std::string> split(const std::string &s, char delim) { //https://stackoverflow.com/questions/9435385/split-a-string-using-c11
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> elems;
  while (std::getline(ss, item, delim)) {
    //elems.push_back(item);
    elems.push_back(std::move(item)); // if C++11 (based on comment from @mchiasson)
  }
  return elems;
}

bool vecBaseCompare(std::vector<std::string> message, std::vector<std::string> base){
    bool temp;
    if (std::equal(base.begin(), base.end(), message.begin())){
        temp = true;
    }else{
        return false;
    }
    return true;
}
bool stringBaseCompare(std::string message, std::string base){
    bool temp;
    if (std::equal(base.begin(), base.end(), message.begin())){
        temp = true;
    }else{
        return false;
    }
    return true;
}


/***********************************/

using namespace org::eclipse::cyclonedds;


/**** Class ****/
networkData::networkData(std::string filename,
        queuedefs::dataQueueSlowdef& dataQueue,
        queuedefs::pidCoeffQueuedef& pidCoeffQueue,
        queuedefs::pidCoeffUpdateQueuedef& pidCoeffUpdateQueue,
        queuedefs::pidCommandUpdateQueuedef& pidCommandUpdateQueue) :

    /**initialize templated types/objects when the class is created**/
    participant(1),
    //msgTopic(participant, "Msg"),
    ChInfoTopic(participant, "ChInfo"),
    BundleTopic(participant, "Bundle"),
    SinglePtTopic(participant, "SlowBundle"),
    PIDStateTopic(participant, "PIDState"),
    PIDCoeffTopic(participant, "PIDCoeffUpdate"),
    pidControlsTopic(participant, "PIDControlsUpdate"),


    publisher(participant), //basic, may need more for multiple QOS

    //MsgWriter(publisher, msgTopic),
    BundleWriter(publisher, BundleTopic),
    SinglePtWriter(publisher, SinglePtTopic),
    PIDStateWriter(publisher, PIDStateTopic),

    subscriber(participant),

    //attach listener in body in case publication is already matched and class isn't completely instanciated
    chinfolistener(*this),
    //ChInfoWriter(publisher, ChInfoTopic, dwqos, &chinfolistener, dds::core::status::StatusMask::publication_matched()),
    ChInfoWriter(publisher, ChInfoTopic), 


    pidCoeffListener(*this),
    pidControlsListener(*this),
    //pidCoeffReader(subscriber, PIDCoeffTopic, dds::sub::qos::DataReaderQos(), &pidCoeffListener, dds::core::status::StatusMask::data_available()),
    pidCoeffReader(subscriber, PIDCoeffTopic),
    //pidControlsReader(subscriber, pidControlsTopic, dds::sub::qos::DataReaderQos(), &pidControlsListener, dds::core::status::StatusMask::data_available())
    pidControlsReader(subscriber, pidControlsTopic),

    _dataQueue(dataQueue),
    _pidCoeffQueue(pidCoeffQueue),
    _pidCoeffUpdateQueue(pidCoeffUpdateQueue),
    _pidCommandUpdateQueue(pidCommandUpdateQueue)
    

{    //Networkdata:NetworkData body
    
    if (!validateConfigurationFile(filename)){
        throw std::runtime_error("Invalid/missing configuration file for networkData");
    }


    dds::pub::DataWriter<networkDataStructures::ChInfo> tempWriter(publisher, ChInfoTopic, dwqos, &chinfolistener, dds::core::status::StatusMask::publication_matched());
    ChInfoWriter = tempWriter;
    dds::sub::DataReader<networkDataStructures::PIDCoeff> tempReader1(subscriber, PIDCoeffTopic, dds::sub::qos::DataReaderQos(), &pidCoeffListener, dds::core::status::StatusMask::data_available());
    pidCoeffReader = tempReader1;
    dds::sub::DataReader<networkDataStructures::PIDControls> tempReader2(subscriber, pidControlsTopic, dds::sub::qos::DataReaderQos(), &pidControlsListener, dds::core::status::StatusMask::data_available());
    pidControlsReader = tempReader2;


    
    std::ifstream ifs(filename);
    std::string content( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );
    JSONinput = content;

    rapidjson::Document document1;
    document1.Parse(content.c_str());
    if (document1.HasParseError()) {
        std::cerr << "Error: Failed to parse JSON from file. Error offset: " << document1.GetErrorOffset() << std::endl;
        std::exit(1);
    }

    machineString = document1["MachineName"].GetString();

    iterationTopic = "/"+machineString+"/phys/iteration";

    allinfoT = "/"+machineString+"/info";

    if (document1["Channels"].HasMember("phys")) {
        if  (document1["Channels"]["phys"].HasMember("ai")){
            
            aiTopicName = "/"+machineString+"/phys/ai";
            aiBundleTopicName = aiTopicName + "/bundle";
            aiBundleLengthT = aiTopicName + "/bundle/length"; //double
            aiBundleinfoT = aiTopicName +"/bundle/chInfo"; //write raw JSON to this
            for (int i=0; i<INPUT_CHANNELS; i++){
                aiTopicNames[i]=aiTopicName+"/Ch" + (i<10? "0":"") + std::to_string(i);
            }
        }


        if  (document1["Channels"]["phys"].HasMember("ao")){
            std::string aoTopicName_s = std::string("/"+machineString+"/phys/ao");
            aoTopicName = aoTopicName_s.c_str();
            aoBundleTopicName = std::string(aoTopicName_s + "/bundle").c_str();
            aoBundleLengthT = std::string(aoTopicName_s + "/bundle/length").c_str(); //double
            aoBundleinfoT = std::string(aoTopicName_s +"/bundle/chInfo").c_str(); //write raw JSON to this
            for (int i=0; i<OUTPUT_CHANNELS; i++){
                aoTopicNames[i]=aoTopicName_s+"/Ch" + (i<10? "0":"") + std::to_string(i);
            }
        }


        if  (document1["Channels"]["phys"].HasMember("ro")){
            std::string roTopicName_s = std::string("/"+machineString+"/phys/ro");
            roTopicName = roTopicName_s.c_str();
            roBundleTopicName = std::string(roTopicName_s + "/bundle").c_str();
            roBundleLengthT = std::string(roTopicName_s + "/bundle/length").c_str(); //double
            roBundleinfoT = std::string(roTopicName_s +"/bundle/chInfo").c_str(); //write raw JSON to this
            for (int i=0; i<RELAY_CHANNELS; i++){
                roTopicNames[i]=roTopicName_s+"/Ch" + (i<10? "0":"") + std::to_string(i);
            }  
        }

        if  (document1["Channels"]["soft"].HasMember("calculated")){
            std::string calcTopicName_s = std::string("/"+machineString+"/soft/calculated");
            calcBundleTopicName = std::string(calcTopicName_s+"/bundle");
            calcbundleLengthT = std::string(calcTopicName_s+"/bundle/length");
            for (int i=0; i<CALC_CHANNELS; i++){
                calcTopicNames[i]=calcTopicName_s+"/Ch" + (i<10? "0":"") + std::to_string(i);
            }

        }



        

    }
}

// Static method to validate configuration file
bool networkData::validateConfigurationFile(const std::string& filename) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        throw std::runtime_error("Failed to open configuration file: " + filename);
    }

    std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());
    ifs.close();

    rapidjson::Document document;
    if (document.Parse(content.c_str()).HasParseError()) {
        throw std::runtime_error("Invalid JSON format in configuration file");
    }

    // Validate required fields
    if (!document.HasMember("MachineName") || !document["MachineName"].IsString()) {
        throw std::runtime_error("Configuration file missing MachineName field");
    }

    return true;
}


void networkData::connect(){

    //disconnect/delete stuff first if reconnecting??


    // dwqos = ChInfoTopic.qos();
    // dwqos << dds::core::policy::Durability(dds::core::policy::DurabilityKind::TRANSIENT_LOCAL) //saves samples depending on history settings
    //         << dds::core::policy::Reliability(dds::core::policy::ReliabilityKind::RELIABLE); //will redeliver samples if not ack'd
    //     //listener defined in header
    // dds::pub::DataWriter<networkDataStructures::ChInfo> tempWriter(publisher, ChInfoTopic, dwqos, &chinfolistener, dds::core::status::StatusMask::publication_matched());
    // ChInfoWriter = tempWriter;

    networkDataStructures::ChInfo chinfo(1, JSONinput.c_str());
    ChInfoWriter.write(chinfo);

}

void networkData::qstart(){
    //start the queue monitor
    dataThreadRunning = true;
    dataMonitorThread = std::thread(&networkData::dataMonitor, this);
    pidCThreadRunning = true;
    pidCMonitorThread = std::thread(&networkData::pidCMonitor, this);
}
void networkData::qstop(){
    dataThreadRunning = false;
    pidCThreadRunning = false;
    if (dataMonitorThread.joinable()) {
        dataMonitorThread.join();
    }
    if (pidCMonitorThread.joinable()) {
        pidCMonitorThread.join();
    }

}
void networkData::dataMonitor(){
    while (dataThreadRunning) {
        if(!_dataQueue.consume_one([this](dataStructures1::dataTuple d){this->publishValsMQTT(d);})){
            //handle error
        };
        if (_dataQueue.read_available()==0){
            std::this_thread::sleep_for(std::chrono::microseconds((1000000/INPUT_FREQ)*1/5));
        }
    }
}
void networkData::pidCMonitor(){
    while (pidCThreadRunning) {
        if(!_pidCoeffQueue.consume_one([this](dataStructures1::pidCoeffTuple d){this->publishPIDpar(d);})){
            //handle error
            };
        if (_pidCoeffQueue.read_available()==0){
            std::this_thread::sleep_for(std::chrono::microseconds((1000000/INPUT_FREQ)*1/5));
        }
    }
}



void networkData::coeffUpdate_signaller(networkDataStructures::PIDCoeff data){
    std::array<double, 10> temp;
    temp[0] = data.kP();
    temp[1] = data.kI();
    temp[2] = data.kD();
    temp[3] = data.bias();
    temp[4] = data.outputMax();
    temp[5] = data.outputMin();
    temp[6] = data.windupMax();
    temp[7] = data.windupMin();
    temp[8] = data.deadbandMax();
    temp[9] = data.deadbandMin();
    //coeffUpdate.fire(std::make_tuple(data.channel(), temp));
    _pidCoeffUpdateQueue.push(std::make_tuple(data.channel(), temp, data.inputMap(), data.outputMap(), data.voltsConversion(), data.controlUpdateSelector() ));


    // networkDataStructures::Msg msgContainer(1, "coeffUpdate initiated");
    // MsgWriter.write(msgContainer);
    
    //TODO: input source changes for load/disp control switching
}

void networkData::controlUpdate_signaller(networkDataStructures::PIDControls data){
    //controlUpdate.fire(std::make_tuple(data.channel(), data.setpoint(), data.manualOP(), data.autoCtrl(), data.clamped()));
    _pidCommandUpdateQueue.push(std::make_tuple(data.channel(), data.setpoint(), data.manualOP(), data.autoCtrl(), data.clamped()));
    
    // networkDataStructures::Msg msgContainer(1, "controlUpdate initiated");
    // MsgWriter.write(msgContainer);
}




void networkData::publishValsMQTT(dataStructures1::dataTuple data){
    uint64_t iteration = std::get<0>(data);
    std::array<double, INPUT_CHANNELS> aiValue = std::get<1>(data);
    std::array<double, OUTPUT_CHANNELS> aoValue = std::get<2>(data);
    std::array<bool, RELAY_CHANNELS> roValue = std::get<3>(data);
    std::array<double, INPUT_CHANNELS+CALC_CHANNELS> calcValue = std::get<4>(data);


    //networkDataStructures::BundleStruct bundleContainer(1,
                                                        // iteration,
                                                        // aiValue,
                                                        // aoValue,
                                                        // roValue,
                                                        // calcValue);
    bundleContainer.machineID(1);
    bundleContainer.iteration(iteration);
    bundleContainer.aiBundle(aiValue);
    bundleContainer.aoBundle(aoValue);
    bundleContainer.roBundle(roValue);
    bundleContainer.calcBundle(calcValue);

    BundleWriter.write(bundleContainer);    
}



void networkData::publishPIDpar(dataStructures1::pidCoeffTuple data){


    std::array<PID, PID_CHANNELS> pidElements = std::get<0>(data);
    std::array<uint32_t, PID_CHANNELS> inputmap = std::get<1>(data);
    std::array<uint32_t, PID_CHANNELS> outputmap = std::get<2>(data);
    std::array<double, PID_CHANNELS> voltsConversion = std::get<3>(data);



    for (int i=0; i<PID_CHANNELS; i++){

        PIDcoeff temp = pidElements[i].getCoeffs();

        PIDStateContainer.pidCoeffs()[i].channel(i);
        //PIDStateContainer.pidCoeffs()[i].controlName(temp.controlName);
        PIDStateContainer.pidCoeffs()[i].kP(temp.kP);
        PIDStateContainer.pidCoeffs()[i].kI(temp.kI);
        PIDStateContainer.pidCoeffs()[i].kD(temp.kD);
        PIDStateContainer.pidCoeffs()[i].bias(temp.bias);
        PIDStateContainer.pidCoeffs()[i].outputMax(temp.outputMax);
        PIDStateContainer.pidCoeffs()[i].outputMin(temp.outputMin);
        PIDStateContainer.pidCoeffs()[i].windupMax(temp.windupMax);
        PIDStateContainer.pidCoeffs()[i].windupMin(temp.windupMin);
        PIDStateContainer.pidCoeffs()[i].deadbandMax(temp.deadbandMax);
        PIDStateContainer.pidCoeffs()[i].deadbandMin(temp.deadbandMin);
        PIDStateContainer.pidCoeffs()[i].inputMap(inputmap[i]);
        PIDStateContainer.pidCoeffs()[i].outputMap(outputmap[i]);
        PIDStateContainer.pidCoeffs()[i].voltsConversion(voltsConversion[i]);
        PIDStateContainer.pidCoeffs()[i].controlUpdateSelector(0); //0 for no update, cases for other updates

        PIDStateContainer.pidControls()[i].channel(i);
        PIDStateContainer.pidControls()[i].setpoint(pidElements[i].curSetpoint);
        PIDStateContainer.pidControls()[i].manualOP(pidElements[i].newManualOutput);
        PIDStateContainer.pidControls()[i].autoCtrl((pidElements[i].controlMode()==1));
        PIDStateContainer.pidControls()[i].clamped((pidElements[i].clamped()==1));

        PIDStateContainer.pidIO()[i].channel(i);
        PIDStateContainer.pidIO()[i].PV(pidElements[i].curInput);
        PIDStateContainer.pidIO()[i].SP(pidElements[i].curSetpoint);
        PIDStateContainer.pidIO()[i].OP(pidElements[i].newOutput);
        PIDStateContainer.pidIO()[i].manOP(pidElements[i].newManualOutput);
    }
    
    PIDStateWriter.write(PIDStateContainer);

}


#ifndef NETWORKDATA_H
#define NETWORKDATA_H

#include <string>
#include <vector> //for string concatenation

#include "hardwareInterface/getdata.h"
#include "PID/PID_V1.h"


#include "dds/dds.hpp"
#include "../include/DDSsources/networkDataStructures.hpp"


class networkData
{

public:
    explicit networkData(std::string filename, queuedefs::dataQueueSlowdef& dataQueue, 
            queuedefs::pidCoeffQueuedef& pidCoeffQueue, 
            queuedefs::pidCoeffUpdateQueuedef& pidCoeffUpdateQueue,
            queuedefs::pidCommandUpdateQueuedef& pidCommandUpdateQueue);

    bool validateConfigurationFile(const std::string& filename);
    
    //int subscribePIDpar();
    //void handlePIDmsg(std::string topicString, const struct mosquitto_message *message);
    std::string makepidChstring(std::string machine, int ChNo);

    void connect();

    void qstart();
    void qstop();
    void dataMonitor();
    void pidCMonitor();

    void controlUpdate_signaller(networkDataStructures::PIDControls);
    void coeffUpdate_signaller(networkDataStructures::PIDCoeff);

    void publishValsMQTT(dataStructures1::dataTuple); //uint64_t iteration, double aiValue[INPUT_CHANNELS+CALC_CHANNELS], double aoValue[OUTPUT_CHANNELS], bool roValue[RELAY_CHANNELS], double calcValue[CALC_CHANNELS]);
    void publishPIDpar(dataStructures1::pidCoeffTuple);//PID pidElements[], uint32_t inputmap[], uint32_t outputmap[], double voltsConversion[]);
    // void publishPIDIO(std::tuple<std::array<double, PID_CHANNELS>, 
    //                     std::array<double, PID_CHANNELS>, 
    //                     std::array<double, PID_CHANNELS>, 
    //                     std::array<double, PID_CHANNELS>>);//double PV[], double OP[], double SP[], double manOP[]);
    


private:

    queuedefs::dataQueueSlowdef& _dataQueue;
    queuedefs::pidCoeffQueuedef& _pidCoeffQueue;
    queuedefs::pidCoeffUpdateQueuedef& _pidCoeffUpdateQueue;
    queuedefs::pidCommandUpdateQueuedef& _pidCommandUpdateQueue;
    
    std::atomic<bool> dataThreadRunning;
    std::atomic<bool> pidCThreadRunning;
    std::thread dataMonitorThread;
    std::thread pidCMonitorThread;
    

    int bundlePublish(std::string BundleTopicName, double Values[], unsigned int channels, int qos);
    

    dds::domain::DomainParticipant participant;

    /* To publish something, a topic is needed. */
    //dds::topic::Topic<networkDataStructures::Msg> msgTopic;
    dds::topic::Topic<networkDataStructures::ChInfo> ChInfoTopic;
    dds::topic::Topic<networkDataStructures::BundleStruct> BundleTopic;
    dds::topic::Topic<networkDataStructures::BundleStruct> SinglePtTopic;
    dds::topic::Topic<networkDataStructures::PIDState> PIDStateTopic;
    dds::topic::Topic<networkDataStructures::PIDCoeff> PIDCoeffTopic;
    dds::topic::Topic<networkDataStructures::PIDControls> pidControlsTopic;

    /* A writer also needs a publisher. */
    dds::pub::Publisher publisher;
    /* A Reader needs a subscriber */
    dds::sub::Subscriber subscriber;

    /* writers */
    //dds::pub::DataWriter<networkDataStructures::Msg> MsgWriter;
    dds::pub::DataWriter<networkDataStructures::ChInfo> ChInfoWriter; //redifined with a listener in connect function
    dds::pub::qos::DataWriterQos dwqos;
    dds::pub::DataWriter<networkDataStructures::BundleStruct> BundleWriter;
    dds::pub::DataWriter<networkDataStructures::BundleStruct> SinglePtWriter;
    dds::pub::DataWriter<networkDataStructures::PIDState> PIDStateWriter;

    /* readers */
    dds::sub::DataReader<networkDataStructures::PIDCoeff> pidCoeffReader;
    dds::sub::DataReader<networkDataStructures::PIDControls> pidControlsReader;

    //instanciate the data structures so we can reuse them
    networkDataStructures::BundleStruct bundleContainer = {};
    networkDataStructures::PIDState PIDStateContainer = {};
    networkDataStructures::PIDCoeff PIDCoeffContainer = {};
    networkDataStructures::PIDControls PIDControlsContainer = {};
    

    class ChInfoListener : public dds::pub::NoOpDataWriterListener<networkDataStructures::ChInfo> {
        networkData& nd;  // Add a reference to this networkData class
    public:
        ChInfoListener(networkData& nd) : nd(nd) {}  // Initialize it in the constructor

        void on_publication_matched(dds::pub::DataWriter<networkDataStructures::ChInfo>& writer, const dds::core::status::PublicationMatchedStatus& status) {
            networkDataStructures::ChInfo msg(1, nd.JSONinput.c_str());  // Use the reference here
            writer.write(msg);
        }
    };
    
    ChInfoListener chinfolistener;


    //instanciate the listener for PIDCoeff which will read the PID Coeffcients to be changed
    class PIDCoeffListener : public dds::sub::NoOpDataReaderListener<networkDataStructures::PIDCoeff> {
        networkData& nd;  // Add a reference to this networkData class
    public:
        PIDCoeffListener(networkData& nd) : nd(nd) {}  // Initialize it in the constructor

        void on_data_available(dds::sub::DataReader<networkDataStructures::PIDCoeff>& reader) {
            auto samples = reader.take();
            for (const auto& sample : samples) {
                if (sample.info().valid()) {
                    nd.coeffUpdate_signaller(sample.data()); //pass by value
                }
            }
        }
    };

    PIDCoeffListener pidCoeffListener;

    class PIDControlsListener : public dds::sub::NoOpDataReaderListener<networkDataStructures::PIDControls> {
        networkData& nd;  // Add a reference to this networkData class
    public:
        PIDControlsListener(networkData& nd) : nd(nd) {}  // Initialize it in the constructor

        void on_data_available(dds::sub::DataReader<networkDataStructures::PIDControls>& reader) {
            auto samples = reader.take();
            for (const auto& sample : samples) {
                if (sample.info().valid()) {
                    nd.controlUpdate_signaller(sample.data()); //pass by value
                }
            }
        }
    };

    PIDControlsListener pidControlsListener;



    const char mqttHost[32] = "127.0.0.1";
    const char mqttUser[32] = "";
    const char mqttPassword[32] = "";
    const int mqttPort = 1883;
    int ret;
    char topicName[64];
    char valueString[32];
    int mqtt_keepalive = 60;



    
    std::string JSONinput;
    std::string machineString;

    std::string allinfoT = ""; //all json data

    std::string iterationTopic= "machine0/phys/ai/iteration";

    std::string aiBundleTopicName = "/machine0/phys/ai/bundle";
    std::string aiBundleLengthT = "/machine0/phys/ai/bundle/length"; //double
    std::string aiBundleinfoT = "/machine0/phys/ai/bundle/chInfo"; //packed
    std::string aiTopicName = "/machine0/phys/ai";
    std::string aiTopicNames[INPUT_CHANNELS];


    std::string aoBundleTopicName = "/machine0/phys/ao/bundle";
    std::string aoBundleLengthT = "/machine0/phys/ao/bundle/length"; //double
    std::string aoBundleinfoT = "/machine0/phys/ao/bundle/chInfo"; //packed with commas between each value, semicolons between channels
    std::string aoTopicName = "/machine0/phys/ao";
    std::string aoTopicNames[OUTPUT_CHANNELS];



    std::string roBundleTopicName = "/machine0/phys/ro/bundle";
    std::string roBundleLengthT = "/machine0/phys/ro/bundle/length"; //double
    std::string roBundleinfoT = "/machine0/phys/ro/bundle/chInfo"; //packed with commas between each value, semicolons between channels
    std::string roTopicName = "/machine0/phys/ro";
    std::string roTopicNames[RELAY_CHANNELS];


    std::string calcBundleTopicName = "/machine0/soft/calculated/bundle";
    std::string calcbundleLengthT = "/machine0/soft/calculated/bundle/length";
    std::string calcTopicNames[CALC_CHANNELS];

};



#endif // NETWORKDATA_H

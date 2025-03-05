#include "definitions.h"
#include "hardwareInterface/getdata.h"
#include "decimation/decimator.h"
#include "csvIO/datasaver.hpp"
#include "fileChecks.hpp"
#include "networkData/networkData.h"
#include "display/tui.hpp"
#include "queueTemplate/queueTemplates.hpp"

#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/policies.hpp>

#include <signal.h>
#include <cstring>
#include <chrono>

#define BENCHMARK 1

bool stop=false;
// Handler for SIGINT
void handler(int sig)
{
    stop = true;
}


//instanciate queues that transfer data
namespace blf = boost::lockfree;

queuedefs::calcDataQueueFastdef hardwareOutputQueue;

queuedefs::dataQueueSlowdef dataDecimationQueue100;
queuedefs::dataQueueSlowdef dataDecimationQueue10;
queuedefs::dataQueueSlowdef dataDecimationQueue1;

queuedefs::dataQueueSlowdef dataNetworkQueue;
queuedefs::dataQueueSlowdef csvQueue;
queuedefs::dataQueueSlowdef displayQueue;

queuedefs::pidCoeffQueuedef pidGetQueue;
queuedefs::pidCoeffUpdateQueuedef pidCoeffUpdateQueue;
queuedefs::pidCommandUpdateQueuedef  pidCommandUpdateQueue;

blf::queue<dataStructures1::logTuple, blf::capacity<100>> logQueue;


int main(int argc, char *argv[]){

    // if (argc <= 1) {
    //     std::cout << "Please provide a filename as an argument." << std::endl;
    //     return -1;
    // }
    // std::string filename_unused = argv[1];

    const char* homeDir = std::getenv("HOME");
    if (homeDir == nullptr || *homeDir == '\0') {
        std::cerr << "Error: HOME environment variable not set." << std::endl;
        std::exit(1); // Or throw an exception
    }
    std::string filename = std::string(homeDir) + "/rdControllerConfigs/RigConfig.json";


    int fileRet = configFileChecks(filename);
    if (fileRet != 0) {
        std::exit(1);
    }

    signal(SIGINT, handler);


    //why does this break the code? docs suggest maybe should be set for each pthread?
    // struct sched_param schedp;
    // memset(&schedp, 0, sizeof(schedp));
    // schedp.sched_priority = 40;
    // sched_setscheduler(0, SCHED_FIFO, &schedp);


    std::cout<<"starting main" << std::endl;
    std::cout << "Thread: " << std::this_thread::get_id() << std::endl;




    GetData getdata1(filename, hardwareOutputQueue, pidGetQueue, pidCoeffUpdateQueue, pidCommandUpdateQueue);
    decimator dec1(getdata1.aiScaling, getdata1.aiOffsets, hardwareOutputQueue, dataDecimationQueue100, dataDecimationQueue10, dataDecimationQueue1);
    spsc3FanoutConsumer<dataStructures1::dataTuple> dec1Fan(dataDecimationQueue10, dataNetworkQueue, csvQueue, displayQueue);

    networkData netIO(filename, dataNetworkQueue, pidGetQueue, pidCoeffUpdateQueue, pidCommandUpdateQueue); //already threaded
    dataSaver csv1(csvQueue);
    //displayTable tui1(displayQueue);


    // core and producers
    getdata1.start(); //~6% utilization
    dec1.start(); //~7% utilization
    dec1Fan.start(); //~3% utilization

    //consumers
    csv1.start(); //~3% utilization

    std::cout<<std::endl;
    //tui1.start(); //~16% utilization


    //network comms
    netIO.connect();
    netIO.qstart(); //~6% nothing connected


    int i=0;
    while (!stop)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // if (BENCHMARK>0){
        //     i++;
        //     if (i>=10){break;}
        // }
    }

    std::cout << "Stopping" << std::endl;
    getdata1.stop_data(); //should stop loops

    dec1.stop();
    dec1Fan.stop();

    //tui1.stop();
    csv1.stop();

    netIO.qstop();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "Stopped. Ending" << std::endl;

}

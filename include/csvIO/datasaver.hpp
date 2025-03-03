#ifndef DATASAVER_HPP
#define DATASAVER_HPP

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include <sys/time.h>
#include "queueTemplate/queueTemplates.hpp"
#include "definitions.h" //for number of channels

double localData[INPUT_CHANNELS] {};

class dataSaver : public spscConsumer<dataStructures1::dataTuple>
{
public:

    const size_t MAX_FILE_SIZE = 1000 * 1024 * 1024;
    std::ofstream fs_;
    size_t file_pos_;
    std::string fileName = "./oneHzData_log.csv";// + std::to_string(now_c) + ".csv";
    size_t headerSize = 0;

    dataSaver(boost::lockfree::spsc_queue<dataStructures1::dataTuple, boost::lockfree::capacity<100>>& queue)
        : spscConsumer<dataStructures1::dataTuple>(queue), file_pos_(0)
    {
        fs_.exceptions(std::ios::failbit | std::ios::badbit);


        time_t now_c = startTime();
        writeHeader(now_c);

    }

    ~dataSaver() override {
        fs_.flush();
        fs_.close();
    }

    time_t startTime(){
        auto now = std::chrono::system_clock::now();
        time_t now_c = std::chrono::system_clock::to_time_t(now);
        return now_c;   
    }

    void writeHeader(time_t now_c){
        fs_.open(fileName);
        fs_ << "Restart Epoch " << std::to_string(now_c) << std::endl;
        fs_ << "Iteration, Axial Load, Confining Pressure, LVDT Displacement, Pore Pressure, Ch5, Ch6, Ch7, Ch8"<< std::endl;
        headerSize = fs_.tellp();
    }

    using spscConsumer<dataStructures1::dataTuple>::start;
    using spscConsumer<dataStructures1::dataTuple>::stop;

protected:
    using spscConsumer<dataStructures1::dataTuple>::monitor;
    using spscConsumer<dataStructures1::dataTuple>::running;
    using spscConsumer<dataStructures1::dataTuple>::q;

    void consume(const dataStructures1::dataTuple& data) override
    {

        // Update file position
        file_pos_ = fs_.tellp();

        if (file_pos_ > MAX_FILE_SIZE) {
            // File is full, rotate and reset file position to point after header. 
            //Iteration number contains time index in milliseconds
            fs_.close();
            fs_.open("oneHzData_log.csv", std::ios::ate | std::ios::binary);
            fs_.seekp(headerSize);
            file_pos_ = headerSize;
        }

        uint64_t iteration = std::get<0>(data);
        //double data1Hz[INPUT_CHANNELS] = std::get<1>(data);
        const double* data1Hz = std::get<1>(data).data(); //pointer for C style text writing
        
        fs_ << iteration << ",";
        for (int i=0; i<(INPUT_CHANNELS); i++){
            fs_ << data1Hz[i] << ",";
        }
        fs_ << std::endl;

    }
};

#endif
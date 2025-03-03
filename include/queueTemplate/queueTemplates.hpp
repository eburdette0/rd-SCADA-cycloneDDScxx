#ifndef QUEUE_TEMPLATES_HPP
#define QUEUE_TEMPLATES_HPP

#include <atomic>
#include <thread>
#include <iostream>

#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/queue.hpp>


template<typename T>
class spscConsumer {

public:
    spscConsumer(boost::lockfree::spsc_queue<T, boost::lockfree::capacity<100>>& queue) : q(queue), running(false) 
    {

    }

    virtual ~spscConsumer() {
        stop();
    }

    void start() {
        running = true;
        monitorThread = std::thread(&spscConsumer::monitor, this);
    }

    void stop() {
        running = false;
        if (monitorThread.joinable()) {
            monitorThread.join();
        }
    }

protected:
    boost::lockfree::spsc_queue<T, boost::lockfree::capacity<100>>& q;
    std::atomic<bool> running;
    std::thread monitorThread;

    virtual void monitor() {
        while (running) {
            if(!q.consume_one([this](const T& d){this->consume(d);})){
                //handle error
            };
            if (q.read_available()==0){
                std::this_thread::sleep_for(std::chrono::microseconds((1000000/INPUT_FREQ)*1/5));
            }
        }
    }

    virtual void consume(const T& value) { //override this
        // Act on the value here
        std::cout << "Consumed: not implemented" << std::endl;
    }

};

template<typename T>
class mpmcConsumer {

public:
    mpmcConsumer(boost::lockfree::queue<T, boost::lockfree::capacity<100>>& queue) : q(queue), running(false) 
    {

    }

    void start() {
        running = true;
        monitorThread = std::thread(&mpmcConsumer::monitor, this);
    }

    void stop() {
        running = false;
        if (monitorThread.joinable()) {
            monitorThread.join();
        }
    }

protected:
    boost::lockfree::queue<T, boost::lockfree::capacity<100>>& q;
    std::atomic<bool> running;
    std::thread monitorThread;
    void monitor() {
        while (running) {
            if(!q.consume_one([this](T d){this->consume(d);})){
                std::this_thread::sleep_for(std::chrono::nanoseconds(10000));
            };
        }
    }

    void consume(T value) {
        // Act on the value here
        std::cout << "Consumed: not implemented" << std::endl;
    }

};


template<typename T>
class spsc3FanoutConsumer {

public:
    spsc3FanoutConsumer(boost::lockfree::spsc_queue<T, boost::lockfree::capacity<100>>& queue_source,
                        boost::lockfree::spsc_queue<T, boost::lockfree::capacity<100>>& queue1,
                        boost::lockfree::spsc_queue<T, boost::lockfree::capacity<100>>& queue2,
                        boost::lockfree::spsc_queue<T, boost::lockfree::capacity<100>>& queue3
                        ) : q(queue_source), q1(queue1), q2(queue2), q3(queue3),running(false) 
    {

    }

    void start() {
        running = true;
        monitorThread = std::thread(&spsc3FanoutConsumer::monitor, this);
    }

    void stop() {
        running = false;
        if (monitorThread.joinable()) {
            monitorThread.join();
        }
    }

protected:
    boost::lockfree::spsc_queue<T, boost::lockfree::capacity<100>>& q;
    boost::lockfree::spsc_queue<T, boost::lockfree::capacity<100>>& q1;
    boost::lockfree::spsc_queue<T, boost::lockfree::capacity<100>>& q2;
    boost::lockfree::spsc_queue<T, boost::lockfree::capacity<100>>& q3;
    std::atomic<bool> running;
    std::thread monitorThread;

    // Monitor the source queue 
    void monitor() {
        while (running) {
            if(!q.consume_one([this](T d){this->consume(d);})){
                //check for errors
            };
            if(q.read_available()==0){
                std::this_thread::sleep_for(std::chrono::microseconds((1000000/INPUT_FREQ)*1/5));
            }
        }
    }

    // push fan out to the 3 destination queues
    void consume(T value) {
        q1.push(value);
        q2.push(value);
        q3.push(value);
    }

};


#endif // QUEUE_TEMPLATES_HPP
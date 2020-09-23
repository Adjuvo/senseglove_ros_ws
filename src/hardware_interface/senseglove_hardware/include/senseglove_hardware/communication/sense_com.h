// Copyright 2020 Senseglove.
#ifndef ROS_WORKSPACE_SENSE_COM_H
#define ROS_WORKSPACE_SENSE_COM_H

#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <ros/ros.h>

namespace senseglove
{
    class Sensecom
    {
    public:
        Sensecom(int max_slave_index, int cycle_time);
        ~Sensecom();

        bool isOperational() const;

        /**
         * returns the cycle time
         * @return cycle time in milliseconds
         */
        int getCycleTime() const;

        /**
         * Starts the communication for a SenseGlove
         * TO DO: Start a thread that communicates between the SenseGlove and the ROS master
         */
        void start();

        /**
         * Stops the communication bus
         * TO DO: Join the thread
         */
        void stop();

    private:
        std::atomic<bool> is_operational_;
        const int max_slave_index_;
        const int cycle_time_ms_;

        std::thread communication_thread_;
    };
}


#endif //ROS_WORKSPACE_SENSE_COM_H

// Copyright 2020 Senseglove.
#ifndef ROS_WORKSPACE_SENSE_COM_H
#define ROS_WORKSPACE_SENSE_COM_H

#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <ros/ros.h>
#include <SGConnect.h>

namespace senseglove
{
    class Sensecom
    {
    public:
        Sensecom(int cycle_time);
        ~Sensecom();

        /* Delete move constructor/assignment since atomic bool cannot be moved */
        Sensecom(Sensecom&&) = delete;
        Sensecom& operator=(Sensecom&&) = delete;

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
        int start(); // 1 == success!

        /**
         * Stops the communication bus
         * TO DO: Join the thread
         */
        void stop();

    private:
        std::atomic<bool> is_operational_;
        const int cycle_time_ms_;
    };
}


#endif //ROS_WORKSPACE_SENSE_COM_H

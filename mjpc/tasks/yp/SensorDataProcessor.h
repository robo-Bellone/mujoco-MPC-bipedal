#ifndef SENSOR_DATA_PROCESSOR_H
#define SENSOR_DATA_PROCESSOR_H

#include <queue>
#include <cstddef>  // for size_t

#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {
    class SensorDataProcessor {
    private:
        std::queue<double> dataQueue;
        std::queue<std::chrono::steady_clock::time_point> timeQueue;
        
        std::chrono::steady_clock::time_point timestamp;

    public:
        int dataSteps;
        explicit SensorDataProcessor(int targetDelayMs);
        double addData(double sensorData = 0.51);

        std::chrono::milliseconds targetDelay;
    };
}  // namespace mjpc

#endif // SENSOR_DATA_PROCESSOR_H
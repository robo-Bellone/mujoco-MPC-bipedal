#include "MotorControl.h"
#include <iostream>
#include <boost/interprocess/sync/scoped_lock.hpp> // Add this line

MotorControl::MotorControl(const char* memoryName) {
    shared_memory_object::remove("ControlSharedMemory");
    shm = shared_memory_object(create_only, memoryName, read_write);
    shm.truncate(sizeof(SharedData));
    region = mapped_region(shm, read_write);
    data = new (region.get_address()) SharedData();
}

MotorControl::~MotorControl() {
    shared_memory_object::remove("ControlSharedMemory");
}

void MotorControl::sendCommands(const MotorData commands[NUM_MOTORS]) {
    scoped_lock<interprocess_mutex> lock(data->mutex);
    for (int i = 0; i < NUM_MOTORS; i++) {
        data->command[i] = commands[i];
    }
}
void MotorControl::sendPositions(double* positions) {
    scoped_lock<interprocess_mutex> lock(data->mutex);
    for (int i = 0; i < NUM_MOTORS; i++) {
        data->command[i].position = positions[i];
    }
}

void MotorControl::sendFeedbacks(const MotorData feedbacks[NUM_MOTORS]) {
    scoped_lock<interprocess_mutex> lock(data->mutex);
    for (int i = 0; i < NUM_MOTORS; i++) {
        data->feedback[i] = feedbacks[i];
    }
}

void MotorControl::readData() {
    scoped_lock<interprocess_mutex> lock(data->mutex);
    // for (int i = 0; i < NUM_MOTORS; i++) {
    //     std::cout << "Motor " << i << " Command - Torque: " << data->command[i].torque 
    //               << ", Speed: " << data->command[i].speed << ", Position: " << data->command[i].position << std::endl;
    //     std::cout << "Motor " << i << " Feedback - Position: " << data->feedback[i].position 
    //               << ", Speed: " << data->feedback[i].speed << ", Torque: " << data->feedback[i].torque << std::endl;
    // }
}

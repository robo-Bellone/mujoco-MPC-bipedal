#include "MotorControl.h"
#include <iostream>
#include <boost/interprocess/sync/scoped_lock.hpp> // Add this line
#include <unistd.h>
#include <stdlib.h>


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

void MotorControl::sendFeedbacks(const MotorData feedbacks[NUM_MOTORS]) {
    scoped_lock<interprocess_mutex> lock(data->mutex);
    for (int i = 0; i < NUM_MOTORS; i++) {
        data->feedback[i] = feedbacks[i];
    }

}

void MotorControl::readData() {
    scoped_lock<interprocess_mutex> lock(data->mutex);
   
    for (int i = 0; i < NUM_MOTORS; i++) {
        std::cout << "Motor " << i << " Command - Torque: " << data->command[i].torque 
                  << ", Speed: " << data->command[i].speed << ", Position: " << data->command[i].position << std::endl;
        std::cout << "Motor " << i << " Feedback - Torque: " << data->feedback[i].torque
                  << ", Speed: " << data->feedback[i].speed << ", Position : " << data->feedback[i].position<< std::endl;
    }
}

int main() {
    const char* test = "ControlSharedMemory";

    MotorControl control(test);

    MotorData commands[NUM_MOTORS];
    MotorData feedbacks[NUM_MOTORS];
    while(1) {
    for (int k = 0; k < 10; k++){
    for (int i = 0; i < NUM_MOTORS; ++i) {
        double j = i +1 ;
        commands[i] = { 0.0,  0.0,  0.05*k};
      //  feedbacks[i] = {j * 2.0, j * 20.0, j * 200.0};
        control.sendCommands(commands);
        }
        control.readData();
        usleep(200000);
    }

    for (int k = 10; k >= 0; k--){
    for (int i = 0; i < NUM_MOTORS; ++i) {
        double j = i +1 ;
        commands[i] = { 0.0,  0.0,  0.05*k};
      //   feedbacks[i] = {j * 2.0, j * 20.0, j * 200.0};
        control.sendCommands(commands);
        }
        control.readData();
        usleep(200000);
    }
    }

    // control.sendCommands(commands);
    // control.sendFeedbacks(feedbacks);

    // while (1) {
    // control.readData();
    // // sleep(0.5);
    // }
    // return 0;
}

//clang++ -o MotorControl MotorControl.cc -pthread -lrt
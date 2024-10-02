#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

using namespace boost::interprocess;

const int NUM_MOTORS = 8;

struct MotorData {
    double torque = 0.0;
    double speed = 0.0;
    double position = 0.0;
};

struct SharedData {
    MotorData command[NUM_MOTORS];
    MotorData feedback[NUM_MOTORS];
    interprocess_mutex mutex;
};

class MotorControl {
private:
    shared_memory_object shm;
    mapped_region region;
    SharedData *data;

public:
    MotorControl(const char* memoryName);
    ~MotorControl();
    void sendCommands(const MotorData commands[NUM_MOTORS]);
    void sendFeedbacks(const MotorData feedbacks[NUM_MOTORS]);
    void readData();
};

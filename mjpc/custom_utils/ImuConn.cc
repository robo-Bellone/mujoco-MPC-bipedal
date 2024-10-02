#include "ImuConn.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <errno.h>
#include <sstream>
#include <vector>
#include <cmath>

#include <stdexcept> // std::runtime_error, std::invalid_argument

///  센터 /dev/ttyUSB0 
/// x_vel, y_vel, z_vel 은 센터의 선속도
/// 앞이 y, 오른쪽이 x(로봇시점 기준)
///  왼발 /dev/ttyUSB1
///  오른발 /dev/ttyUSB2

using namespace std;

// IMU 데이터 초기값 설정
float center_x_pos = 0.0, center_y_pos = 0.0, center_z_pos = 0.0, center_roll = 0.0, center_pitch = 0.0, center_yaw = 0.0, center_x_loc = 0.0, center_y_loc = 0.0, center_z_loc = 0.0;
float left_x_pos = 0.0, left_y_pos = 0.0, left_z_pos = 0.0, left_roll = 0.0, left_pitch = 0.0, left_yaw = 0.0, left_x_loc = 0.0, left_y_loc = 0.0, left_z_loc = 0.0;
float right_x_pos = 0.0, right_y_pos = 0.0, right_z_pos = 0.0, right_roll = 0.0, right_pitch = 0.0, right_yaw = 0.0, right_x_loc = 0.0, right_y_loc = 0.0, right_z_loc = 0.0;

//float roll_m = 0.0; //x속도
//float pitch_m = 0.0; //y속도
//float yaw_m = 0.0;   //z속도


float x_vel = 0.0; //x 속도
float y_vel = 0.0; //y 속도
float z_vel = 0.0; //z 속도


float prev_center_z_loc = 0.0;

float diff_z = 0;

float l_a_h = 0; // 왼발높이
float r_a_h = 0; // 오른발 높이 

const float heel = 49.5;
const float toe = 85.5;

atomic<bool> exitFlag(false); // 스레드 종료 플래그

// 압력 센서 데이터 초기값 설정
int left_f = 0, left_b = 0, right_f = 0, right_b = 0;
bool left_f_status = false, left_b_status = false, right_f_status = false, right_b_status = false;


const int THRESHOLD_1 = 200; //left_f
const int THRESHOLD_2 = 200; //left_b
const int THRESHOLD_3 = 200; //right_f
const int THRESHOLD_4 = 200; //right_b 

int imu_counter = 0;
int force_counter = 0;

//*************************************************************************************************************************************
// 각종 함수

// 센터 z값 순간 차이
float localPredict(float center_z_loc, float& prev_center_z_loc, float& diff_z) {
    diff_z = center_z_loc - prev_center_z_loc;
    prev_center_z_loc = center_z_loc; // 새로운 값으로 이전 값 업데이트
    return diff_z; // 차이 값 반환
}

// 왼 발목 높이
float left_ankle_hi(float left_x_pos, const float heel, const float toe, float& l_a_h, bool left_f_status, bool left_b_status) {
    float theta;
    float radian_theta;

    if (-5 < left_x_pos && left_x_pos < 5) {
        l_a_h = 50;
    } else if (left_x_pos <= -5) {
        theta = -left_x_pos;
        radian_theta = theta * (M_PI / 180.0); // 각도를 라디안으로 변환
        l_a_h = 33.5 / cos(radian_theta) + (toe - 15 + 15 / tan(radian_theta) - 33.5 * tan(radian_theta)) * sin(radian_theta);
    } else {
        theta = left_x_pos;
        radian_theta = theta * (M_PI / 180.0); // 각도를 라디안으로 변환
        l_a_h = 33.5 / cos(radian_theta) + (heel - 10 + 15 / tan(radian_theta) - 33.5 * tan(radian_theta)) * sin(radian_theta);
    }
    l_a_h -= 4.8181;
    l_a_h *= 0.001;
    return l_a_h;
}

// 오른 발목 높이
float right_ankle_hi(float right_x_pos, const float heel, const float toe, float& r_a_h, bool right_f_status, bool right_b_status) {
    float theta;
    float radian_theta;

    if (-5 < right_x_pos && right_x_pos < 5) {
        r_a_h = 50;
    } else if (right_x_pos <= -5) {
        theta = -right_x_pos;
        radian_theta = theta * (M_PI / 180.0); // 각도를 라디안으로 변환
        r_a_h = 33.5 / cos(radian_theta) + (toe - 15 + 15 / tan(radian_theta) - 33.5 * tan(radian_theta)) * sin(radian_theta);
    } else {
        theta = right_x_pos;
        radian_theta = theta * (M_PI / 180.0); // 각도를 라디안으로 변환
        r_a_h = 33.5 / cos(radian_theta) + (heel - 10 + 15 / tan(radian_theta) - 33.5 * tan(radian_theta)) * sin(radian_theta);
    }
    r_a_h -= 4.8181;
    r_a_h *= 0.001;
    return r_a_h;
}


/*
// 보정 로직을 포함한 함수
void smoothSensorData(float &roll, float &pitch, float &yaw, float &roll_m, float &pitch_m, float &yaw_m, float step = 0.05f, float dif = 0.1f) {
    if (fabs(roll) == 0.0 && fabs(roll_m) > dif) {
        roll_m = (roll_m > 0) ? roll_m - step : roll_m + step;
    } else {
        roll_m = roll;
    }

    if (fabs(pitch) == 0.0 && fabs(pitch_m) > dif) {
        pitch_m = (pitch_m > 0) ? pitch_m - step : pitch_m + step;
    } else {
        pitch_m = pitch;
    }

    if (fabs(yaw) == 0.0 && fabs(yaw_m) > dif) {
        yaw_m = (yaw_m > 0) ? yaw_m - step : yaw_m + step;
    } else {
        yaw_m = yaw;
    }
}
*/
void smoothSensorData(float &center_x_loc, float &center_y_loc, float &center_z_loc, float &x_vel, float &y_vel, float &z_vel, float step = 0.05f, float dif = 0.2f) {
    if (fabs(center_x_loc) == 0.0 && fabs(x_vel) > dif) {
        x_vel = (x_vel > 0) ? x_vel - step : x_vel + step;
    } else {
        x_vel = center_x_loc;
    }

    if (fabs(center_y_loc) == 0.0 && fabs(y_vel) > dif) {
        y_vel = (y_vel > 0) ? y_vel - step : y_vel + step;
    } else {
        y_vel = center_y_loc;
    }

    if (fabs(center_z_loc) == 0.0 && fabs(z_vel) > dif) {
        z_vel = (z_vel > 0) ? z_vel - step : z_vel + step;
    } else {
        z_vel = center_z_loc;
    }
}


// 압력 센서 컨택 함수
void checkSensorThreshold() {
    left_f_status = (left_f > THRESHOLD_1);
    left_b_status = (left_b > THRESHOLD_2);
    right_f_status = (right_f > THRESHOLD_3);
    right_b_status = (right_b > THRESHOLD_4);
}

// IMU 데이터를 처리하는 함수
void processIMUData(const char* portName, float& xpos, float& ypos, float& zpos, 
                    float& roll, float& pitch, float& yaw,
                    float& xloc, float& yloc, float& zloc) {
    int serial_port = open(portName, O_RDWR);
    if (serial_port < 0) {
        cerr << "Failed to open serial port: " << strerror(errno) << std::endl;
        exitFlag = true;
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(serial_port, &tty) != 0) {
        cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        exitFlag = true;
        return;
    }

    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    tty.c_cflag |= (CS8 | CREAD | CLOCAL);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        exitFlag = true;
        return;
    }

    char buffer[256];
    string line;

    while (!exitFlag) {
        ssize_t bytes_read = read(serial_port, buffer, sizeof(buffer));
        if (bytes_read < 0) {
            cerr << "Error reading: " << strerror(errno) << std::endl;
            exitFlag = true;
            break;
        }

        line.append(buffer, bytes_read);
        size_t star_pos = line.find('*');
        size_t newline_pos = line.find('\n', star_pos);
        if (newline_pos != string::npos) {
            string sensor_data = line.substr(star_pos + 1, newline_pos - star_pos - 1);
            istringstream ss(sensor_data);
            string token;
            vector<float> values;

            while (getline(ss, token, ',')) {
                try {
                    values.push_back(stof(token));
                } catch (const invalid_argument& e) {
                    cerr << "Invalid argument: " << token << std::endl;
                } catch (const out_of_range& e) {
                    cerr << "Out of range: " << token << std::endl;
                }
            }

            if (values.size() == 9) {
                xpos = values[0];
                ypos = values[1];
                zpos = values[2]; 
                roll = values[3];
                pitch = values[4];
                yaw = values[5];
                xloc = values[6];
                yloc = values[7];
                zloc = values[8];
            }
            
            //  std::cout << "Data from " << setw(8) << portName << ": "
            //      << "x_pos=" << setw(7) << fixed << setprecision(2) << xpos << ", "
            //      << "y_pos=" << setw(7) << fixed << setprecision(2) << ypos << ", "
            //      << "z_pos=" << setw(7) << fixed << setprecision(2) << zpos << ", "
            //      << "roll="  << setw(6) << fixed << setprecision(2) << roll << ", "
            //     << "pitch=" << setw(6) << fixed << setprecision(2) << pitch << ", "
            //      << "yaw="   << setw(6) << fixed << setprecision(2) << yaw << ", "
            //      << "x_loc=" << setw(8) << fixed << setprecision(2) << xloc << ", "
            //      << "y_loc=" << setw(8) << fixed << setprecision(2) << yloc << ", "
            //      << "z_loc=" << setw(8) << fixed << setprecision(2) << zloc << std::endl;

            

            if (string(portName) == "/dev/ttyUSB1") {
                left_ankle_hi(xpos, heel, toe, l_a_h, left_f_status, left_b_status);
                // std::cout << "left_ankle_hi: " << l_a_h << std::endl;
            }    
            if (string(portName) == "/dev/ttyUSB2") {
                right_ankle_hi(xpos, heel, toe, r_a_h, right_f_status, right_b_status);
                // std::cout << "right_ankle_hi: " << r_a_h << std::endl;
            }  

            if (string(portName) == "/dev/ttyUSB0") {
                localPredict(zloc, prev_center_z_loc, diff_z);
                // std::cout << "diff_z: " << diff_z << std::endl;


                smoothSensorData(roll, pitch, yaw, x_vel, y_vel, z_vel);
                // std::cout << "x_vel: " << x_vel << ", y_vel: " << y_vel << ", z_vel: " << z_vel << std::endl;
            }
            
            
            
            line.erase(0, newline_pos + 1);
        }
        // imu_counter++;
        // std::cout << "imu_counter: " << imu_counter << std::endl;
    }

    close(serial_port);
}

// 압력 센서 데이터를 처리하는 함수
void processPressureData() {
    int serial_port = open("/dev/ttyACM0", O_RDWR);

    if (serial_port < 0) {
        cerr << "Failed to open serial port: " << strerror(errno) << std::endl;
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | INPCK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        return;
    }

    std::string line;
    char buffer[128];
    size_t max_length = 256; // 최대 길이 설정
    size_t temp_npos = 0;
    while (!exitFlag) {
        ssize_t bytes_read = read(serial_port, buffer, sizeof(buffer));

        if (bytes_read < 0) {
            std::cerr << "Error reading: " << strerror(errno) << std::endl;
            break;
        }
        line.append(buffer, bytes_read);

        if (line.length() > max_length) {
            // 오래된 데이터 제거
            line.erase(0, line.length() - max_length);
        }
        //cout << line << endl;
        size_t newline_pos = line.find('\n');
        //cout << newline_pos << endl;
        //18446744073709551615UL
        if (newline_pos != string::npos) {
            string sensor_data = line.substr(temp_npos, newline_pos+1);
            temp_npos = newline_pos;
            //std::cout << "Parsed line data: " << sensor_data << std::endl;  // Debugging output
            std::istringstream ss(sensor_data);
            std::string token;
            int sensorIndex = 1;
            while (std::getline(ss, token, ',')) {
                try {
                    int value = std::stoi(token);
                    switch (sensorIndex) {
                        case 1: left_f = value; break;
                        case 2: left_b = value; break;
                        case 3: right_f = value; break;
                        case 4: right_b = value; break;
                    }
                    //std::cout << "Sensor" << sensorIndex << ": " << value << std::endl;
                    sensorIndex++;
                } catch (const exception& e) {
                    //std::cerr << "Error parsing '" << token << "': " << e.what() << std::endl;
                }
            }
            // std::cout << std::endl;

            // std::cout << "left_f: " << left_f << std::endl;
            // std::cout << "left_b: " << left_b << std::endl;
            // std::cout << "right_f: " << right_f << std::endl;
            // std::cout << "right_b: " << right_b << std::endl;
            // std::cout << std::endl;

            checkSensorThreshold();

            // std::cout << "left_f status: " << (left_f_status ? "True" : "False") << std::endl;
            // std::cout << "left_b status: " << (left_b_status ? "True" : "False") << std::endl;
            // std::cout << "right_f status: " << (right_f_status ? "True" : "False") << std::endl;
            // std::cout << "right_b status: " << (right_b_status ? "True" : "False") << std::endl;
            // std::cout << std::endl;

            //force_counter++;
            //std::cout << "force_counter: " << force_counter << std::endl;
            // std::cout << std::endl;

            // line.erase(0, newline_pos + 1);
            line.erase(0, newline_pos + 1);
        }
    }

    close(serial_port);
}
#include <fstream>
#include <iostream>
#include <unistd.h>

int main() {
    std::string pipe1 = "/tmp/pipe1";
    std::string pipe2 = "/tmp/pipe2";

    int writes = 10;
    std::ofstream write_pipe(pipe1, std::ios::out);
    std::ifstream read_pipe(pipe2, std::ios::in);
    
    std::cout << "Opened pipes" << std::endl;
    while (writes--) {
        write_pipe << 1234 << std::endl;
        printf("sent message\n");

        printf("Reading response...\n");
        std::string response;
        std::getline(read_pipe, response);
        printf("Received response: %s\n", response.c_str());
    }
    write_pipe.close();
    read_pipe.close();

    return 0;
}
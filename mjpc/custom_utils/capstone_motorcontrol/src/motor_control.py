import posix_ipc
import mmap
import struct
import time

class MotorControl:
    NUM_MOTORS = 8

    def __init__(self, memory_name):
        self.memory = posix_ipc.SharedMemory(memory_name)
        self.map_file = mmap.mmap(self.memory.fd, self.memory.size)
        self.memory.close_fd()
        self.data_format = '24d'  # 8 motors * 3 values per motor * 2 (command + feedback)
        try:
            self.semaphore = posix_ipc.Semaphore("shm_mutex")
            self.semaphore.release()
        except posix_ipc.ExistentialError:
            print("Semaphore does not exist. Creating a new one.")
            self.semaphore = posix_ipc.Semaphore("shm_mutex", flags=posix_ipc.O_CREAT)
            self.semaphore.release()

    def __del__(self):
        self.semaphore.close()
        self.map_file.close()
        posix_ipc.unlink_shared_memory("ControlSharedMemory")

    def send_feedbacks(self, feedbacks):
        self.semaphore.acquire()
        try:
            struct.pack_into(self.data_format, self.map_file, 192, *(val for fb in feedbacks for val in fb))
            print('send feedback')
        finally:
            self.semaphore.release()

    def read_commands(self):
        self.semaphore.acquire()
        try:
            data = struct.unpack_from(self.data_format, self.map_file, 0)
        finally:
            self.semaphore.release()
        return data


if __name__ == "__main__":
    mc = MotorControl("ControlSharedMemory")
    
    while True:
        commands = mc.read_commands()
        print("Received Commands:", commands)
        
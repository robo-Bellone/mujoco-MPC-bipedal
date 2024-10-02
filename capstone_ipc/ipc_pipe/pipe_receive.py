import errno
import os

pipe1 = '/tmp/pipe1'
pipe2 = '/tmp/pipe2'
try:
    os.mkfifo(pipe1, mode=0o777)

except OSError as oe:
    if oe.errno != errno.EEXIST:
        raise

try:
    os.mkfifo(pipe2, mode=0o777)
except OSError as oe:
    if oe.errno != errno.EEXIST:
        raise


# blocks here until writer opens read_pipe
read_pipe = open(pipe1, 'r')
write_pipe = open(pipe2, 'w')
print('opened pipes')

count = 10
while count:
    r = read_pipe.readline().strip('\n')
    while not r :
        r = read_pipe.readline().strip('\n')
        if r :
            print(f'received {r}')
            break
    
    write_pipe.write(f"Response: 5678\n")
    write_pipe.flush()
    print("wrote response")
    count-=1
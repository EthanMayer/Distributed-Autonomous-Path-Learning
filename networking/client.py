import socket
from .commands import *
from .utils import *
import time
import io
import array

class Client:
    def __init__(self):
        # create an INET, STREAMing socket
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # now connect to the web server on port 8080 - the normal http port
        ip = input("Enter IP to connect to (port 8080 will be used): ")
        self.s.connect((ip, 8080))
    
    def sendPath(self, path):
        msg = makeRobotPathCommand(path, NetworkCommand.uploadPath)
        self.s.send(msg)

    def receivePath(self):
        RECV_BUFSIZE = 4096  # maximum amount of data to be received at a time
        while True:
            data = self.s.recv(RECV_BUFSIZE)
            def handler():
                data = self.s.recv(RECV_BUFSIZE) # Change the outer `data`
                if len(data) == 0:
                    time.sleep(0.4) # Avoid burning CPU
                return data
            return runCommand(data, self._do_stuff, handler)
    
    def _do_stuff(self, dataRest, command):
        if command == NetworkCommand.setPath:
            # Upload to server needs to be handled.
            pass
        else:
            print("Error: the command", command, "cannot be issued to a client")
            return
        
        # Receive the path
        self.path = unpack_array('B', dataRest)
        return self.path
    
    def close(self):
        self.s.shutdown(socket.SHUT_RDWR)
        self.s.close()
    
    # "Destructor": called upon garbage collection of this object.
    def __del__(self):
        self.close()

def main():
    c = Client()
    time.sleep(2)
    c.sendPath([90,1,2,3])
    p = c.receivePath()
    print(p)

if __name__ == "__main__":
    main()

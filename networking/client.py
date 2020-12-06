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
        print("Connected")
    
    def sendPath(self, path):
        msg = makeRobotPathCommand(path, NetworkCommand.uploadPath)
        self.s.send(msg)
        print("Sent path:",path)

    def receivePath(self):
        # Send getLastPath command
        with io.BytesIO() as output:
            # Write command
            outputCommand(NetworkCommand.getLastPath, output)

            self.s.send(output.getvalue())

        # Receive path
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
    #time.sleep(2)
    recvOrSend = input("Receive (1) or send (0)?: ")
    if recvOrSend == "0":
        # Send path
        #p1 = [90,1,2,3]
        p2 = [100, 80, 180, 0, 160, 100, 160, 130]
        c.sendPath(p2)
    else:
        # Receive path
        p = c.receivePath()
        print(p)

if __name__ == "__main__":
    main()

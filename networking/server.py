#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import signal
import socket
import threading
import commands
import io
import pickle

# Sources for some of this code:
# https://docs.python.org/3/howto/sockets.html
# https://gist.github.com/dp7k/a496ef8e8a32f713900fd5fe1e12d41a
# https://docs.python.org/3/library/socket.html#socket.socket.listen

defaultSeparator = ' '

# Returns a memoryview of the data for the item and the length that was read.
# For documentation on memoryview, see https://docs.python.org/3.3/library/stdtypes.html#memoryview
# and https://docs.python.org/3.3/library/struct.html#format-characters
def readSeparatedData(data, ending=defaultSeparator):
    length = 0
    while True:
        if data[length] == ending:
            break
        length += 1
    if length == 0:
        raise Exception("Invalid length")
    return (memoryview(data)[:length], length)

def readAsciiInt(data, ending=defaultSeparator):
    rest, length = readSeparatedData(data, ending)
    return int(str(rest)), rest, length

def readUInt8(data, ending=defaultSeparator):
    rest, length = readSeparatedData(data, ending)
    return rest.cast('B'), rest, length # 'B' is for unsigned char which is size 1; see
    # https://docs.python.org/3.3/library/struct.html#format-characters for more info. 

# Returns the int up until a space, then the rest of the data, 
# and then the length of the rest of the data
def readASCIIIntAndSpace(data):
    length = 0
    while True:
        if data[length] == ' ':
            break
        length += 1
    if length == 0:
        raise Exception("Invalid length")
    restBytesLength = int(str(data[:length]))
    return (restBytesLength, data[length:length + restBytesLength], length)

# Represents a single client connected to the server.
class ClientThread:
    def __init__(self, clientsocket, address, allClients):
        RECV_BUFSIZE = 4096  # maximum amount of data to be received at a time
        self.clientsocket = clientsocket
        self.clientsocket_str = '%s:%d' % (address[0],address[1])
        self.allClients = allClients

        # Car path that was uploaded. Currently is a list of angles.
        self.path = []

        # main loop: receive/send data from this client
        while True:
            data = self.clientsocket.recv(RECV_BUFSIZE)
            if not data:
                break
            # Read in the length specified in the packet as ASCII and convert it to an int,
            # putting the result into `itemLength`.
            itemLength, dataRest, amountRead = readUInt8(data)
            commandNumber = pickle.loads(dataRest[:itemLength])
            if not NetworkCommand.has_value(commandNumber):
                raise Exception("Invalid command number received: " + str(commandNumber))
            # We can now use `commandNumber` to run a command on the server.
            self._do_stuff(dataRest[:itemLength], commandNumber)
        self.clientsocket.close()
        print('ClientThread', self.clientsocket_str, 'disconnected')
    
    # play with received data
    def _do_stuff(self, data):
        if data.startswith(str(commands.NetworkCommand.uploadPath)):
            # Upload to server needs to be handled.
            listLen = int.from_bytes(data[1:4], "little")
            self.path = list(data[2:2+listLen]) # Convert bytearray to a list
        else:
            print("Error: unrecognized command")
        
        # setPath: send to all clients except sender

        output = io.StringIO()
        output1 = pickle.dumps(commands.NetworkCommand.setPath)
        print(str(len(output1)) + " ", file=output)
        output.write(output1)
        
        output2 = pickle.dumps(self.path)
        print(str(len(output2)) + " ", file=output)
        output.write(output2)
        #print('And so does this.', file=output)

        reply = output.getvalue() # Bytes object containing entire buffer
        #reply = ('OK: %s' % data.decode()).encode()
        # Send to all clients except sender:
        for client in allClients:
            if client != self.clientsocket:
                client.send(reply)
        
if __name__ == "__main__":
    # create an INET, STREAMing socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # now connect to the web server on port 80 - the normal http port
    #s.connect(("www.python.org", 80))

    # bind the socket to a public host, and a well-known port
    s.bind((socket.gethostname(), 8080))
    # become a server socket
    s.listen(5)

    # close socket on shutdown
    def shutdown():
        s.close()
        print('Gracefull shutdown. Bye.')
    def _shutdown_handler(signum, frame):
        shutdown()
    # set signal handler
    signal.signal(signal.SIGINT, _shutdown_handler)   # KeyboardInterrupt
    signal.signal(signal.SIGTERM, _shutdown_handler)  # kill

    # main loop to accept connections
    allClients = []
    while True:
        # accept connections from outside
        (clientsocket, address) = s.accept()
        allClients.append(clientsocket)
        
        # Spawn a thread for each client.
        thread = threading.Thread(target=ClientThread, args=(clientsocket, address, allClients))
        thread.setDaemon(True)
        thread.start()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import signal
import socket
import threading
import commands
import io
import pickle
import array

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
    return rest.cast('!B'), rest, length # 'B' is for unsigned char which is size 1 only if
    # using a standard format specified which is "one of '<', '>', '!' or '='". '<' is 
    # little endian, "!" is network byte order (big-endian also), and '>' is big endian. 
    # See
    # https://docs.python.org/3.3/library/struct.html#format-characters for more info. 

# This unpacks bytes `data` into an array of network byte order.
# Returns an array in machine
# byte order, from data which is possibly not machine byte order and is instead the
# struct format specified by `fmt` for each item in the bytes.
def unpack_array(fmt, data):
    charactersToRemove = ['<', '>', '!', '=']
    arrayFmt = fmt
    for c in charactersToRemove:
        arrayFmt = arrayFmt.replace(c, '')
    size = struct.calcsize(arrayFmt) * len(data)
    return array.array(arrayFmt).extend(struct.iter_unpack(fmt, data))

# Packs an array into a bytes object from machine byte order (always the case for
# an array.array in Python) into network byte order.
def pack_array(arr):
    #elemSize = arr.itemsize #struct.calcsize('!' + arr.typecode)
    with io.StringIO() as output:
        for elem in arr:
            output.write(struct.pack('!' + arr.typecode, elem))
        return output.getvalue()

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

        # Car path that was uploaded. Currently is an array of angles encoded as bytes.
        self.path = None

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
            self._do_stuff(dataRest[:itemLength], NetworkCommand(commandNumber))
        self.clientsocket.close()
        print('ClientThread', self.clientsocket_str, 'disconnected')
    
    # play with received data
    def _do_stuff(self, dataRest, command):
        if command == NetworkCommand.uploadPath:
            # Upload to server needs to be handled.
            listLen = int.from_bytes(data[1:4], "little")
            self.path = list(data[2:2+listLen]) # Convert bytearray to a list
        else:
            print("Error: the command", command, "cannot be issued to a server")
            return
        
        # Receive the uploadPath
        self.path = array.array('B')
        self.path.extend(struct.iter_unpack('!B', dataRest)) # iter_unpack "returns an iterator
        # which will read equally-sized chunks from the buffer until all its contents have 
        # been consumed." ( https://docs.python.org/3/library/struct.html#struct.iter_unpack )
        # array.extend: "Append items from iterable to the end of the array." [...] "If 
        # iterable is not an array, it must be iterable and its elements must be the 
        # right type to be appended to the array."
        # ( https://docs.python.org/3/library/array.html#array.array.extend )

        # We have received uploadPath so now we send a bunch of setPath's:
        # Send the path to all clients except the sender.

        with io.StringIO() as output:
            commandBytes = pickle.dumps(commands.NetworkCommand.setPath)
            print(str(len(commandBytes)) + " ", file=output)
            output.write(commandBytes)
            
            pathBytes = struct.pack('!s', self.path.tobytes())
            print(str(len(pathBytes)) + " ", file=output)
            output.write(pathBytes)

            replyBytes = output.getvalue() # Bytes object containing entire buffer
        
        #reply = ('OK: %s' % data.decode()).encode()
        # Send to all clients except sender:
        for client in allClients:
            if client != self.clientsocket:
                client.send(replyBytes)
        
        def makeUploadPath():
            output = io.StringIO()
            commandBytes = pickle.dumps(commands.NetworkCommand.uploadPath)
            output.write(commandBytes.)


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
        print('Shutting down')
        s.close()
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
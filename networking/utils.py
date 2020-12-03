import struct
import array
import io
from .commands import *
import time

# Separator for use within buffers sent over the network.
defaultSeparator = ' '

# Indicates that more data is needed to continue reading the requested item,
# but it couldn't be read.
class UnfinishedLengthException(Exception):
    pass

# Returns a memoryview of the data for the item, another memoryview for the rest
# of the data remaining, and the length that was read.
# For documentation on memoryview, see https://docs.python.org/3.3/library/stdtypes.html#memoryview
# and https://docs.python.org/3.3/library/struct.html#format-characters
# @param handleUnfinishedLength -- handler for when we need more bytes for this to be a
# complete input. When this is called, return a new bytes object that has the next data.
def readSeparatedData(data, handleUnfinishedLength, ending=defaultSeparator):
    length = 0
    alreadyPrintedZeroLengthMessage = False
    print("Data:", str(bytes(data)))
    while True:
        try:
            while True:
                if data[length] == ord(ending):
                    print("Received:", bytes(data[:length]))
                    break
                length += 1
                alreadyPrintedZeroLengthMessage = False
            if length == 0:
                raise Exception("Invalid zero-length")
        except IndexError:
            # Need to get more bytes
            if len(data) != 0:
                print("readSeparatedData(): Need more bytes: have", len(data), "byte(s) so far")
            elif len(data) == 0 and not alreadyPrintedZeroLengthMessage:
                # Print a message that we don't have any more data
                # if we didn't already show a zero length message
                print("readSeparatedData(): No more data, waiting for more...")
                alreadyPrintedZeroLengthMessage = True
            
            #time.sleep(0.4)
            res = handleUnfinishedLength()
            if res == False:
                raise UnfinishedLengthException()
            else:
                data = bytes(data) + bytes(res) # Combine bytes objects (concatenate)
                continue # Try reading from `data` again.
        return (memoryview(data)[:length], memoryview(data)[length+1:], length)
        #                                                        ^^^ +1 to move past the
        # `ending` separator.

def ensureLength(data, requiredLength, handleUnfinishedLength):
    while len(data) < requiredLength:
        res = handleUnfinishedLength()
        if res == False:
            raise UnfinishedLengthException()
        else:
            data = data + res # Combine bytes objects (concatenate)
            continue # Try checking `data` again.

def readAsciiInt(data, handleUnfinishedLength, ending=defaultSeparator):
    read, rest, length = readSeparatedData(data, handleUnfinishedLength, ending)
    print("Reading ASCII int:", bytes(read))
    return int(read), rest, length

def readUInt8(data, handleUnfinishedLength, ending=defaultSeparator):
    read, rest, length = readSeparatedData(data, handleUnfinishedLength, ending)
    return read.cast('>B'), rest, length # 'B' is for unsigned char which is size 1 only if
    # using a standard format specified which is "one of '<', '>', '!' or '='". '<' is 
    # little endian, "!" is network byte order (big-endian also), and '>' is big endian. 
    # See
    # https://docs.python.org/3.3/library/struct.html#format-characters for more info. 

# This unpacks bytes `data` into an array.
# Returns an array in machine
# byte order, from data which is possibly not machine byte order and is instead the
# struct format specified by `fmt` for each item in the bytes.
def unpack_array(fmt, data):
    charactersToRemove = ['<', '>', '!', '=']
    arrayFmt = fmt
    for c in charactersToRemove:
        arrayFmt = arrayFmt.replace(c, '')
    #size = struct.calcsize(arrayFmt) * len(data)
    #print(type(arrayFmt), arrayFmt)
    #print(type(fmt), fmt)
    #print(type(data), data)
    # iter_unpack returns an iterator that returns tuples for each formatting letter
    # in `fmt` so we need to place these into the array properly. The array can hold
    # one type at a time, so we will ignore the rest (and report an error if the rest
    # exists). We `map` a function over the results from struct.iter_unpack to ensure
    # they work as single values instead of as tuples.
    def unpackHandler(tuple):
        if len(tuple) == 1:
            return tuple[0]
        else:
            raise Exception("An array.array can't have multiple types in the format string")
    return array.array(arrayFmt, map(unpackHandler, struct.iter_unpack(fmt, data)))
    # iter_unpack "returns an iterator
    # which will read equally-sized chunks from the buffer until all its contents have 
    # been consumed." ( https://docs.python.org/3/library/struct.html#struct.iter_unpack )
    # array.extend: "Append items from iterable to the end of the array." [...] "If 
    # iterable is not an array, it must be iterable and its elements must be the 
    # right type to be appended to the array."
    # ( https://docs.python.org/3/library/array.html#array.array.extend )

# Packs an array into a bytes object from machine byte order (always the case for
# an array.array in Python) into big-endian byte order.
def pack_array(arr):
    #elemSize = arr.itemsize #struct.calcsize('!' + arr.typecode)
    with io.BytesIO() as output:
        for elem in arr:
            output.write(struct.pack('>' + arr.typecode, elem))
        return output.getvalue()

# Robot commands #

import commands

# @param path -- array.array
def outputRobotPath(path, outputIO):
    pathBytes = pack_array(path)
    # Write length of bytes
    outputIO.write(bytes(str(len(pathBytes)) + " ", encoding='utf-8'))
    # Write bytes
    outputIO.write(pathBytes)

# @param path -- list
def makeRobotPathCommand(path, command):
    replyBytes = None
    with io.BytesIO() as output:
        commandBytes = str(command.value)
        # Write bytes
        output.write(bytes(commandBytes + ' ', encoding='utf-8'))

        # Write path
        outputRobotPath(array.array('B', path), output)

        replyBytes = output.getvalue() # Bytes object containing entire buffer
    #print(type(replyBytes))
    return replyBytes

def runCommand(data, callback, handler):
    # Read in the length specified in the packet as ASCII and convert it to an int,
    # putting the result into `itemLength`.
    commandNumber, dataRest, amountRead = readAsciiInt(data, handler)
    if not NetworkCommand.has_value(commandNumber):
        raise Exception("Invalid command number received: " + str(commandNumber))
    # Get the length of the path:
    pathLength, dataRest, amountRead = readAsciiInt(dataRest, handler)
    ensureLength(dataRest, pathLength, handler)
    # We can now use `commandNumber` to run a command on the server.
    return callback(dataRest[:pathLength], NetworkCommand(commandNumber))
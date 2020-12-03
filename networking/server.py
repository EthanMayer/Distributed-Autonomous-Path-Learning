#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import signal
import socket
import threading
from commands import *
import io
import pickle
import array
import struct
from utils import *

# Sources for some of this code:
# https://docs.python.org/3/howto/sockets.html
# https://gist.github.com/dp7k/a496ef8e8a32f713900fd5fe1e12d41a
# https://docs.python.org/3/library/socket.html#socket.socket.listen

# Represents a single client connected to the server.
class ClientThread:
    def __init__(self, clientsocket, address, allClients):
        try:
            RECV_BUFSIZE = 4096  # maximum amount of data to be received at a time
            self.clientsocket = clientsocket
            self.clientsocket_str = '%s:%d' % (address[0],address[1])
            self.allClients = allClients

            # Car path that was uploaded. Currently is an array of angles encoded as bytes.
            self.path = None

            # main loop: receive/send data from this client
            while True:
                data = self.clientsocket.recv(RECV_BUFSIZE)
                def handler():
                    data = self.clientsocket.recv(RECV_BUFSIZE) # Change the outer `data`
                    return data
                runCommand(data, self._do_stuff, handler)
        finally:
            self.clientsocket.close()
            print('ClientThread', self.clientsocket_str, 'disconnected')
    
    # play with received data
    def _do_stuff(self, dataRest, command):
        if command == NetworkCommand.uploadPath:
            # Upload to server needs to be handled.
            pass
        else:
            print("Error: the command", command, "cannot be issued to a server")
            return
        
        # Receive the uploadPath
        self.path = unpack_array('>B', dataRest)

        # We have received uploadPath so now we send a bunch of setPath's:
        # Send the path to all clients except the sender.

        replyBytes = makeRobotPathCommand(self.path, NetworkCommand.setPath)
        
        #reply = ('OK: %s' % data.decode()).encode()
        # Send to all clients except sender:
        for client in self.allClients:
            if client != self.clientsocket:
                print("sendPath:", replyBytes)
                client.send(replyBytes)
        

if __name__ == "__main__":
    s = None
    try:
        # create an INET, STREAMing socket
        #with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # now connect to the web server on port 80 - the normal http port
        #s.connect(("www.python.org", 80))

        # bind the socket to a public host, and a well-known port
        s.bind((socket.gethostname(), 8080))
        # become a server socket
        s.listen(5)

        # main loop to accept connections
        allClients = []
        while True:
            print("Waiting for connections...")

            # accept connections from outside
            (clientsocket, address) = s.accept()
            allClients.append(clientsocket)
            
            # Spawn a thread for each client.
            thread = threading.Thread(target=ClientThread, args=(clientsocket, address, allClients))
            thread.setDaemon(True)
            thread.start()
    finally:
        s.shutdown(socket.SHUT_RDWR)
        s.close()
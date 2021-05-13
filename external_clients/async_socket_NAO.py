import socket
import errno
import sys, os
import time
from time import sleep
import json

class WriteNAOSocket(asyncio.DatagramProtocol):
    def __init__(self, robot_number, ip, port):
        super().__init__()

        self.robot_number = robot_number
        self.ip = ip
        self.port = port

        self.alive = False
        self.last_message_timestamp = time.time()
        self.transport = None

    def connection_made(self, transport):
        self.transport = transport

    def error_received(self, exc):
        print('Error received:', exc)

class ReadNAOSocket(asyncio.DatagramProtocol):
    def __init__(self, robot_number, ip, port, writeNAOSocket, GUIWebSocket):
        super().__init__()

        self.robot_number = robot_number
        self.ip = ip
        self.port = port

        self.alive = False
        self.last_message_timestamp = time.time()
        self.transport = None

        self.writeNAOSocket = writeNAOSocket
        self.GUIWebSocket = GUIWebSocket

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        # Here is where you would push message to whatever methods/classes you want.
        print(f"Received Syslog message: {data}")

    def error_received(self, exc):
        print('Error received:', exc)


def setup_read_sockets(ROBOT_LIST, UDP_IP, UDP_PORTS_READ, READ_TIMEOUT):
    #Setup read sockets
    read_sockets = {}
    for i, robot_read_port in enumerate(UDP_PORTS_READ):
        print("Robot %d: UDP read port: %s" % (ROBOT_LIST[i], robot_read_port))
        
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

        addr = socket.getaddrinfo(UDP_IP, robot_read_port, socket.AF_INET, socket.SOCK_DGRAM)[0]

        sock.bind((UDP_IP, robot_read_port))
        sock.setblocking(0)
        sock.settimeout(READ_TIMEOUT)
        
        read_sockets[ROBOT_LIST[i]] = {"robot_number" : ROBOT_LIST[i], "sock": sock, "address": addr, "alive" : False, "last_message_timestamp": time.time()}  # UDP
        
    return read_sockets

def setup_write_sockets(ROBOT_LIST, UDP_IP, UDP_PORTS_WRITE):
    #Setup read sockets
    write_sockets = {}
    for i, (robot_write_port, robot_dest_port) in enumerate(zip(UDP_PORTS_WRITE, UDP_PORTS_DEST)):
        print("Robot %d: UDP write port: %s" % (ROBOT_LIST[i], robot_write_port))
        
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

        addr = socket.getaddrinfo(UDP_IP, robot_dest_port, socket.AF_INET, socket.SOCK_DGRAM)[0]

        sock.bind((UDP_IP, robot_write_port))
        sock.setblocking(0)
        #sock.settimeout(SOCKET_TIMEOUT)
        
        write_sockets[ROBOT_LIST[i]] = {"robot_number" : ROBOT_LIST[i], "sock": sock, "address": addr, "alive" : False}  # UDP
    
    return write_sockets


def send_keepalive_request(robot_number):
    write_sock = write_sockets[robot_number]["sock"]
    write_sock.sendto(b"uthere?\x00", write_sockets[robot_number]["address"][4])

def send_keepalive_response(robot_number):
    write_sock = write_sockets[robot_number]["sock"]
    write_sock.sendto(b"yeah\x00", write_sockets[robot_number]["address"][4])
    print("KEEPALIVE response sent %s" % b"yeah\x00")

def handleMessage(data, robot_number):

    #Set robot as "alive" because we received data from it
    read_sockets[robot_number]["llast_message_timestamp"] = time.time()
    read_sockets[robot_number]["alive"] = True
    write_sockets[robot_number]["alive"] = True

    #Decode data into a string (might be improved later)
    data = data.decode('utf-8')

    print("Received message from robot %d: %s" % (robot_number, data))

    #keep-alive message
    if(data == "uthere?"):
        print("KEEPALIVE received from robot %s" % robot_number)
        send_keepalive_response(robot_number)
        
    #obs = data.split("|")
    #if(float(obs[4]) < 0):
    #    MESSAGE = b"1"
    #elif(abs(float(obs[5])) > 400):
    #    MESSAGE = b"2"
    #elif(float(obs[5]) > 0):
    #    MESSAGE = b"4"
    #elif (float(obs[5]) <= 0):
    #    MESSAGE = b"3"
    #sock.sendto(MESSAGE, (UDP_IP, UDP_PORT_WRITE))


def receive_data_from_socket(robot_number):
    read_sock = read_sockets[robot_number]
    data, addr = None, None
    try:
        #read_sock["sock"].connect(read_sock["address"][4]) #Not really needed for UDP but should solve some resolving issues in the local API
        data, addr = read_sock["sock"].recvfrom(1024)
    except socket.timeout:
        #SOCKET TIMEOUT (or something similar)
        #print('No data received from robot: %s (Timed out)' % robot_number)
        return None, None
    except socket.error as e:
        err = e.args[0]
        if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
            #SOCKET FULL (or something similar)
            sleep(1)
            print('No data received from robot: %s (Socket buffer full)' % robot_number)
            return None
        else:
            #ERROR
            print("GENERIC ERROR: ",e)
            sys.exit(1)

    return data, addr

if __name__ == "__main__":

    UDP_IP = "127.0.0.1"
    #UDP_IP = "localhost"

    #Notice: 
    # - the DEST_PORTs here have to be the READ_PORTs on the robots
    # - the READ_PORTs here have to be the DEST_PORTs on the robots
    # - the WRITE_PORTs here have to be different from those of the robots
    UDP_BASE_READ_PORT = 65200 #Listening port for incoming messages
    UDP_BASE_WRITE_PORT = 65100  #Socket port of outgoing messages for each robot
    UDP_BASE_DEST_PORT = 65000 #Destination port of outgoing messages (each message will be sent to 127.0.0.1:UDP_WRITE_PORT)

    ROBOT_LIST = [3]
    #ROBOT_LIST = [1,2,3,4,5,6,7,8,9,10]


    UDP_PORTS_WRITE = [UDP_BASE_READ_PORT + robot_number for robot_number in ROBOT_LIST]
    UDP_PORTS_DEST = [UDP_BASE_DEST_PORT + robot_number for robot_number in ROBOT_LIST]
    UDP_PORTS_READ = [UDP_BASE_WRITE_PORT + robot_number for robot_number in ROBOT_LIST]

    READ_SOCKET_TIMEOUT = 0.05
    MESSAGE = b"1"

    ALIVE_CLIENT_TIMEOUT = 2000

    print("UDP target IP: %s" % UDP_IP)

    read_sockets = setup_read_sockets(ROBOT_LIST, UDP_IP, UDP_PORTS_READ, READ_SOCKET_TIMEOUT)
    write_sockets = setup_write_sockets(ROBOT_LIST, UDP_IP, UDP_PORTS_WRITE)

    print("message: %s" % MESSAGE)


    while(True):
        for robot_number, read_sock in read_sockets.items():

            #Receive message
            data, addr = receive_data_from_socket(read_sock["robot_number"])
            
            if data is not None and addr is not None:
                handleMessage(data, robot_number)
        
            #If robot has not sent messages for a while, set it as not "alive"
            if read_sock["last_message_timestamp"] - time.time()>ALIVE_CLIENT_TIMEOUT:
                print("Robot %d stopped sending: setting to not alive" % robot_number)
                read_sockets[robot_number]["alive"] = False
                write_sockets[robot_number]["alive"] = False

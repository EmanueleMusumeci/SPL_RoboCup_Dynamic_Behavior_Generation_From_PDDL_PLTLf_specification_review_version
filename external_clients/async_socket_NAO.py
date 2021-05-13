import socket
import errno
import sys, os
import time
from time import sleep
import json


def setup_read_socket(robot_number, ip, read_port, read_timeout):
    #Setup read socket
    print("Robot %d: UDP read port: %s" % (robot_number, read_port))
    
    read_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    read_addr = socket.getaddrinfo(ip, read_port, socket.AF_INET, socket.SOCK_DGRAM)[0]

    read_socket.bind((ip, read_port))
    read_socket.setblocking(0)
    read_socket.settimeout(read_timeout)
    
    return read_socket, read_addr

def setup_write_socket(robot_number, ip, write_port, dest_port):
    #Setup write socket
    print("Robot %d: UDP write port: %s" % (robot_number, write_port))
    
    write_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    write_addr = socket.getaddrinfo(ip, write_port, socket.AF_INET, socket.SOCK_DGRAM)[0]
    dest_addr = socket.getaddrinfo(ip, dest_port, socket.AF_INET, socket.SOCK_DGRAM)[0]

    write_socket.bind((ip, write_port))
    write_socket.setblocking(0)
    
    return write_socket, write_addr, dest_addr


class NAOCommunicationController:
    def __init__(self, robot_number, ip, write_port, dest_port, read_port, read_socket_timeout, GUIWebSocket):
        super().__init__()

        self.robot_number = robot_number

        self.alive = False

        self.write_socket, self.write_addr, self.dest_addr = setup_write_socket(robot_number, ip, write_port, dest_port)
        self.last_sent_message_timestamp = 0

        self.readNAOSocket, self.read_addr = setup_read_socket(robot_number, ip, read_port, read_socket_timeout)
        self.last_received_message_timestamp = 0
        self.read_socket_timeout = read_socket_timeout

    def send_message(self, data):
        self.write_socket.sendto(data, self.dest_addr[4])
        self.last_sent_message_timestamp = time.time()

    def send_keepalive_request(self):
        self.send_message(b"uthere?\x00")

    def send_keepalive_response(self):
        self.send_message(b"yeah\x00")
        print("KEEPALIVE response sent %s" % b"yeah\x00")

    def handleMessage(self, data):

        #Update received message timestamp
        self.last_received_message_timestamp = time.time()
        #Set robot as "alive" because we received data from it
        self.alive = True

        #Decode data into a string (might be improved later)
        data = data.decode('utf-8')

        print("Received message from robot %d: %s" % (self.robot_number, data))

        #keep-alive message
        if(data == "uthere?"):
            print("KEEPALIVE received from robot %s" % self.robot_number)
            self.send_keepalive_response()
            
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

    def receive_data(self):
        data, addr = None, None
        try:
            #read_sock["sock"].connect(read_sock["address"][4]) #Not really needed for UDP but should solve some resolving issues in the local API
            data, addr = self.read_socket.recvfrom(1024)
        except socket.timeout:
            #SOCKET TIMEOUT (or something similar)
            #print('No data received from robot: %s (Timed out)' % robot_number)
            pass
        except socket.error as e:
            err = e.args[0]
            if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                #SOCKET FULL (or something similar)
                sleep(1)
                print('No data received from robot: %s (Socket buffer full)' % robot_number)
            else:
                #ERROR
                print("GENERIC ERROR: ",e)
                sys.exit(1)

        if data is not None:
            self.last_received_message_timestamp = time.time()
        #If robot has not sent messages for a while, set it as not "alive"
        elif time.time() - self.last_received_message_timestamp > self.read_socket_timeout:
            print("Robot %d has not sent for %d seconds: setting to not alive" % (self.robot_number, self.read_socket_timeout))
            self.alive = False

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

    READ_SOCKET_TIMEOUT = 0.05
    ALIVE_CLIENT_TIMEOUT = 2000
    
    print("UDP target IP: %s" % UDP_IP)

    GUIWebSocket = None

    robotCommunicationControllers = {}
    for robot_number in ROBOT_LIST:
        write_port = UDP_BASE_WRITE_PORT + robot_number
        read_port = UDP_BASE_READ_PORT + robot_number
        dest_port = UDP_BASE_DEST_PORT + robot_number
        robotCommunicationControllers[robot_number] = NAOCommunicationController(robot_number, UDP_IP, 
                                                                                write_port, dest_port, 
                                                                                read_port,  READ_SOCKET_TIMEOUT,
                                                                                ALIVE_CLIENT_TIMEOUT,
                                                                                GUIWebSocket)
    
    while(True):
        for robot_number, robot_comm in robotCommunicationControllers.items():

            #Receive message
            data, addr = robot_comm.receive_data()
            
            if data is not None and addr is not None:
                robot_comm.handleMessage(data)
        

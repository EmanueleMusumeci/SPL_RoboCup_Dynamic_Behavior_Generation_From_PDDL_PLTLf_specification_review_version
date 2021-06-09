import socket
import errno
import sys, os
import time
from time import sleep
import json

from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor


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
    def __init__(self, robot_number, ip, 
                write_port, dest_port, 
                read_port, read_socket_timeout, 
                active_client_timeout, 
                fieldGUI):
        
        self.robot_number = robot_number

        self.alive = False
        self.active_client_timeout = active_client_timeout
        
        self.write_socket, self.write_addr, self.dest_addr = setup_write_socket(robot_number, ip, write_port, dest_port)
        self.last_sent_message_timestamp = time.time()

        self.read_socket, self.read_addr = setup_read_socket(robot_number, ip, read_port, read_socket_timeout)
        self.last_received_message_timestamp = time.time()
        self.read_socket_timeout = read_socket_timeout

        self.fieldGUI = fieldGUI        

    def send_keepalive_request(self):
        self.send_data(b"uthere?\x00")

    def send_keepalive_response(self):
        self.send_data(b"yeah\x00")
        print("KEEPALIVE response sent %s" % b"yeah\x00")

    #MESSAGE HANDLERS
    def handleRobotPoseMessage(self, content, message_info):
        content_fields = content.split(",")
        robot_angle = float(content_fields[1])
        position = (float(content_fields[2]), float(content_fields[3]))

        #print("[Robot %d] Angle: %f, Position: (%f, %f)" % (self.robot_number, robot_angle, position[0], position[1]))

        self.fieldGUI.updateRobotPosition(self.robot_number, message_info["timestamp"], position)

    def handleBallPositionMessage(self, content, message_info):
        content_fields = content.split(",")
        position = (float(content_fields[1]), float(content_fields[2]))

        #print("[Robot %d] Ball position: (%f, %f)" % (self.robot_number, position[0], position[1]))

        self.fieldGUI.updateBallPosition(self.robot_number, message_info["timestamp"], position)
    
    def handleObstaclesMessage(self, content, message_info):
        content_fields = content.split(";")[1:]
        obstacle_list = [(float(obsX), float(obsY)) for (obsX, obsY) in [obs.split(",") for obs in content_fields]]
        
        print("[Robot %d] Obstacles: ", ", ".join(["({}, {})".format(obsX, obsY) for (obsX, obsY) in obstacle_list]))

        self.fieldGUI.updateObstaclesPosition(self.robot_number, message_info["timestamp"], obstacle_list)
    

    def handleMessage(self, data):
        #Decode data into a string (might be improved later)
        data = data.decode('utf-8')

        #print("Received message from robot %d: %s" % (self.robot_number, data))

        message_fields = data.split("|")

        header = message_fields[0]
        content = message_fields[1]
        
        message_info = {"timestamp" : time.time(), "robot_number" : self.robot_number, "content" : content}

        if len(header)>0:
            for field in header.split("."):
                if field.startswith("timestamp"):
                    message_info["timestamp"] = int(field.split(",")[1])

                #Not necessary to extract robot number because we know it already
                #elif field.startswith("robot_number"):
                #    message_info["robot_number"] = field.split(",")[1] 

        #print(message_info)

        #keep-alive message
        if content == "uthere?":
            print("KEEPALIVE received from robot %s" % self.robot_number)
            self.send_keepalive_response()
        else:
            if content.startswith("robot_pose"):
                self.handleRobotPoseMessage(content, message_info)

            elif content.startswith("ball_position"):
                self.handleBallPositionMessage(content, message_info)

            elif content.startswith("obstacles"):
                self.handleObstaclesMessage(content, message_info)


    def send_data(self, data):
        self.write_socket.sendto(data, self.dest_addr[4])
        self.last_sent_message_timestamp = time.time()

    def receive_data(self):
        #print("Checking messages for robot ", robot_number, " on port ", self.read_addr[4][1])
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
            #Update received message timestamp
            self.last_received_message_timestamp = time.time()
            #Set robot as "alive" because we received data from it
            self.alive = True
        #If robot has not sent messages for a while, set it as not "alive"
        elif self.alive and time.time() - self.last_received_message_timestamp > self.active_client_timeout:
            print("Robot %d has not sent for %d seconds: setting to not alive" % (self.robot_number, time.time() - self.last_received_message_timestamp))
            self.alive = False

        return data, addr

class FieldGUI:
    def __init__(self, websocket_ip, websocket_port):
        self.ball_position = {"position_timestamp": time.time(), "update_timestamp": time.time(), "last_seen_by_robot" : -1, "position" : (0, 0)}

        self.robot_positions = {}

        self.obstacles = {"position_timestamp": time.time(), "update_timestamp": time.time(), "obstacles" : []}
    
        self.webSocket = None

    def update(self):
        #print("\n\nBALL POSITION\n",self.ball_position)
        #print("\nROBOT POSITIONS\n",self.robot_positions)
        #print("\nOBSTACLES\n",self.obstacles)
        pass

    def updateRobotPosition(self, robot_number, timestamp, position):
        if robot_number not in self.robot_positions.keys() or timestamp > self.robot_positions[robot_number]["position_timestamp"]:
            self.robot_positions[robot_number] = {"position_timestamp": timestamp, "update_timestamp": time.time(), "position" : position}
            
            self.update()

    #MIGHT ADD A CONTROL ON THE DISTANCE OF THE BALL TO CHOOSE THE MOST PRECISE OBSERVATION
    def updateBallPosition(self, seen_by_robot_number, timestamp, position):
        if timestamp > self.ball_position["position_timestamp"]:
            self.ball_position = {"position_timestamp": timestamp, "update_timestamp": time.time(), "last_seen_by_robot" : seen_by_robot_number, "position" : position}
        
            self.update()
    
    #MIGHT ADD A CONTROL ON THE DISTANCE OF THE BALL TO CHOOSE THE MOST PRECISE OBSERVATION
    def updateObstaclesPosition(self, seen_by_robot_number, timestamp, obstacles):
        if timestamp > self.obstacles["position_timestamp"]:
            self.obstacles = {"position_timestamp": timestamp, "update_timestamp": time.time(), "last_seen_by_robot" : seen_by_robot_number, "obstacles" : obstacles}
            
            self.update()

if __name__ == "__main__":

    UDP_IP = "127.0.0.1"
    #UDP_IP = "localhost"

    #Notice: 
    # - the DEST_PORTs here have to be the READ_PORTs on the robots
    # - the READ_PORTs here have to be the DEST_PORTs on the robots
    # - the WRITE_PORTs here have to be different from those of the robots
    UDP_BASE_READ_PORT = 65100 #Listening port for incoming messages
    UDP_BASE_WRITE_PORT = 65200  #Socket port of outgoing messages for each robot
    UDP_BASE_DEST_PORT = 65000 #Destination port of outgoing messages (each message will be sent to 127.0.0.1:UDP_WRITE_PORT)

    ROBOT_LIST = [3]
    #ROBOT_LIST = [1,2,3,4,5,6,7,8,9,10]

    READ_SOCKET_TIMEOUT = 0.05
    ALIVE_CLIENT_TIMEOUT = 1
    
    print("UDP target IP: %s" % UDP_IP)

    GUIWebSocket = None

    fieldGUI = FieldGUI(None, None)

    robotCommunicationControllers = {}
    for robot_number in ROBOT_LIST:
        write_port = UDP_BASE_WRITE_PORT + robot_number
        read_port = UDP_BASE_READ_PORT + robot_number
        dest_port = UDP_BASE_DEST_PORT + robot_number
        robotCommunicationControllers[robot_number] = NAOCommunicationController(robot_number, UDP_IP, 
                                                                                write_port, dest_port, 
                                                                                read_port,  READ_SOCKET_TIMEOUT,
                                                                                ALIVE_CLIENT_TIMEOUT,
                                                                                fieldGUI)
    

    while(True):
        for robot_number, robot_comm in robotCommunicationControllers.items():
            
            #Receive message
            data, addr = robot_comm.receive_data()
            
            if data is not None and addr is not None:
                robot_comm.handleMessage(data)
        
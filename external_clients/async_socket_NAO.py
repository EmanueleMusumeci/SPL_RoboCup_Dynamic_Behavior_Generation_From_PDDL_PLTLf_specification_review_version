from dataclasses import field
import socket
import errno
import sys, os
import signal
import time
from time import sleep
import json
import math
from typing import Dict, Type
from abc import ABC, abstractmethod
from numpy import isin

import twisted
from twisted.internet import reactor, task

from python_utils.singleton import Singleton

#NOTICE: This node controls the task list BUT the IdleTask and the initial PerformInitialSpeechTask are hard-coded in the robot behavior

#TODO: NAOCommunicationController should not call the GUIController directly but do it through the TeamBehaviorController
#TODO: [LATER] TeamBehaviorController should "discover" available robots on the network somehow 
#   1) Use periodic announcement messages from the robots (sending robot name, ip and role)
#   2) Add NAOCommunicationController dynamically when receiving announcement messages from the robot
#   3) If robot crashes, it will be handled correctly by the NAOCommunicationController
#           Maybe delete the interface after a lot of time has passed since the last message
#   4) If the Python server crashes, no problem... Maybe the robot should go into idle mode
#TODO: Populate task list using the gui or the DFA depending on the current modality (hard-coded until the user can choose it from the GUI)
#TODO: [LATER] Users unable to control should have a "passive-only" GUI (showing only the field)
#TODO: Decouple robot number from role
#TODO: Registries (Literal, Value, Action) will be initialized in the TeamBehaviorController
#TODO: Framework will stream booleans while other conditions will be available as templates (only structure, no literals or actions) but initialized by GUI messages carrying literal and action names and parameters 
#      NOTICE: this is all hard-coded for now
#TODO: TeamBehaviorController will manage: NAOCommunicationController, GUIController and DFAs


def setup_read_socket(udpProtocol, reactor, local_interface_ip, read_port, read_timeout, debug_message):
    #Setup read socket
    if debug_message is not None: 
        print(debug_message)
    
    read_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    read_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    read_addr = socket.getaddrinfo(local_interface_ip, read_port, socket.AF_INET, socket.SOCK_DGRAM)[0]

    read_socket.bind((local_interface_ip, read_port))
    read_socket.setblocking(False)
    read_socket.settimeout(read_timeout)
    
    reactor_read_socket = reactor.adoptDatagramPort(read_socket.fileno(), socket.AF_INET, udpProtocol)
    
    return read_socket, read_addr

def setup_write_socket(local_interface_ip, write_port, remote_dest_ip, dest_port = None, debug_message = None):
    #Setup write socket
    if debug_message is not None: 
        print(debug_message)
    
    write_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    write_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    write_addr = socket.getaddrinfo(local_interface_ip, write_port, socket.AF_INET, socket.SOCK_DGRAM)[0]
    
    dest_addr = None
    if dest_port is not None:
        dest_addr = socket.getaddrinfo(remote_dest_ip, dest_port, socket.AF_INET, socket.SOCK_DGRAM)[0]

    write_socket.bind((local_interface_ip, write_port))
    write_socket.setblocking(False)
    
    return write_socket, write_addr, dest_addr

class NAOCommunicationController(twisted.internet.protocol.DatagramProtocol):
    def __init__(self, reactor, robot_number, local_interface_ip, 
                write_port, remote_dest_ip, dest_port, 
                read_port, read_socket_timeout, 
                active_client_timeout, 
                frontend_controller):
        
        self.robot_number = robot_number

        self.alive = False
        self.active_client_timeout = active_client_timeout
        
        self.write_socket, self.write_addr, self.dest_addr = setup_write_socket(local_interface_ip, write_port, remote_dest_ip, dest_port=dest_port, 
                                                                                debug_message=("Writing from %s, with local port %s\n\t-> Robot %d: IP %s, UDP write port: %s" % (local_interface_ip, write_port, robot_number, remote_dest_ip, dest_port)))
        self.last_sent_message_timestamp = time.time()

        self.read_socket, self.read_addr = setup_read_socket(self, reactor, local_interface_ip, read_port, read_socket_timeout, 
                                                                debug_message=("Listening on %s, with local port %s\n\t-> Robot %d: IP %s, UDP read port: %s" % (local_interface_ip, read_port, robot_number, remote_dest_ip, write_port)))
        self.last_received_message_timestamp = time.time()
        self.read_socket_timeout = read_socket_timeout

        self.frontend_controller = frontend_controller

        self.check_alive_task = task.LoopingCall(self.check_alive_states)
        self.check_last_task_id_task = task.LoopingCall(self.check_last_task_id)
        self.update_assigned_tasks_task = task.LoopingCall(self.update_assigned_tasks)
        
        #Should be empty in case the robot just started as well but in case the server crashed and the robot was still running
        #this updates the server with the latest issued tasks
        self.check_task_queue_task = task.LoopingCall(self.check_task_queue)

        self.lastReceivedTaskID = None
        self.lastCompletedTaskID = None


    def close_sockets(self):
        self.write_socket.close()
        self.read_socket.close()

    def send_keepalive_request(self):
        self.send_string("uthere?")

    def send_keepalive_response(self):
        self.send_string("yeah")
        #print("KEEPALIVE response sent %s" % b"yeah\x00")

    #MESSAGE HANDLERS
    def handleLastTaskIDMessage(self, content, message_info):
        #print(content)
        content_fields = content.split(",")
        self.lastReceivedTaskID = int(content_fields[1])
        self.lastCompletedTaskID = int(content_fields[2])

        self.frontend_controller.updateRobotLastTaskID(self.robot_number, message_info["timestamp"], self.lastReceivedTaskID, self.lastCompletedTaskID)
        self.update_assigned_tasks()

    def handleRobotPoseMessage(self, content, message_info):
        content_fields = content.split(",")
        robot_angle = float(content_fields[1])
        robot_position = (float(content_fields[2]), float(content_fields[3]))

        #print("[Robot %d] Angle: %f, Position: (%f, %f)" % (self.robot_number, robot_angle, robot_position[0], robot_position[1]))

        self.frontend_controller.updateRobotPosition(self.robot_number, message_info["timestamp"], robot_angle, robot_position)

    def handleBallPositionMessage(self, content, message_info):
        content_fields = content.split(",")
        position = (float(content_fields[1]), float(content_fields[2]))

        #print("[Robot %d] Ball position: (%f, %f)" % (self.robot_number, position[0], position[1]))

        self.frontend_controller.updateBallPosition(self.robot_number, message_info["timestamp"], position)
    
    def handleRobotRoleMessage(self, content, message_info):
        content_fields = content.split(",")
        role = content_fields[1]

        #print("[Robot %d] Robot role: %s" % (self.robot_number, role))

        self.frontend_controller.updateRobotRole(self.robot_number, message_info["timestamp"], role)
    
    def handleObstaclesMessage(self, content, message_info):
        content_fields = content.split(";")[1:]
        obstacle_list = [(float(obsX), float(obsY)) for (obsX, obsY) in [obs.split(",") for obs in content_fields]]
        
        #print("[Robot {}] Obstacles: ".format(self.robot_number), ", ".join(["({}, {})".format(obsX, obsY) for (obsX, obsY) in obstacle_list]))

        self.frontend_controller.updateObstaclesPosition(self.robot_number, message_info["timestamp"], obstacle_list)
    
    def handleTaskQueueMessage(self, content, message_info):
        content_fields = list(filter(None, content.split(";")[1:]))
        #print(content_fields)

        lastReceivedTaskID = int(content_fields[0].split(",")[1])
        lastCompletedTaskID = int(content_fields[0].split(",")[2])
        
        self.frontend_controller.resetRobotTaskQueue(self.robot_number, message_info["timestamp"], lastReceivedTaskID, lastCompletedTaskID)
        
        #If there isn't any task return
        if(len(content_fields) == 1): return

        for task in content_fields[1:]:
            task_fields = task.split(",")
            
            taskType = task_fields[0]
            taskID = int(task_fields[1])

            if len(task_fields)==2:
                self.frontend_controller.addTask(self.robot_number, taskType, taskID)
            elif len(task_fields)==4:
                xPos = int(math.floor(float(task_fields[2])))
                yPos = int(math.floor(float(task_fields[3])))
                self.frontend_controller.addTask(self.robot_number, taskType, taskID, parameters = [xPos, yPos])
            else:
                print("Wrong taskQueue format")
                raise
        


    def handleMessage(self, data):
        #Decode data into a string (might be improved later)
        data = data.decode('utf-8')

        #print("Received message from robot %d: %s" % (self.robot_number, data))

        message_fields = data.split("|")

        header = message_fields[0]
        content = message_fields[1]
        
        message_info = {"timestamp" : None, "robot_number" : self.robot_number, "content" : content}

        if len(header)>0:
            for field in header.split("."):
                if field.startswith("timestamp"):
                    message_info["timestamp"] = int(field.split(",")[1])

                #Not necessary to extract robot number because we know it already
                #elif field.startswith("robot_number"):
                #    message_info["robot_number"] = field.split(",")[1] 

        #Set last_received_message_timestamp
        self.last_received_message_timestamp = time.time()
        self.set_to_alive()
        
        #print(message_info)

        #keep-alive message
        if content == "uthere?":
            #print("KEEPALIVE received from robot %s" % self.robot_number)
            self.send_keepalive_response()
        else:
            if content.startswith("lastTaskID"):
                self.handleLastTaskIDMessage(content, message_info)

            elif content.startswith("robot_pose"):
                self.handleRobotPoseMessage(content, message_info)

            elif content.startswith("ball_position"):
                self.handleBallPositionMessage(content, message_info)

            elif content.startswith("robot_role"):
                self.handleRobotRoleMessage(content, message_info)

            elif content.startswith("obstacles"):
                self.handleObstaclesMessage(content, message_info)

            elif content.startswith("lastTaskQueue"):
                print(content)
                self.handleTaskQueueMessage(content, message_info)
                print("Stopping task queue check")
                if self.check_task_queue_task.running:
                    self.check_task_queue_task.stop()

    def send_string(self, string, terminator="\x00"):
        print("Sending data to robot "+str(self.robot_number)+": "+string)
        self.send_data(str.encode(string+terminator))
    
    def send_data(self, data):
        #print(self.dest_addr[4])
        self.write_socket.sendto(data, self.dest_addr[4])
        self.last_sent_message_timestamp = time.time()

    #Overridden method
    def datagramReceived(self, data, addr):
        self.handleMessage(data)

    def set_to_alive(self):
        #Set as alive 
        self.alive = True
        
        self.frontend_controller.enableClient(self.robot_number)

    def set_to_not_alive(self):
        self.alive = False
        self.lastReceivedTaskID = None
        self.lastCompletedTaskID = None
        #If the robot is not alive (e.g. it crashed) we reset the task list. If the connection dropped, the robot will keep executing the tasks anyway
        #and also the last task ID will be correctly received upon reconnection
        self.frontend_controller.resetRobotTasks()
        #We also reset the current robot role to 'unknown' and clean its DFA
        self.frontend_controller.resetRobotRole()

        self.frontend_controller.sendRobotNotRespondingMessage(self.robot_number)
        print("Starting task queue check")
        if not self.check_task_queue_task.running:
            self.check_task_queue_task.start(LAST_TASK_ID_TIMEOUT)


    def check_alive_states(self):
        #print("Starting alive check")
        if self.alive:
            if time.time() - self.last_received_message_timestamp > self.active_client_timeout:
                print("Robot %d has not sent for %f seconds: setting to not alive" % (self.robot_number, time.time() - self.last_received_message_timestamp))
                self.set_to_not_alive()
            else:
                print("Robot {} is alive".format(self.robot_number))

    def check_last_task_id(self):
        #print("Asking last task ID")
        if self.alive:
            self.send_string("lastTaskID?")

    def check_task_queue(self):
        if self.alive:
            self.send_string("lastTaskQueue?")

    def update_assigned_tasks(self):
        if self.alive:
            #Check that:
            # 1) The robot role is known
            # 2) There is a DFA for that role
            if self.robot_number in self.frontend_controller.robot_number_to_role.keys() \
                and \
                    self.frontend_controller.robot_number_to_role[self.robot_number]["role"] in self.frontend_controller.role_to_DFA.keys():
                    
                    robot_role = self.frontend_controller.robot_number_to_role[self.robot_number]["role"]
                    print("Robot %d role: %s" % (self.robot_number, robot_role))
                    dfa_state = self.frontend_controller.role_to_DFA[robot_role].get_current_state()
            if self.lastReceivedTaskID is not None and self.robot_number in self.frontend_controller.robot_tasks.keys():
                task_list_string = "taskQueue|"

                #if len(self.frontend_controller.robot_tasks[self.robot_number]["tasks"]) == 0:
                #    return

                if self.frontend_controller.robot_tasks[self.robot_number]["reset"]:
                    self.frontend_controller.robot_tasks[self.robot_number]["reset"] = False
                    self.send_string("resetTasks")

                if len(self.frontend_controller.robot_tasks[self.robot_number]["tasksToDelete"]) > 0:
                    for taskID in self.frontend_controller.robot_tasks[self.robot_number]["tasksToDelete"]:
                        self.send_string("deleteTask,"+taskID)
                    self.frontend_controller.robot_tasks[self.robot_number]["tasksToDelete"] = []

                for i, task in enumerate(self.frontend_controller.robot_tasks[self.robot_number]["tasks"]):
                    if(i > 0): 
                        task_list_string+= ";"
                    task_list_string += task["taskType"]

                    task_list_string+=","+str(task["taskID"])

                    if task["parameters"] is not None:
                        for parameter in task["parameters"]:
                            task_list_string+=","+str(parameter)
                    
                
                #print(task_list_string)
                self.send_string(task_list_string)
            else:
                return

class FrontendController(ABC, twisted.internet.protocol.DatagramProtocol):
    def __init__(self, reactor, manager : Type["BehaviorManager"] = None):
        self.reactor = reactor

        if manager is not None:
            assert isinstance(manager, BehaviorManager)
        self.manager = manager

    def set_manager(self, manager : Type["BehaviorManager"]):
        assert self.manager is None or isinstance(manager, BehaviorManager)
        self.manager = manager

    #HINT for twisted.internet.protocol.DatagramProtocol only abstract method
    @abstractmethod
    def datagramReceived(self, data, addr):
        pass

class GUIController(FrontendController):
    def __init__(self, 
        reactor, 
        
        local_websocket_interface_ip, read_port, 
        remote_dest_ip, write_port, client_port,
        
        read_socket_timeout,
        active_client_timeout
    ):
        super().__init__(reactor)
        self.ball_position = {"position_timestamp": None, "update_timestamp": time.time(), "last_seen_by_robot" : -1, "position" : (0, 0)}


        self.robot_role_to_number = {}
        self.robot_number_to_role = {}

        self.robot_positions = {}

        self.robot_tasks = {}

        self.obstacles = {"position_timestamp": None, "update_timestamp": time.time(), "obstacles" : []}
        
        self.write_socket, self.write_addr, self.dest_addr = setup_write_socket(local_websocket_interface_ip, write_port, remote_dest_ip, dest_port=client_port, 
                                                                                debug_message=("Web client LOCAL UDP write port: %s\n -> Web client REMOTE UDP read port: %s" % (write_port, client_port)))
        self.last_sent_message_timestamp = None

        self.read_socket, self.read_addr = setup_read_socket(self, reactor, local_websocket_interface_ip, read_port, read_socket_timeout, 
                                                            debug_message=("Web client UDP read port: %s" % (read_port)))
        self.last_received_message_timestamp = None

        self.last_client_id = None
        self.alive = False
        self.active_client_timeout = active_client_timeout

        self.check_alive_task = task.LoopingCall(self.check_client_alive)

        self.role_to_DFA = {}


    def close_sockets(self):
        self.write_socket.close()
        self.read_socket.close()

    def register_new_client(self, address, port, client_id):
        self.dest_addr = socket.getaddrinfo(address, port, socket.AF_INET, socket.SOCK_DGRAM)[0]
        self.last_client_id = client_id
        self.alive = True

    def generate_header(self, robotNumber):
        return str("robotNumber,"+str(robotNumber))

    def generateLastQueueMessage(self, robotNumber):
        message = "taskQueue;"
        #TaskIDs
        message += "lastTaskID,"+str(self.robot_tasks[robotNumber]["lastReceivedTaskID"])+","+str(self.robot_tasks[robotNumber]["lastCompletedTaskID"])
        
        if(len(self.robot_tasks[robotNumber]["tasks"]) == 0):
            return message

        #TaskQueues
        message += ";"
        for i, task in enumerate(self.robot_tasks[robotNumber]["tasks"]):
            
            message += str(task["taskType"])+","+str(task["taskID"])
            
            if task["parameters"] is not None:
                message += ","+(",".join([str(param) for param in task["parameters"]]))
            
            if len(self.robot_tasks[robotNumber]["tasks"]) > 0 \
            and \
            i < len(self.robot_tasks[robotNumber]["tasks"]) - 1: 
                message += ";"
        
        return message
            
    def update_client(self, robotNumber, data):
        self.send_string(self.generate_header(robotNumber) + "|" + data)

    def send_string(self, string, terminator="\x00"):
        self.send_data(str.encode(string+terminator))
    
    def send_data(self, data):
        if self.dest_addr is not None:
            self.write_socket.sendto(data, self.dest_addr[4])
            self.last_sent_message_timestamp = time.time()

    def scheduleRobotTasksReset(self, robot_number):
        self.robot_tasks[robot_number]["reset"] = True

    #Only called in case the server just restarted after the crash and the robot was still running and had already assigned tasks
    #so the server will recover the control session
    def resetRobotTaskQueue(self, robotNumber, timestamp, lastReceivedTaskID, lastCompletedTaskID):
        #if robotNumber not in self.robot_tasks.keys():
        self.robot_tasks[robotNumber] = {"last_timestamp": timestamp, "update_timestamp" : time.time(), "lastCompletedTaskID" : lastCompletedTaskID, "lastReceivedTaskID" : lastReceivedTaskID, "tasks" : [], "reset" : False, "tasksToDelete" : []}
        

    def updateRobotLastTaskID(self, robotNumber, timestamp, lastReceivedTaskID, lastCompletedTaskID):
        if robotNumber not in self.robot_tasks.keys():
            self.robot_tasks[robotNumber] = {"last_timestamp": timestamp, "update_timestamp" : time.time(), "lastCompletedTaskID" : lastCompletedTaskID, "lastReceivedTaskID" : lastReceivedTaskID, "tasks" : [], "reset" : False, "tasksToDelete" : []}

        else:
        #elif timestamp > self.robot_tasks[robotNumber]["last_timestamp"]:

            #In this case scan the list of tasks currently assigned to the robot #robotNumber and delete the ones with
            #a taskID lower than the last ID of the last completed task received by the robot controller
            old_tasks = self.robot_tasks[robotNumber]["tasks"]
            new_tasks = []
            for task in old_tasks:
                if task["taskID"] > self.robot_tasks[robotNumber]["lastCompletedTaskID"]:
                    new_tasks.append(task)

            self.robot_tasks[robotNumber] = {"last_timestamp": timestamp, "update_timestamp" : time.time(), "lastCompletedTaskID" : lastCompletedTaskID, "lastReceivedTaskID" : lastReceivedTaskID, "tasks" : new_tasks, "reset" : False, "tasksToDelete" : self.robot_tasks[robotNumber]["tasksToDelete"]}
        
        self.update_client(robotNumber, self.generateLastQueueMessage(robotNumber))

    def updateRobotPosition(self, robotNumber, timestamp, angle, position):
        #if robotNumber not in self.robot_positions.keys() or timestamp > self.robot_positions[robotNumber]["position_timestamp"]:
        self.robot_positions[robotNumber] = {"position_timestamp": timestamp, "update_timestamp": time.time(), "position" : position}

        self.update_client(robotNumber, "robotPosition:"+str(robotNumber)+","+str(angle)+","+str(self.robot_positions[robotNumber]["position"][0])+","+str(self.robot_positions[robotNumber]["position"][1]))

    #MIGHT ADD A CONTROL ON THE DISTANCE OF THE BALL TO CHOOSE THE MOST PRECISE OBSERVATION
    def updateRobotRole(self, robotNumber, timestamp, role):
        #if timestamp > self.ball_position["position_timestamp"]:
        self.robot_role_to_number[role] = {"role_timestamp": timestamp, "update_timestamp": time.time(), "robot_number" : robotNumber}
        self.robot_number_to_role[robotNumber] = {"role_timestamp": timestamp, "update_timestamp": time.time(), "role" : role}
        
        self.update_client(robotNumber, "robotRole:"+str(robotNumber)+","+role)
    
    #MIGHT ADD A CONTROL ON THE DISTANCE OF THE BALL TO CHOOSE THE MOST PRECISE OBSERVATION
    def updateBallPosition(self, robotNumber, timestamp, position):
        #if timestamp > self.ball_position["position_timestamp"]:
        self.ball_position = {"position_timestamp": timestamp, "update_timestamp": time.time(), "last_seen_by_robot" : robotNumber, "position" : position}
        
        self.update_client(robotNumber, "ballPosition:"+str(self.ball_position["position"][0])+","+str(self.ball_position["position"][1]))
    
    def updateObstaclesPosition(self, robotNumber, timestamp, obstacles):
        #if timestamp is None or timestamp > self.obstacles["position_timestamp"]:
        self.obstacles = {"position_timestamp": timestamp, "update_timestamp": time.time(), "last_seen_by_robot" : robotNumber, "obstacles" : obstacles}
        
        self.update_client(robotNumber, "obstacles:"+";".join([str(obstacle[0]) + "," + str(obstacle[1]) for obstacle in self.obstacles["obstacles"]]))
    
    def send_keepalive_request(self):
        self.send_string("uthere?")

    def send_keepalive_response(self):
        self.send_string("yeah")
        #print("KEEPALIVE response sent: %s" % b"yeah\x00")

    def addTask(self, robotNumber, taskType, taskID, parameters = None):
        #Don't add the task if it is already contained in the task list for the robot #robotNumber
        for task in self.robot_tasks[robotNumber]["tasks"]:
            if task["taskID"] == taskID:
                return
        
        self.robot_tasks[robotNumber]["tasks"].append({"taskType" : taskType, "taskID" : taskID, "parameters" : parameters})
        
    def handleClientMessage(self, data, addr):
        #Decode data into a string (might be improved later)
        data = data.decode('utf-8')

        print("Received message from client")

        print(data)

        message_fields = data.split("|")

        header = message_fields[0]
        content = message_fields[1]
        
        message_info = {"timestamp" : None, "client_id" : None, "content" : content}

        if len(header)>0:
            for field in header.split(";"):
                if field.startswith("timestamp"):
                    message_info["timestamp"] = int(field.split(",")[1])

                if field.startswith("client_id"):
                    message_info["client_id"] = field.split(",")[1:]
                    
        if message_info["client_id"] is None:
            print("Client not recognized (NO CLIENT ID RECEIVED)")
            raise


        #Update the client with the latest info if the last_client_id changed (new client)
        if self.last_client_id != message_info["client_id"]:
            self.register_new_client(message_info["client_id"][0], message_info["client_id"][1], message_info["client_id"][2])

        #Set last_received_message_timestamp
        self.last_received_message_timestamp = time.time()
        #Set as alive 
        self.alive = True

        #print(content)

        #keep-alive message
        if content == "uthere?":
            #print("KEEPALIVE received from client %s" % message_info["client_id"])
            self.send_keepalive_response()
        else:   
            if content.startswith("resetTasks"):
                print(content)
                print("resetTasks")
                robotNumber = int(content.split(",")[1])
                self.robot_tasks[robotNumber]["reset"] = True
            elif content.startswith("deleteTask"):
                print(content)
                robotNumber = int(content.split(",")[1])
                taskID = int(content.split(",")[2])
                self.robot_tasks[robotNumber]["tasksToDelete"].append(str(taskID))

            elif content.startswith("taskType"):
                content_fields = content.split(":")[1].split(",")
                robotNumber = int(content_fields[0])
                selectionMode = content_fields[1]
                taskType = content_fields[2]
                taskID = int(content_fields[3])
                if selectionMode == "noSelection":
                    self.addTask(robotNumber, taskType, taskID)
                elif selectionMode == "singlePosition":
                    xPos = int(content_fields[4])
                    yPos = int(content_fields[5])
                    self.addTask(robotNumber, taskType, taskID, parameters = [xPos, yPos])

    def datagramReceived(self, data, addr):
        self.handleClientMessage(data, addr)

    def set_to_not_alive(self):
        self.alive = False

    def enableClient(self, robotNumber):
        pass

    def sendRobotNotRespondingMessage(self, robotNumber):
        self.send_string(self.generate_header(robotNumber) + "|robotNotResponding")

    def check_client_alive(self):
        #print("Starting alive check")
        if self.alive:
            if time.time() - self.last_received_message_timestamp > self.active_client_timeout:
                #print("Client %s has not sent for %d seconds: setting to not alive" % (self.last_client_id, time.time() - self.last_received_message_timestamp))
                self.set_to_not_alive()
            else:
                pass
                #print("Client {} is alive".format(self.last_client_id))


class BehaviorManager(metaclass=Singleton):
    def __init__(self,
            local_interface_ip : str,                           #Local LAN interface IP
            robor_number_to_robot_IP_map : Dict[str, str],      #Maps robot number to data (including robot ip)
            robot_udp_base_read_port : int = 65100,             #Listening port for incoming messages from robots
            robot_udp_base_write_port : int = 65200,            #Socket port of outgoing messages for each robot (each message will be sent to 127.0.0.1:UDP_BASE_WRITE_PORT+<robot_number>)
            robot_udp_base_dest_port : int = 65000,             #Destination port of outgoing messages (each message will be sent to 127.0.0.1:UDP_BASE_WRITE_PORT+<robot_number>)

            robot_read_socket_timeout : int = 0.05,             #Timeout after which the read operation on a socket is aborted
            robot_alive_timeout : int = 0.05,                   #Timeout after which a non-responding robot is considered "not alive" (crashed or disconnected)

            frontend_controller : FrontendController = None,
        ):

        print("LOCAL INTERFACE IP: %s" % LOCAL_INTERFACE_IP)

        ''' 
        ______________________________
        |                             |
        |  SETUP FRONTEND CONTROLLER  |
        |_____________________________|

        '''
        
        self.frontend_controller = frontend_controller
        self.frontend_controller.set_manager(self)

    #-----------------------------------------------------------------

        ''' 
        __________________________________
        |                                  |
        |  SETUP ROBOT NETWORK INTERFACES  |
        |__________________________________|

        '''
        self.local_interface_ip = local_interface_ip
        self.robot_udp_base_read_port = robot_udp_base_read_port
        self.robot_udp_base_write_port = robot_udp_base_write_port
        self.robot_udp_base_dest_port = robot_udp_base_dest_port

#TODO: Populate this with a broadcast discovery process (e.g. broadcast a message on the LAN until some robot responds) maybe asyncronous and continuous
        self.robor_number_to_robot_IP_map = robor_number_to_robot_IP_map

        self.robot_read_socket_timeout = robot_read_socket_timeout
        self.robot_alive_timeout = robot_alive_timeout

#TODO: Populate this by broadcast (as above)
        self.robot_communication_controllers = {}
        for robot_number in robot_number_to_robot_ip_map.keys():
            write_port = self.robot_udp_base_write_port + robot_number
            read_port = self.robot_udp_base_read_port + robot_number
            dest_port = self.robot_udp_base_dest_port + robot_number
            self.robot_communication_controllers[robot_number] = \
                NAOCommunicationController(
                    reactor, 
                    robot_number, 
                    self.local_interface_ip, 
                    
                    write_port, self.robor_number_to_robot_IP_map[robot_number]["robotIP"], dest_port, 
                    read_port,  self.robot_read_socket_timeout,
                    
                    self.robot_alive_timeout,
                    
                    self.frontend_controller
                )
        #-----------------------------------------------------------------

        ''' 
        ________________________
        |                        |
        |  SETUP PERIODIC TASKS  |
        |________________________|

        '''

        #Start LoopingCalls for each robot controller

        #Does two things:
        # 1) If the robot is currently alive but it has not sent any message since controller.alive_client_timeout seconds, will set it to "not alive"
        # 2) If the robot has recently (less than controller.alive_client_timeout seconds ago) sent the any message, will
        #       send the message "uthere?", as a KEEPALIVE request to the robot. The response to the KEEPALIVE is DISABLED on the robot as the
        #       regular traffic is already more than enough.                                    
        for controller in self.robot_communication_controllers.values():
            controller.check_alive_task.start(ALIVE_CLIENT_TIMEOUT)      

        for controller in self.robot_communication_controllers.values():
            #Sends a "lastTaskID?" message to the robot every LAST_TASK_ID_TIMEOUT seconds, to which the robot will answer with a message like:
            #   "lastTaskID,<last task ID received by robot>,<last task ID completed by robot>"
            #See the NAOCommunicationController documentation to see how the "lastTaskID" message is handled         
            controller.check_last_task_id_task.start(LAST_TASK_ID_TIMEOUT)
            #Sends a "lastTaskQueue?" message to the robot every LAST_TASK_ID_TIMEOUT seconds, to which the robot will answer with a message like:
            #   "lastTaskQueue;lastTaskID,<last task ID received by robot>,<last task ID completed by robot>;<task type>,<task fields separated by commas>;..."
            #where "..." represents other task types and their fields (i.e. destination for robot or for ball)
            #   NOTICE: this task is activated when the Python server first starts or whenever the robot is detected as "not alive". It is deactivated when the robot
            #   is detected to be "alive"
            controller.check_task_queue_task.start(LAST_TASK_ID_TIMEOUT)

        for controller in self.robot_communication_controllers.values():
            controller.update_assigned_tasks_task.start(UPDATE_TASKS_TIMEOUT)

        #Start LoopingCalls for the GUI client
        self.frontend_controller.check_alive_task.start(ALIVE_CLIENT_TIMEOUT)
    #-----------------------------------------------------------------






    def exit_gracefully(self, signal=None, frame=None):
        print("Exiting server gracefully...")
        reactor.stop()
        for controller in self.robot_communication_controllers.values():
            controller.close_sockets()
        self.frontend_controller.close_sockets()



if __name__ == "__main__":


#-----------------------------------------------------------------

    ''' 
     ____________________________
    |                            |
    |         TIMEOUTS           |
    |____________________________|

    '''

    READ_SOCKET_TIMEOUT = 0.05
    ALIVE_CLIENT_TIMEOUT = 1
#TODO maybe i'll have to use ms instead for these two as well
    #TIME between two subsequent "lastTaskID?" requests, that periodically check the last executed task ID
    LAST_TASK_ID_TIMEOUT = 1
    #TIME between two subsequent "taskQueue?" requests, that periodically ask the task
    UPDATE_TASKS_TIMEOUT = 0.2
#-----------------------------------------------------------------





    ''' 
    ______________________________
    |                              |
    |  FRONTEND NETWORK ADDRESSES  |
    |______________________________|

    '''

    LOCAL_WEBSOCKET_INTERFACE_IP = "127.0.0.1"
    FRONTEND_SOCKET_IP = "127.0.0.1"

    
    WEB_CLIENT_READ_PORT = 65300
    WEB_CLIENT_WRITE_PORT = 65400
    WEB_CLIENT_REMOTE_READ_PORT = 65301
    WEB_CLIENT_REMOTE_WRITE_PORT = 65401
#-----------------------------------------------------------------

    ''' 
    _____________________________________
    |                                    |
    |  SETUP FRONTEND NETWORK INTERFACE  |
    |____________________________________|

    '''
    frontend_controller = GUIController(
        reactor, 
        LOCAL_WEBSOCKET_INTERFACE_IP, WEB_CLIENT_READ_PORT, 
        FRONTEND_SOCKET_IP, WEB_CLIENT_WRITE_PORT, WEB_CLIENT_REMOTE_READ_PORT, 
        
        READ_SOCKET_TIMEOUT, 
        ALIVE_CLIENT_TIMEOUT
    )




#TODO: replace this with an automatic robot discovery broadcast
    ''' 
     ____________________________
    |                            |
    |  ROBOTS NETWORK ADDRESSES  |
    |____________________________|

    '''

    USE_LOCALHOST = True


    robot_number_to_robot_ip_map = {}
    if(USE_LOCALHOST):
        LOCAL_INTERFACE_IP = "127.0.0.1"
        robot_number_to_robot_ip_map[3] = {"robotName" : "Caligola", "robotIP" : "127.0.0.1"}
    else:
        LOCAL_INTERFACE_IP = "10.0.255.226"
        robot_number_to_robot_ip_map[3] = {"robotName" : "Caligola", "robotIP" : "10.0.19.17"}
    
    #Notice: 
    # - the DEST_PORTs here have to be the READ_PORTs on the robots
    # - the READ_PORTs here have to be the DEST_PORTs on the robots
    # - the WRITE_PORTs here have to be different from those of the robots
    UDP_BASE_READ_PORT = 65100 #Listening port for incoming messages from robots
    UDP_BASE_WRITE_PORT = 65200  #Socket port of outgoing messages for each robot (each message will be sent to 127.0.0.1:UDP_BASE_WRITE_PORT+<robot_number>)
    UDP_BASE_DEST_PORT = 65000 #Destination port of outgoing messages (each message will be sent to 127.0.0.1:UDP_BASE_WRITE_PORT+<robot_number>)
#-----------------------------------------------------------------




    ''' 
    ______________________________
    |                             |
    |  SETUP BEHAVIOR CONTROLLER  |
    |_____________________________|

    '''

    behavior_controller = BehaviorManager(
        LOCAL_INTERFACE_IP,                                             #Local interface ip address
        
        #Robot communication info
        robor_number_to_robot_IP_map = robot_number_to_robot_ip_map,    #Robot number to robot data (including IP) map (will be replaced by an automatic discovery mechanism)
        robot_udp_base_read_port = UDP_BASE_READ_PORT,                  
        robot_udp_base_write_port = UDP_BASE_WRITE_PORT,
        robot_udp_base_dest_port = UDP_BASE_DEST_PORT,
        
        robot_read_socket_timeout = READ_SOCKET_TIMEOUT,
        robot_alive_timeout = ALIVE_CLIENT_TIMEOUT,

        #Frontend controller
        frontend_controller = frontend_controller
    )




    ''' 
    ________________________________________
    |                                        |
    |  CORRECTLY HANDLE TERMINATION SIGNALS  |
    |________________________________________|

    '''

    #Handle interrupt signals
    signal.signal(signal.SIGINT, behavior_controller.exit_gracefully)
    signal.signal(signal.SIGABRT, behavior_controller.exit_gracefully)
    signal.signal(signal.SIGTERM, behavior_controller.exit_gracefully)
    signal.signal(signal.SIGQUIT, behavior_controller.exit_gracefully)
    signal.signal(signal.SIGHUP, behavior_controller.exit_gracefully)
    #-----------------------------------------------------------------




    ''' 
     _____________________________
    |                             |
    |  START TWISTED UPDATE LOOP  |
    |_____________________________|

    '''

    try:
        reactor.run()
    except:
        behavior_controller.exit_gracefully()

            
from abc import ABC, abstractmethod
from typing import Type
import socket
import time

import twisted
from twisted.internet import task

from communication.socket_utils import setup_read_socket, setup_write_socket
from communication.communication_manager import CommunicationManager


class FrontendController(ABC, twisted.internet.protocol.DatagramProtocol):
    def __init__(self, reactor, communication_manager : Type["CommunicationManager"] = None):
        self.reactor = reactor

        if communication_manager is not None:
            assert isinstance(communication_manager, CommunicationManager)
        self.communication_manager = communication_manager

    def set_manager(self, communication_manager : Type["CommunicationManager"]):
        assert self.communication_manager is None or isinstance(communication_manager, CommunicationManager)
        self.communication_manager = communication_manager

    #HINT for twisted.internet.protocol.DatagramProtocol only abstract method
    @abstractmethod
    def datagramReceived(self, data, addr):
        pass

    #Update frontend status
    @abstractmethod
    def update(self, robot_number, message):
        pass    
    
    #Enable frontend
    @abstractmethod
    def enable(self):
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

        self.communication_manager = None



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
        message += "lastTaskID,"+str(self.communication_manager.getLastReceivedTaskID(robotNumber))+","+str(self.communication_manager.getLastCompletedTaskID(robotNumber))
        
        if(len(self.communication_manager.getRobotTasks(robotNumber)) == 0):
            return message

        #TaskQueues
        message += ";"
        for i, task in enumerate(self.communication_manager.getRobotTasks(robotNumber)):
            
            message += str(task["taskType"])+","+str(task["taskID"])
            
            if task["parameters"] is not None:
                message += ","+(",".join([str(param) for param in task["parameters"]]))
            
            if len(self.communication_manager.getRobotTasks(robotNumber)) > 0 \
            and \
            i < len(self.communication_manager.getRobotTasks(robotNumber)) - 1: 
                message += ";"
        
        return message
            
    def update(self, robotNumber, message):
        self.send_string(self.generate_header(robotNumber) + "|" + message)

    def send_string(self, string, terminator="\x00"):
        self.send_data(str.encode(string+terminator))
    
    def send_data(self, data):
        if self.dest_addr is not None:
            self.write_socket.sendto(data, self.dest_addr[4])
            self.last_sent_message_timestamp = time.time()


    def send_keepalive_request(self):
        self.send_string("uthere?")

    def send_keepalive_response(self):
        self.send_string("yeah")
        #print("KEEPALIVE response sent: %s" % b"yeah\x00")

        
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
                self.communication_manager.scheduleRobotTasksReset(robotNumber)
            elif content.startswith("deleteTask"):
                print(content)
                robotNumber = int(content.split(",")[1])
                taskID = int(content.split(",")[2])
                self.communication_manager.scheduleTaskDeletion(robotNumber, taskID)

            elif content.startswith("taskType"):
                content_fields = content.split(":")[1].split(",")
                robotNumber = int(content_fields[0])
                selectionMode = content_fields[1]
                taskType = content_fields[2]
                taskID = int(content_fields[3])
                if selectionMode == "noSelection":
                    self.communication_manager.addTask(robotNumber, taskType, taskID)
                elif selectionMode == "singlePosition":
                    xPos = int(content_fields[4])
                    yPos = int(content_fields[5])
                    self.communication_manager.addTask(robotNumber, taskType, taskID, parameters = [xPos, yPos])

    def datagramReceived(self, data, addr):
        self.handleClientMessage(data, addr)

    def set_to_not_alive(self):
        self.alive = False

    def enable(self):
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

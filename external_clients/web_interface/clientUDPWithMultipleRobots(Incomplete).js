//NOTICE: this client acts only as a buffer between multiple control clients and the python server, just repeating messages and managing connections (with keepalives)
//all the protocol logic is contained in the python client and the js client

//TODO: the js client has to send its unique ID to this node which in turn sends it to the python script in order to open a "control session"
//TODO: this node should manage disconnection of js clients by updating its representations of the "control session"
//TODO: the python server should manage disconnection of this client through keepalive and by deleting all client control sessions in case keepalive not received

//This server is supposed to act an interface between the Python server and the browser-based GUI, to serve it to multiple clients.



//Use something like this to map a client ID to each robot and therefore deliver messages correctly to the clients
//var robotToClientIDMap = new Map();
//var clientIDToConnectionMap = new Map();
//To use it, the robotNumber should be added to messages sent by the Python server to this server in order to correctly deliver them

//Perciò se il messaggio è relativo al clientID deve essere intercettato, gestito per aggiornare le mappe e ripetuto, altrimenti deve essere ripetuto e basta

var currentConnection;

var WebSocketServer = require('websocket').server;

//DO NOT TOUCH
var remote_read_port = 65300
//

var local_ip = "127.0.0.1"
var local_read_port = 65301
var local_write_port = 65300

//Compute client id as hex hash of the address
var crypto = require('crypto');
var client_id = crypto.createHash('md5').update(local_ip + local_read_port).digest('hex');

const dgram = require('dgram');
const read_socket = dgram.createSocket('udp4');

read_socket.on('error', (err) => {
  console.log(`server error:\n${err.stack}`);
  read_socket.close();
});

read_socket.on('message', (msg, rinfo) => {
  //console.log(`server got: ${msg} from ${rinfo.address}:${rinfo.port}`);

  message_fields = message.split("|")
          
  message_header = message_fields[0]
  message_content = message_fields[1]

  headerFields = message_header.split(";")
  
  var receivedRobotNumber;
  for(field of headerFields)
  {
    if(field.startsWith("robotNumber"))
    {
      receivedRobotNumber = field.split(",")[1]
    }
  }

  if(currentConnection != undefined)
  {
    if(receivedRobotNumber!=undefined && receivedRobotNumber == currentRobotNumber)
    {
      console.log("Received robot number: "+receivedRobotNumber+", Current robot number: "+currentRobotNumber)
      currentConnection.sendUTF(message_content)
    }
  }
});

read_socket.on('listening', () => {
  const address = read_socket.address();
  console.log(`Backend communication socket listening ${address.address}:${address.port}`);
});

read_socket.bind(65301);


//Register this server in the backend
var message_header = Buffer.from('client_id,'+local_ip+','+local_read_port+','+client_id+'|');
var message_content = "uthere?"

var message = message_header + message_content

const write_socket = dgram.createSocket('udp4');

write_socket.send(message, 65300, '127.0.0.1', (err) => {
  write_socket.close();
});

//THIS (INCOMPLETE) CODE IS SUPPOSED TO HANDLE MULTIPLE ROBOTS
//AS A PROOF OF CONCEPT THOUGH WE'RE USING A SINGLE ROBOT
/*var robotNumberToClientID = new Map();
var clientIDToConnection = new Map();

var availableRobots = [];

function addRobotControlSession(clientID, robotNumber, connection)
{
  if(robotNumberToClientID.has(robotNumber))
  {
    console.log("Robot #"+robotNumber+" is already taken")
    return false;
  }

  robotNumberToClientID[robotNumber] = clientID;
  clientIDToConnection[clientID] = connection;
  connection.clientID = clientID;
  connection.robotNumber = robotNumber;
  
  var newAvailableRobots = []
  for(availableRobotNumber of availableRobots)
  {
    if(availableRobotNumber != robotNumber)
    {
      newAvailableRobots.push(availableRobotNumber);
    }
  }
  availableRobots = newAvailableRobots;
  return true;
}

function removeRobotControlSession(clientID)
{
  if(clientIDToConnection.has(clientID))
  {
    clientIDToConnection.delete(clientID);
    for(robotNumber of robotNumberToClientID.keys())
    {
      if(robotNumberToClientID[robotNumber] == clientID)
      {
        robotNumberToClientID.delete(robotNumber);
        availableRobots.push(robotNumber)
      }
    }
  }
}

function getRobotControlSessionConnectionFromRobotNumber(robotNumber)
{
  if(robotNumberToClientID.has(robotNumber)) return clientIDToConnection[robotNumberToClientID[robotNumber]];
  else return undefined;
}

function getRobotControlSessionConnectionFromClientID(clientID)
{
  if(clientIDToConnection.has(clientID)) return clientIDToConnection[clientID];
  else return undefined;
}

function listAvailableRobots()
{
  if(availableRobots.length() == 0)
    return undefined;

  var res = "availableRobots";
  for(robotNumber of availableRobots)
  {
    res+=","+robotNumber
  }
}*/

function generateWebSocketHeader()
{
  return "TOCLIENT!"
}

//Setup Web GUI
//Taken from https://medium.com/@martin.sikora/node-js-websocket-simple-chat-tutorial-2def3a841b61
var http = require('http');

var server = http.createServer(function(request, response) {
  // process HTTP request. Since we're writing just WebSockets
  // server we don't have to implement anything.
});
server.listen(4000, function() { });

// create the server
wsServer = new WebSocketServer({
  httpServer: server,
  keepalive: true,
  keepaliveInterva: 5000,
  keepaliveGracePeriod: 5000,
  //autoAcceptConnections: true
});

var currentClientID = undefined;
var currentRobotNumber = undefined;
var currentConnection = undefined;
var waitingConnections = [];

// WebSocket server
wsServer.on('request', function(request) {

  receivedConnection = request.accept(null, request.origin);

  console.log("Frontend communication socket listening")

  //If no client is connected right now, set the request as the current connection
  //else tell the client another client is already connected and he's disabled for now
  if(currentConnection == undefined)
  {
    currentConnection = receivedConnection;
  }
  else
  {
    receivedConnection.sendUTF("disabled")
    waitingConnections.push(receivedConnection)
  }

  console.log((new Date()) + ' Connection accepted.');
  receivedConnection.on('message', function(message) {
      if (message.type === 'utf8') {
          console.log('Received Message: ' + message.utf8Data);
          
          //Skip messages that I sent (to the client)
          if(message.utf8Data.split("!")[0] == "TOCLIENT") return;

          message_fields = message.utf8Data.split("!")[1].split("|")
          console.log(message_fields)
          
          message_header = message_fields[0]
          message_content = message_fields[1]

          receivedClientID = message_header.split(";")[0].split(",")[1]
          receivedRobotNumber = message_header.split(";")[1].split(",")[1]

          if(currentClientID == undefined) 
          {
            currentClientID = receivedClientID;
            currentRobotNumber = receivedRobotNumber;
          }
          else if(currentClientID != receivedClientID)
          {
            receivedConnection.sendUTF(generateWebSocketHeader() + "disable")
            return;
          }
          
          if(message_content === "uthere?")
          {
            receivedConnection.sendUTF(generateWebSocketHeader() + "yeah")
          }

          /*if(!getRobotControlSessionConnectionFromClientID)
          {
            if(message_content === "availableRobots")
            {
              var availableRobotsString = listAvailableRobots()
              if(availableRobotsString == undefined)
              {
                receivedConnection.sendUTF("wait")
              }
              else
              {
                receivedConnection.sendUTF(listAvailableRobots())
              }
            }
            else addRobotControlSession(clientID, robotNumber, receivedConnection)
          }
          else
          {
            write_socket.send(message.utf8Data)
          }*/
          
          try{
            write_socket.send(message.utf8Data, 65300, '127.0.0.1', (err) => {
              write_socket.close();
            });
          } catch {}
        }
  });

  receivedConnection.on('close', function(connection) {
    if(waitingConnections.length > 0)
    {
      currentConnection = waitingConnections.shift();
      currentConnection.sendUTF("enabled")
    }
    else
    {
      currentConnection = undefined;
    }
    currentClientID = undefined;
    currentRobotNumber = undefined;
  });
});
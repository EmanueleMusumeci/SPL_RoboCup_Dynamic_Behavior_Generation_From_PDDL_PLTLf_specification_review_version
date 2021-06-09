//This server is supposed to act an interface between the Python server and the browser-based GUI, to serve it to multiple clients.


//NOTICE: this client acts only as a buffer between multiple control clients and the python server, just repeating messages and managing connections (with keepalives)
//all the protocol logic is contained in the python client and the js client

//DONE: the js client has to send its unique ID to this node which in turn sends it to the python script in order to open a "control session"
//DONE: this node should manage disconnection of js clients by updating its representations of the "control session"
  //TODO: test with multiple clients at once
//DONE: the python server should manage disconnection of this client through keepalive and by deleting all client control sessions in case keepalive not received
  //TODO: Solve bug that happens when the NodeJS server is restarted and the JS client reconnects but orders issued are not delivered (probly wrong webSocket reference is used in the client)
//TODO when robot crashes, it is still appearing in the web interface but commands can still be issued even though they're not received: grey out the buttons
//DONE also, clear the task queue when the robot disconnects

//TODO: Fix the robot still keeping the same position after disconnecting

//TODO: reset tasks from GUI

//TODO: fix the graphical render
//TODO: improve graphics: action markers, task bar in GUI, client disabled screen

//TODO: interactions

var currentConnection;

var WebSocketServer = require('websocket').server;

//DO NOT TOUCH
var remote_read_port = 65300
//

var BACKEND_CLIENT_ID = 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
  var r = Math.random() * 16 | 0, v = c == 'x' ? r : (r & 0x3 | 0x8);
  return v.toString(16);
});

var BACKEND_INACTIVE = true;
var BACKEND_KEEPALIVE_SEND_INTERVAL = 4000;
var BACKEND_KEEPALIVE_RECEIVE_INTERVAL = 2000;

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

read_socket.on('message', (message, rinfo) => {
  
  BACKEND_INACTIVE = false;

  //console.log(`[BACKEND] Received message: ${message} from ${rinfo.address}:${rinfo.port}`)
  message_fields = message.toString().split("|")
          
  message_header = message_fields[0]
  //console.log(message_header)
  message_content = message_fields[1]
  //console.log(message_content)

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
    //console.log("currentRobotNumber:" + currentRobotNumber)
    //console.log("receivedRobotNumber:" + receivedRobotNumber)
    if(currentRobotNumber!=undefined && receivedRobotNumber == currentRobotNumber)
    {
      if(message_content.startsWith("disableClient"))
      {
        currentConnection.sendUTF(generateWebSocketHeader() + "disableClient")
      }
      else
      {
        //console.log("Received robot number: "+currentRobotNumber+", Current robot number: "+currentRobotNumber)
        currentConnection.sendUTF(generateWebSocketHeader() + message_content)
      }
    }
  }
});

read_socket.on('listening', () => {
  const address = read_socket.address();
  console.log(`Backend communication socket listening ${address.address}:${address.port}`);
});

read_socket.bind(65301);

//Write socket
function generateHeader()
{
  return Buffer.from('client_id,'+local_ip+','+local_read_port+','+client_id+'|');
}

function send_registration_message_to_backend()
{
  //Register this server in the backend
  var message_header = generateHeader();
  var message_content = "uthere?"

  var message = message_header + message_content

  write_socket.send(message, 65300, '127.0.0.1', (err) => {});
}

var backend_keepalive_receive_timeout;
var backend_keepalive_send_timeout;
function requestKeepaliveToBackend() {
    
  //console.log("[FRONTEND; KEEPALIVE] Requesting keepalive to backend")
  send_registration_message_to_backend()

  //Set a timeout to be cleared in case a keepalive is received
  backend_keepalive_receive_timeout = setTimeout(function () {
      //If this is executed, the server failed to respond to the keepalive
      BACKEND_INACTIVE = true;
  }, BACKEND_KEEPALIVE_RECEIVE_INTERVAL);
}

var write_socket = dgram.createSocket('udp4');
backend_keepalive_send_timeout = setInterval(requestKeepaliveToBackend, BACKEND_KEEPALIVE_SEND_INTERVAL)





// ---------------------
// | Client WebSockets |
// ---------------------


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
  //keepalive: true,
  //keepaliveInterva: 5000,
  //keepaliveGracePeriod: 5000,
  //autoAcceptConnections: true
});

var currentClientID = undefined;
var currentRobotNumber = undefined;
var currentConnection = undefined;
var waitingConnections = [];

// WebSocket server
wsServer.on('request', function(request) {

  receivedConnection = request.accept(null, request.origin);

  console.log("[FRONTEND; SETUP] Frontend communication socket listening")

  //If no client is connected right now, set the request as the current connection
  //else tell the client another client is already connected and he's disabled for now
  if(currentConnection == undefined)
  {
    currentConnection = receivedConnection;
  }
  else
  {
    receivedConnection.sendUTF("disableClient")
    waitingConnections.push(receivedConnection)
  }

  console.log('[FRONTEND; REGISTERING] Connection accepted.');
  receivedConnection.on('message', function(message) {
      if (message.type === 'utf8') {
        
        if(message.utf8Data == undefined) return;

        //console.log('[FRONTEND] Received Message: ' + message.utf8Data);
        
        //Skip messages that I sent (to the client)
        if(message.utf8Data.split("!")[0] == "TOCLIENT") return;
        
        message_fields = message.utf8Data.split("!")[1].split("|")
        //console.log(message_fields)
        
        message_header = message_fields[0]
        message_content = message_fields[1]

        receivedClientID = message_header.split(";")[0].split(",")[1]
        receivedRobotNumber = message_header.split(";")[1].split(",")[1]
        //console.log(receivedClientID)
        //console.log(currentClientID)
        //console.log(receivedRobotNumber)

        if(currentClientID == undefined) 
        {
          currentClientID = receivedClientID;
          currentRobotNumber = receivedRobotNumber;
          currentConnection = receivedConnection;
          console.log("[FRONTEND; REGISTERING] Registering new web interface client (ID: "+currentClientID+", Robot number: "+currentRobotNumber+")")
        }
        else if(currentClientID != receivedClientID)
        {
          console.log("[FRONTEND; REGISTERING] One client is already connected. Disabling new client (ID: "+receivedClientID+")")
          receivedConnection.sendUTF(generateWebSocketHeader() + "disableClient")
          return;
        }
        if(message_content === "uthere?")
        {
          //console.log("[FRONTEND; KEEPALIVE] Received keepalive from client (ID: "+receivedClientID+"). Sending response.")
          receivedConnection.sendUTF(generateWebSocketHeader() + "yeah")
        }
        else if(message_content.startsWith("taskType"))
        {
          console.log(message_content)
          var message_fields = message_content.split(":")[1].split(",")

          var selectionMode = message_fields[1]
          var taskType = message_fields[2]
          var taskID = message_fields[3]

          if(selectionMode === "noSelection")
          {
            console.log("[FRONTEND] Received new task from client (ID: "+receivedClientID+"). taskType: "+ taskType +", taskID: "+taskID)
          }
          else
          {
            var xPos = Math.floor(parseFloat(message_fields[4]))
            var yPos = Math.floor(parseFloat(message_fields[5]))
            console.log("[FRONTEND] Received new task from client (ID: "+receivedClientID+"). taskType: "+ taskType +", taskID: "+taskID+", selectionMode: "+selectionMode+", position ("+xPos+","+yPos+")")
          }

          try{
            write_socket.send(generateHeader() + message_content, 65300, '127.0.0.1', (err) => {});
          } catch {}
        }
        else if(message_content.startsWith("resetTasks"))
        {
          console.log("resetTasks")
          try{
            write_socket.send(generateHeader() + message_content, 65300, '127.0.0.1', (err) => {});
          } catch {}
        }
        /*else if(message_content.length > 0)
        {
          try{
            write_socket.send(message_content, 65300, '127.0.0.1', (err) => {});
          } catch {}
        }*/
      }
  });

  receivedConnection.on('close', function(connection) {
    //console.log("[FRONTEND] Closing connection")
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
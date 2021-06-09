"use strict";


//TODO: Terminare protocollo di rete (task queue attuale, conviene inviare i task attuali dal backend in caso di disconnessione e riconnessione del client, invio di task appena assegnati)
//TODO: Implementare rappresentazione grafica task queue
//TODO: Migliorare schermata client disabilitato
//TODO: Imporre al server NodeJS di disabilitare il client se il backend è inattivo!!!


var ACTIVE_ROBOT_NUMBER = 3
    
var canvas;
var gl;

var showMesh, showNormals, showContours, showLightPosition, showTexture;

var FIELD_WIDTH = 10400;
var FIELD_HEIGHT = 7400;

var FIELD_LINES_WIDTH = 50;

var CENTER_CIRCLE_RADIUS = 750;
var CENTER_DOT_RADIUS = 35;

var Y_POS_SIDE_LINE = 3000;
var X_POS_SIDE_LINE = 4500;

var PENALTY_AREA_WIDTH = 1650;
var PENALTY_AREA_HEIGHT = 4000;

var GOAL_BOX_WIDTH = 600;
var GOAL_BOX_HEIGHT = 2200;

var TARGET_RADIUS = 100;

var PENALTY_CROSS_SIZE = 100;
var PENALTY_CROSS_X_DISTANCE = 1300;
var X_POS_PENALTY_CROSS = X_POS_SIDE_LINE - PENALTY_CROSS_X_DISTANCE;

var GOAL_POST_RADIUS = 60;
var GOAL_POST_WIDTH = 500;
var GOAL_POST_HEIGHT = 1500;

var BALL_RADIUS = 100;
var ROBOT_RADIUS = 150;
var OBSTACLE_RADIUS = ROBOT_RADIUS;

var fieldBackgroundColor = "#00FF00";
var fieldLinesColor = "#00FF00";
var ballColor = "#00FF00";
var robotColor = "#00FF00";
var obstacleColor = "";
var goalColor = "";
var targetPositionColor = "";

var canvasTextColor = "#FFFFFF";
var canvasContainerBackgroundColor = "#DDDDEE";
var canvasBackgroundColor = "#00BB00";


var robotNumbersToPositions = new Map();
var ballPosition = [0.0, 0.0];
var obstaclesPositions = [];

var taskTypeToTaskLabel = new Map();
var lastReceivedTaskID = -1;
var lastCompletedTaskID = -1;
var currentTaskList = [];
var currentTaskPreviews = [];

var mouseOverCanvas = false;

function drawCircle(ctx, centerX, centerY, radius, fillColor = undefined, lineColor = "#000000", lineWidth = 5, withRespectToCenter = undefined)
{
    if(withRespectToCenter != undefined)
    {
        centerX = centerX + withRespectToCenter[0];
        centerY = centerY + withRespectToCenter[1];
    }

    ctx.beginPath();
    ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI, false);
    
    if(fillColor == undefined)
        fillColor = canvasBackgroundColor;
        ctx.fillStyle = fillColor;
    ctx.fill();

    ctx.lineWidth = lineWidth;
    ctx.strokeStyle = lineColor;
    ctx.stroke();
}

function drawLine(ctx, fromX, fromY, toX, toY, lineColor = "#000000", lineWidth = 5, withRespectToCenter = undefined)
{
    if(withRespectToCenter != undefined)
    {
        fromX = fromX + withRespectToCenter[0];
        toX = toX + withRespectToCenter[0];

        fromY = fromY + withRespectToCenter[1];
        toY = toY + withRespectToCenter[1];
    }

    ctx.beginPath();
    ctx.moveTo(fromX, fromY);
    ctx.lineTo(toX, toY);

    ctx.lineWidth = lineWidth;
    ctx.strokeStyle = lineColor;
    ctx.stroke();
}

//Taken from https://stackoverflow.com/questions/808826/draw-arrow-on-canvas-tag
function drawArrow(ctx, fromX, fromY, toX, toY, lineColor = "#000000", lineWidth = 20, arrowHeadLength = 50, withRespectToCenter = undefined) 
{
    if(withRespectToCenter != undefined)
    {
        fromX = fromX + withRespectToCenter[0];
        toX = toX + withRespectToCenter[0];

        fromY = fromY + withRespectToCenter[1];
        toY = toY + withRespectToCenter[1];
    }
    
    var dx = toX - fromX;
    var dy = toY - fromY;
    var angle = Math.atan2(dy, dx);


    ctx.lineWidth = lineWidth;
    ctx.strokeStyle = lineColor;

    ctx.beginPath();
    ctx.moveTo(fromX, fromY);
    ctx.lineTo(toX, toY);
    ctx.lineTo(toX - arrowHeadLength * Math.cos(angle - Math.PI / 6), toY - arrowHeadLength * Math.sin(angle - Math.PI / 6));
    ctx.moveTo(toX, toY);
    ctx.lineTo(toX - arrowHeadLength * Math.cos(angle + Math.PI / 6), toY - arrowHeadLength * Math.sin(angle + Math.PI / 6));
    ctx.stroke();
}

function drawRectangle(ctx, topLeftX, topLeftY, width, length, fillColor = undefined, lineColor = "#000000", lineWidth = 5, withRespectToCenter = undefined)
{
    if(withRespectToCenter != undefined)
    {
        topLeftX = topLeftX + withRespectToCenter[0];
        topLeftY = topLeftY + withRespectToCenter[1];
    }
        
    ctx.fillStyle = fillColor;
    ctx.lineWidth = lineWidth;
    ctx.strokeStyle = lineColor;
    ctx.fillRect(topLeftX, topLeftY, width, length); 
    ctx.stroke();

}

function drawField(canvas)
{
    var ctx = canvas.getContext('2d');
    
    drawRectangle(ctx, 0, 0, canvas.width, canvas.height, canvasBackgroundColor, "#FFFFFF", 5)

    //Center circle
    drawCircle(ctx, 0, 0, CENTER_CIRCLE_RADIUS, canvasBackgroundColor, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
    drawCircle(ctx, 0, 0, CENTER_DOT_RADIUS, canvasBackgroundColor, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

    //Middle field line
    drawLine(ctx, 0, Y_POS_SIDE_LINE, 0, -Y_POS_SIDE_LINE, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
    
    //Horizontal side lines
    drawLine(ctx, X_POS_SIDE_LINE, Y_POS_SIDE_LINE, -X_POS_SIDE_LINE, Y_POS_SIDE_LINE, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
    drawLine(ctx, X_POS_SIDE_LINE, -Y_POS_SIDE_LINE, -X_POS_SIDE_LINE, -Y_POS_SIDE_LINE, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

    //Vertical side lines
    drawLine(ctx, -X_POS_SIDE_LINE, -Y_POS_SIDE_LINE, -X_POS_SIDE_LINE, Y_POS_SIDE_LINE, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
    drawLine(ctx, X_POS_SIDE_LINE, -Y_POS_SIDE_LINE, X_POS_SIDE_LINE, Y_POS_SIDE_LINE, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

    //Left penalty area
        //Horizontal lines
        drawLine(ctx, -X_POS_SIDE_LINE, -PENALTY_AREA_HEIGHT/2, -X_POS_SIDE_LINE + PENALTY_AREA_WIDTH, -PENALTY_AREA_HEIGHT/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        drawLine(ctx, -X_POS_SIDE_LINE, PENALTY_AREA_HEIGHT/2, -X_POS_SIDE_LINE + PENALTY_AREA_WIDTH, PENALTY_AREA_HEIGHT/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

        //Vertical line
        drawLine(ctx, -X_POS_SIDE_LINE + PENALTY_AREA_WIDTH, -PENALTY_AREA_HEIGHT/2, -X_POS_SIDE_LINE + PENALTY_AREA_WIDTH, PENALTY_AREA_HEIGHT/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

    //Left goal box
        //Horizontal lines
        drawLine(ctx, -X_POS_SIDE_LINE, -GOAL_BOX_HEIGHT/2, -X_POS_SIDE_LINE + GOAL_BOX_WIDTH, -GOAL_BOX_HEIGHT/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        drawLine(ctx, -X_POS_SIDE_LINE, GOAL_BOX_HEIGHT/2, -X_POS_SIDE_LINE + GOAL_BOX_WIDTH, GOAL_BOX_HEIGHT/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

        //Vertical line
        drawLine(ctx, -X_POS_SIDE_LINE + GOAL_BOX_WIDTH, -GOAL_BOX_HEIGHT/2, -X_POS_SIDE_LINE + GOAL_BOX_WIDTH, GOAL_BOX_HEIGHT/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
    
    //Right penalty area
        //Horizontal lines
        drawLine(ctx, X_POS_SIDE_LINE, -PENALTY_AREA_HEIGHT/2, X_POS_SIDE_LINE - PENALTY_AREA_WIDTH, -PENALTY_AREA_HEIGHT/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        drawLine(ctx, X_POS_SIDE_LINE, PENALTY_AREA_HEIGHT/2, X_POS_SIDE_LINE - PENALTY_AREA_WIDTH, PENALTY_AREA_HEIGHT/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

        //Vertical line
        drawLine(ctx, X_POS_SIDE_LINE - PENALTY_AREA_WIDTH, -PENALTY_AREA_HEIGHT/2, X_POS_SIDE_LINE - PENALTY_AREA_WIDTH, PENALTY_AREA_HEIGHT/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

    //Right goal box
        //Horizontal lines
        drawLine(ctx, X_POS_SIDE_LINE, -GOAL_BOX_HEIGHT/2, X_POS_SIDE_LINE - GOAL_BOX_WIDTH, -GOAL_BOX_HEIGHT/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        drawLine(ctx, X_POS_SIDE_LINE, GOAL_BOX_HEIGHT/2, X_POS_SIDE_LINE - GOAL_BOX_WIDTH, GOAL_BOX_HEIGHT/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

        //Vertical line
        drawLine(ctx, X_POS_SIDE_LINE - GOAL_BOX_WIDTH, -GOAL_BOX_HEIGHT/2, X_POS_SIDE_LINE - GOAL_BOX_WIDTH, GOAL_BOX_HEIGHT/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

    //Left penalty cross
    drawLine(ctx, -X_POS_PENALTY_CROSS, PENALTY_CROSS_SIZE/2, -X_POS_PENALTY_CROSS, -PENALTY_CROSS_SIZE/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
    drawLine(ctx, -X_POS_PENALTY_CROSS - PENALTY_CROSS_SIZE/2, 0, -X_POS_PENALTY_CROSS + PENALTY_CROSS_SIZE/2, 0, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

    //Right penalty cross
    drawLine(ctx, X_POS_PENALTY_CROSS, PENALTY_CROSS_SIZE/2, X_POS_PENALTY_CROSS, -PENALTY_CROSS_SIZE/2, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
    drawLine(ctx, X_POS_PENALTY_CROSS + PENALTY_CROSS_SIZE/2, 0, X_POS_PENALTY_CROSS - PENALTY_CROSS_SIZE/2, 0, "#FFFFFF", FIELD_LINES_WIDTH, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

    //Left goal post
        //Net
        drawRectangle(ctx, -X_POS_SIDE_LINE - GOAL_POST_WIDTH, -GOAL_POST_HEIGHT/2, GOAL_POST_WIDTH, GOAL_POST_HEIGHT, "#CCCCCC", "#FFFFFF", 1, [FIELD_WIDTH/2, FIELD_HEIGHT/2])
        drawLine(ctx, -X_POS_SIDE_LINE - GOAL_POST_WIDTH, -GOAL_POST_HEIGHT/2, -X_POS_SIDE_LINE, -GOAL_POST_HEIGHT/2, "#FFFFFF", GOAL_POST_RADIUS/2, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        drawLine(ctx, -X_POS_SIDE_LINE - GOAL_POST_WIDTH, GOAL_POST_HEIGHT/2, -X_POS_SIDE_LINE, GOAL_POST_HEIGHT/2, "#FFFFFF", GOAL_POST_RADIUS/2, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        drawLine(ctx, -X_POS_SIDE_LINE - GOAL_POST_WIDTH, -GOAL_POST_HEIGHT/2, -X_POS_SIDE_LINE - GOAL_POST_WIDTH, GOAL_POST_HEIGHT/2, "#FFFFFF", GOAL_POST_RADIUS/2, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        
        //Front post
        drawCircle(ctx, -X_POS_SIDE_LINE, -GOAL_POST_HEIGHT/2 - GOAL_POST_RADIUS/2, GOAL_POST_RADIUS, "#FFFFFF", "FFFFFF", 5, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        drawCircle(ctx, -X_POS_SIDE_LINE, GOAL_POST_HEIGHT/2 + GOAL_POST_RADIUS/2, GOAL_POST_RADIUS, "#FFFFFF", "FFFFFF", 5, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        drawLine(ctx, -X_POS_SIDE_LINE, -GOAL_POST_HEIGHT/2, -X_POS_SIDE_LINE, GOAL_POST_HEIGHT/2, "#FFFFFF", GOAL_POST_RADIUS, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

    //Right goal post
        //Net
        drawRectangle(ctx, X_POS_SIDE_LINE, -GOAL_POST_HEIGHT/2, GOAL_POST_WIDTH, GOAL_POST_HEIGHT, "#CCCCCC", "#FFFFFF", 1, [FIELD_WIDTH/2, FIELD_HEIGHT/2])
        drawLine(ctx, X_POS_SIDE_LINE, -GOAL_POST_HEIGHT/2, X_POS_SIDE_LINE + GOAL_POST_WIDTH, -GOAL_POST_HEIGHT/2, "#FFFFFF", GOAL_POST_RADIUS/2, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        drawLine(ctx, X_POS_SIDE_LINE, GOAL_POST_HEIGHT/2, X_POS_SIDE_LINE + GOAL_POST_WIDTH, GOAL_POST_HEIGHT/2, "#FFFFFF", GOAL_POST_RADIUS/2, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        drawLine(ctx, X_POS_SIDE_LINE + GOAL_POST_WIDTH, -GOAL_POST_HEIGHT/2, X_POS_SIDE_LINE + GOAL_POST_WIDTH, GOAL_POST_HEIGHT/2, "#FFFFFF", GOAL_POST_RADIUS/2, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        
        //Front post
        drawCircle(ctx, X_POS_SIDE_LINE, -GOAL_POST_HEIGHT/2 - GOAL_POST_RADIUS/2, GOAL_POST_RADIUS, "#FFFFFF", "FFFFFF", 1, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        drawCircle(ctx, X_POS_SIDE_LINE, GOAL_POST_HEIGHT/2 + GOAL_POST_RADIUS/2, GOAL_POST_RADIUS, "#FFFFFF", "FFFFFF", 1, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
        drawLine(ctx, X_POS_SIDE_LINE, -GOAL_POST_HEIGHT/2, X_POS_SIDE_LINE, GOAL_POST_HEIGHT/2, "#FFFFFF", GOAL_POST_RADIUS, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);

}


function scaleFieldPositionToCanvas(canvas, xFieldPos, yFieldPos)
{
    var boundingRect = canvas.getBoundingClientRect();

    //The canvas bitmap has a different size wrt the actual canvas size, so we need to scale it
    var scaleX = canvas.width / boundingRect.width;    
    var scaleY = canvas.height / boundingRect.height;  

    var unscaledMouseXOnCanvas = Math.round(Utils.mapValue(xFieldPos, 0, canvas.width, -Math.floor(FIELD_WIDTH/2), Math.floor(FIELD_WIDTH/2)));
    var unscaledMouseYOnCanvas = Math.round(Utils.mapValue(yFieldPos, 0, canvas.height, -Math.floor(FIELD_HEIGHT/2), Math.floor(FIELD_HEIGHT/2)));

    var mouseXOnCanvas = Math.round((unscaledMouseXOnCanvas - boundingRect.left) * scaleX);  
    var mouseYOnCanvas = Math.round((unscaledMouseYOnCanvas - boundingRect.top) * scaleY);     

    return [mouseXOnCanvas, mouseYOnCanvas]

}

function scaleMousePositionToField(canvas, xPos, yPos)
{
    var boundingRect = canvas.getBoundingClientRect();

    //The canvas bitmap has a different size wrt the actual canvas size, so we need to scale it
    var scaleX = canvas.width / boundingRect.width;    
    var scaleY = canvas.height / boundingRect.height;  

    var mouseXOnCanvas = Math.round((xPos - boundingRect.left) * scaleX);  
    var mouseYOnCanvas = Math.round((yPos - boundingRect.top) * scaleY);     

    var scaledMouseXOnCanvas = Math.round(Utils.mapValue(mouseXOnCanvas, -Math.floor(FIELD_WIDTH/2), Math.floor(FIELD_WIDTH/2), 0, canvas.width));
    var scaledMouseYOnCanvas = Math.round(Utils.mapValue(mouseYOnCanvas, -Math.floor(FIELD_HEIGHT/2), Math.floor(FIELD_HEIGHT/2), 0, canvas.height));

    return [scaledMouseXOnCanvas, scaledMouseYOnCanvas]
}

function viewportMouseToCanvasCoordinates(canvas, xPos, yPos)
{
    var boundingRect = canvas.getBoundingClientRect();
    var scaleX = canvas.width / boundingRect.width;    
    var scaleY = canvas.height / boundingRect.height;  
    return [(xPos - boundingRect.left) * scaleX, (yPos - boundingRect.top) * scaleY];
}

function drawTargetOnField(canvas, debugInfo=false, scaledTargetPos = undefined)
{
    if(!mouseOverCanvas) return
    var ctx = canvas.getContext('2d');
    var unscaledTarget;
    var scaledTarget;
    if(scaledTargetPos == undefined)
    {
        unscaledTarget = canvas.unscaledMousePositionOnCanvas;
        scaledTarget = canvas.scaledMousePositionOnCanvas;    
    }
    else
    {
//TODO perform correct assertions scaledTargetPos should be a list of length 2
        unscaledTarget = scaleFieldPositionToCanvas(canvas, scaledTargetPos[0], scaledTargetPos[1])
        scaledTarget = scaledTargetPos;
    }

    if(debugInfo)
    {
        ctx.fillStyle = canvasTextColor;
        ctx.font = "180px Arial";
        ctx.fillText("X: "+scaledTarget[0]+", Y: "+scaledTarget[1], 280, 300);
    }

    drawCircle(ctx, unscaledTarget[0], unscaledTarget[1], TARGET_RADIUS, canvasBackgroundColor, "#FFFFFF", 20);
    drawLine(ctx, unscaledTarget[0], unscaledTarget[1]+TARGET_RADIUS*3/2, unscaledTarget[0], unscaledTarget[1]-TARGET_RADIUS*3/2, "#FFFFFF", 20)
    drawLine(ctx, unscaledTarget[0]+TARGET_RADIUS*3/2, unscaledTarget[1], unscaledTarget[0]-TARGET_RADIUS*3/2, unscaledTarget[1], "#FFFFFF", 20)

}

function drawRobot(ctx, robotNumber, angle, xPos, yPos, isActiveRobot = false)
{
    var fillColor = "#AAAAAA";
    var lineColor = "#000000";

    if(isActiveRobot)
    {
        fillColor = "#FFFF00";
        lineColor = "#FF0000";
    }

    drawCircle(ctx, xPos, yPos, ROBOT_RADIUS, fillColor, lineColor, 20, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
    drawArrow(ctx, xPos, yPos, xPos+ROBOT_RADIUS*2*Math.cos(angle), yPos-ROBOT_RADIUS*2*Math.sin(angle), lineColor, 20, 50, [FIELD_WIDTH/2, FIELD_HEIGHT/2]);
}

function drawBall(ctx)
{
    drawCircle(ctx, ballPosition[0], ballPosition[1], BALL_RADIUS, "#FFFFFF", "#000000", 20, [FIELD_WIDTH/2, FIELD_HEIGHT/2])
}

function drawObstacles(ctx)
{
    for(var obs of obstaclesPositions)
    {
        drawCircle(ctx, obs[0], obs[1], OBSTACLE_RADIUS, "#0000FF", "#000000", 20, [FIELD_WIDTH/2, FIELD_HEIGHT/2])
    }
}

function drawObjects(canvas)
{
    var ctx = canvas.getContext("2d")
    
    //Draw the ball
    drawBall(ctx)
    //Draw the active robots
    for( const [robotNumber, robotPosition] of Object.entries(robotNumbersToPositions)) 
    {
        drawRobot(ctx, robotNumber, robotPosition[0], robotPosition[1], robotPosition[2], robotNumber == ACTIVE_ROBOT_NUMBER)
    };

    //Draw the obstacles
    drawObstacles(ctx)
}

function drawCanvas()
{
    var canvas = document.getElementById("field-canvas");
    //Draw the static field
    drawField(canvas, FIELD_WIDTH, FIELD_HEIGHT);
    
    //Draw the target for the currently selected task button (if there is one)
    if(canvas.currentlySelectedTaskButton != undefined && canvas.currentlySelectedTaskButton.selectionMode != "noSelection")
        drawTargetOnField(canvas, true)    
    
    //Draw something for each task (a target and or other info)

    //Draw the ball, the robot positions and the obstacle positions
    drawObjects(canvas)
}


function startRenderingLoop()
{
    if(!CLIENT_ENABLED) return;

    console.log("START RENDER")
    var lastRender = 0
    
    function renderingLoop(timestamp)
    {
        if(!CLIENT_ENABLED) return
        
        var progress = timestamp - lastRender
        
        drawCanvas()

        lastRender = timestamp
        window.requestAnimationFrame(renderingLoop)
    }

    window.requestAnimationFrame(renderingLoop)
}

function sendNewTask(selectedRobot, taskType, selectionMode, xPos=undefined, yPos=undefined)
{
    if(selectionMode === "noSelection")
    {
        sendToWebSocket("taskType:"+selectedRobot+","+selectionMode+","+taskType+","+(lastReceivedTaskID+1));
    }
    else
    {
        sendToWebSocket("taskType:"+selectedRobot+","+selectionMode+","+taskType+","+(lastReceivedTaskID+1)+","+xPos+","+-yPos);
    }
}



//--------------
//| WEBSOCKET  |
//--------------


//Taken from https://www.codegrepper.com/code-examples/javascript/javascript+generate+unique+hash
var CLIENT_ID = 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
    var r = Math.random() * 16 | 0, v = c == 'x' ? r : (r & 0x3 | 0x8);
    return v.toString(16);
});

function generateHeader()
{
    return "TOSERVER!clientID,"+CLIENT_ID+";robotNumber,"+ACTIVE_ROBOT_NUMBER+"|";
}

function sendToWebSocket(message)
{
    console.log(webSocket.readyState)
    if(webSocket.readyState === WebSocket.OPEN)
    {
        console.log("Sending message: "+message)
        webSocket.send(generateHeader() + message);
    }
}

function disableClient(reason)
{
    CLIENT_ENABLED = false;
    clearInterval(keepalive_send_timeout);
    clearInterval(keepalive_receive_timeout);
    console.log("Disabling client (reason: "+reason+")")

    document.getElementById("body").style.visibility = "hidden";
    document.getElementById("tasks-tab").innerHTML = "";
    currentTaskList = [];
    currentSocket = undefined;

    /*if(!CLIENT_ENABLED) 
    {
        var webSocket = new WebSocket("ws://localhost:4000");
        setupSocket(webSocket)
        waitForWebSocketConnection(webSocket);
    }*/
}

function sendTask(taskType, robotNumber) {}

function selectTaskButton(button) {

    var canvas = document.getElementById("field-canvas");
    if(canvas.currentlySelectedTaskButton == button)
    {
        canvas.currentlySelectedTaskButton = undefined;
        console.log("Not task button selected")
    }
    else 
    {
        canvas.currentlySelectedTaskButton = button;
        console.log(document.getElementById("field-canvas").currentlySelectedTaskButton.selectionMode);
    }
}

function createTaskAssignmentButton(taskLabel, taskType, selectionMode) {

    var outerDiv = document.createElement("DIV");
    outerDiv.classList.add("settings-horizontal-container")
    outerDiv.style.height = "18%";

    var middleDiv = document.createElement("DIV");  
    middleDiv.classList.add("settings-horizontal-container")  
    middleDiv.style.height = "80%";
    outerDiv.appendChild(middleDiv);
    
    var innerDiv = document.createElement("DIV");   
    innerDiv.classList.add("settings-horizontal-container")        
    innerDiv.style.height = "100%";     
    innerDiv.style.justifyContent = "space-around"; 
    middleDiv.appendChild(innerDiv);
              

    var btn = document.createElement("BUTTON");
    btn.classList.add("settings-horizontal-container")
    btn.innerHTML = taskLabel;
    btn.taskType = taskType;
    btn.selectionMode = selectionMode;

    btn.onmousedown = function(e){
        var canvas = document.getElementById("field-canvas");
        if(selectionMode === "noSelection")
        {
            //Highlight the button for 0.2 seconds after press to give visual feedback for the button press
            toggleButton(e.target)
            setTimeout(function () {toggleButton(e.target)}, 200)

            sendNewTask(ACTIVE_ROBOT_NUMBER, btn.taskType, "noSelection");         
        }
        else
        {
            toggleButton(e.target);
            selectTaskButton(e.target);
        }
    };

    innerDiv.appendChild(btn);               // Append the button to the inner DIV as a child

    var tasksTab = document.getElementById("tasks-tab");
    tasksTab.appendChild(outerDiv);               // Append the outerDiv to the settings tab as a child

}

function scheduleTaskReset()
{
    sendToWebSocket("resetTasks,"+ACTIVE_ROBOT_NUMBER);
}

function createResetTasksButton() {

    var outerDiv = document.createElement("DIV");
    outerDiv.classList.add("settings-horizontal-container")
    outerDiv.style.height = "18%";

    var middleDiv = document.createElement("DIV");  
    middleDiv.classList.add("settings-horizontal-container")  
    middleDiv.style.height = "80%";
    outerDiv.appendChild(middleDiv);
    
    var innerDiv = document.createElement("DIV");   
    innerDiv.classList.add("settings-horizontal-container")        
    innerDiv.style.height = "100%";     
    innerDiv.style.justifyContent = "space-around"; 
    middleDiv.appendChild(innerDiv);
              

    var btn = document.createElement("BUTTON");
    btn.id = "resetTasksButton"
    btn.innerHTML = "Reset tasks";

    btn.onmousedown = function(e){
        var canvas = document.getElementById("field-canvas");

        //Highlight the button for 0.2 seconds after press to give visual feedback for the button press
        toggleButton(e.target)
        setTimeout(function () {toggleButton(e.target)}, 200)

        scheduleTaskReset();
    };

    innerDiv.appendChild(btn);               // Append the button to the inner DIV as a child

    var tasksTab = document.getElementById("tasks-tab");
    tasksTab.appendChild(outerDiv);               // Append the outerDiv to the settings tab as a child
}

function createTaskPreview(taskLabel, taskID, position=undefined) {

    var div = document.createElement("DIV");
    div.classList.add("taskPreview");
    div.id = "task-"+taskID;
    div.taskID = taskID

    div.innerHTML = taskLabel

    if(position!=undefined) div.innerHTML+="\n("+position[0]+", "+position[1]+")";

    var tasksPreview = document.getElementById("tasks-preview");
    tasksPreview.appendChild(div);               // Append the outerDiv to the settings tab as a child
    currentTaskPreviews.push(div)
}

function updateTaskPreviews(lastCompletedTaskID)
{
    var tasksPreviewDiv = document.getElementById("tasks-preview");
    tasksPreviewDiv.innerHTML = ""

    console.log(currentTaskPreviews)
    for(var taskPreview of currentTaskPreviews)
    {
        if(taskPreview.taskID > lastCompletedTaskID)
        {  
            tasksPreviewDiv.appendChild(taskPreview)
        }
    }
}

function enableClient()
{
    CLIENT_ENABLED = true;
    //The following initialization steps follow the numbering on the report
    document.getElementById("body").style.visibility = "visible";
    /*
    1) Canvas initialization
    */
    var canvasContainer = document.getElementById( "canvas-container" );
    canvasContainer.style.backgroundColor = canvasContainerBackgroundColor
    canvas = document.getElementById( "field-canvas" );
    canvas.width = FIELD_WIDTH;
    canvas.height = FIELD_HEIGHT;
    canvas.unscaledMousePositionOnCanvas = [FIELD_WIDTH/2, FIELD_HEIGHT/2];
    canvas.scaledMousePositionOnCanvas = [FIELD_WIDTH/2, FIELD_HEIGHT/2];

    canvas.addEventListener("mousemove", function(evt) 
    {   
        mouseOverCanvas = true;
        evt.target.unscaledMousePositionOnCanvas = viewportMouseToCanvasCoordinates(evt.target, evt.clientX, evt.clientY);
        evt.target.scaledMousePositionOnCanvas = scaleMousePositionToField(evt.target, evt.clientX, evt.clientY)
    });

    canvas.addEventListener("mouseleave", function(evt) 
    {   
        mouseOverCanvas = false;
    });

    canvas.addEventListener("mousedown", function(evt) 
    {   
        //console.log("click")
        var clickPos = scaleMousePositionToField(evt.target, evt.clientX, evt.clientY);
        if(evt.target.currentlySelectedTaskButton != undefined)
        {
            
            //Utils.assert(evt.target.ACTIVE_ROBOT_NUMBER != undefined, "Inconsistent situation: not having a robot selected should make the task buttons unclickable");
            
            sendNewTask(ACTIVE_ROBOT_NUMBER, evt.target.currentlySelectedTaskButton.taskType, evt.target.currentlySelectedTaskButton.selectionMode, clickPos[0], clickPos[1]);         
            toggleButton(evt.target.currentlySelectedTaskButton);
            evt.target.currentlySelectedTaskButton = undefined;
        }
    });

    canvas.currentlySelectedTaskButton = undefined;

    createTaskAssignmentButton("Go to position", "GoToPosition", "singlePosition")
    createTaskAssignmentButton("Kick to position", "KickBallToPosition", "singlePosition")
    createTaskAssignmentButton("Carry ball to position", "CarryBallToPosition", "singlePosition")
    createTaskAssignmentButton("Score a goal", "ScoreGoalTask", "noSelection")
    createResetTasksButton();

}

var KEEPALIVE_SEND_INTERVAL = 5000;
var KEEPALIVE_RECEIVE_INTERVAL = 5000;
var CLIENT_INFO_INTERVAL = 1000;
var RETRY_CONNECTION_TIMEOUT = 2000;

var currentSocket;

var keepalive_receive_timeout;
var keepalive_send_timeout;

function checkSocketStillOpen()
{
    if(currentSocket == undefined || currentSocket.readyState == WebSocket.CLOSED || currentSocket.readyState == WebSocket.CLOSING)
    {
        return false;
    }
    return true;
}

function requestKeepalive() {
    
    if(!CLIENT_ENABLED)
    {
        clearInterval(keepalive_send_timeout);
        clearInterval(keepalive_receive_timeout);
    }

    if(!checkSocketStillOpen())
    {
        disableClient("Socket CLOSED");
        return;
    }

    //console.log("Requesting keepalive")
    currentSocket.send(generateHeader() + 'uthere?');

    //Set a timeout to be cleared in case a keepalive is received
    keepalive_receive_timeout = setTimeout(function () {
        //If this is executed, the server failed to respond to the keepalive
        disableClient("Keepalive not received");
    }, KEEPALIVE_RECEIVE_INTERVAL);
}

function sendClientInfo()
{
    currentSocket.send(generateHeader())
}

function setupSocket(webSocket)
{
    webSocket.onopen = function () {
        // connection is opened and ready to use
        
        currentSocket = webSocket;

        //console.log("WebSocket connected to NodeJS server")
        //console.log(webSocket)
        sendClientInfo(webSocket);

        //Start rendering the canvas
        //enableClient();

        //Set up a ping pong method for keepalive
        //console.log("Setting up keepalive")
        keepalive_send_timeout = setInterval(requestKeepalive, KEEPALIVE_SEND_INTERVAL);

        //startRenderingLoop();
    };

    webSocket.onerror = function (error) {
        // an error occurred when sending/receiving data
    };

    function resetTaskList()
    {
        currentTaskList = [];
        currentTaskPreviews = [];
        document.getElementById("tasks-preview").innerHTML = ""
    }


    function addTask(taskType, taskID, parameters = undefined)
    {
        createTaskPreview(taskType, taskID, parameters)
        if(parameters == undefined)
        {
            currentTaskList.push({taskType : taskType, taskID : taskID})
        }
        else
        {
            currentTaskList.push({taskType : taskType, taskID : taskID, parameters : parameters})
        }
    }

    webSocket.onmessage = function (message) {
        // handle incoming message
        //console.log(message)
        //Avoid reading your own messages
        if(message.data.startsWith("TOSERVER")) return;
        
        var message_content = message.data.toString().split("!")[1]
        //console.log(message_content)

        if(message_content == 'yeah')
        {
            //Keepalive received, stop waiting
            //console.log("Keepalive received!")
            clearTimeout(keepalive_receive_timeout);
        }
        else if(message_content.startsWith('robotNotResponding'))
        {
            console.log("Robot not responding. Disabling client!")
            disableClient("Robot not responding")
        }
        else
        {
            if(!CLIENT_ENABLED)
            {
                enableClient()
                startRenderingLoop()
            }

            if(message_content.startsWith("robotPosition"))
            {
                message_content = message_content.split(":")[1]
                var message_fields = message_content.split(",")
                var robotNumber = message_fields[0]

                //NOTICE: the y coordinate is inverted
                robotNumbersToPositions[robotNumber] = [Math.floor(parseFloat(message_fields[1])), Math.floor(parseFloat(message_fields[2])), -Math.floor(parseFloat(message_fields[3]))]
            }
            else if(message_content.startsWith("ballPosition"))
            {
                message_content = message_content.split(":")[1]
                var message_fields = message_content.split(",")

                //NOTICE: the y coordinate is inverted
                ballPosition = [Math.floor(parseFloat(message_fields[0])), -Math.floor(parseFloat(message_fields[1]))]
            }
            else if(message_content.startsWith("obstacles"))
            {
                var message_fields = message_content.split(":")[1].split(";")
                obstaclesPositions = []
                for(var field of message_fields)
                {
                    var obsCoords = field.split(",")
                    
                    //NOTICE: the y coordinate is inverted
                    obstaclesPositions.push([Math.floor(parseFloat(obsCoords[0])), -Math.floor(parseFloat(obsCoords[1]))])
                }                
            }
            else if(message_content.startsWith("taskQueue"))
            {
                resetTaskList();

                //console.log(message_content)
                var content_fields = message_content.split(";").slice(1)
                
                //console.log(content_fields)
                lastReceivedTaskID = parseInt(content_fields[0].split(",")[1])
                lastCompletedTaskID = parseInt(content_fields[0].split(",")[2])
                //console.log(lastReceivedTaskID)
                //console.log(lastCompletedTaskID)
                
                //If there isn't any task return
                if(content_fields.length == 1) return;
        
                //console.log(content_fields)
                for(var task of content_fields.slice(1))
                {
                    var task_fields = task.split(",")
                    
                    var taskType = task_fields[0]
                    var taskID = parseInt(task_fields[1])
        
                    if(task_fields.length==2)
                    {
                        addTask(taskType, taskID)
                    }
                    else if(task_fields.length==4)
                    {
                        var xPos = Math.floor(parseFloat(task_fields[2]))
                        var yPos = Math.floor(parseFloat(task_fields[3]))
                        addTask(taskType, taskID, [xPos, yPos])
                    }
                    else
                    {
                        console.log("Wrong taskQueue format")
                        return;
                    }
                }
            }
        }
    };
}

//Taken from https://stackoverflow.com/questions/13546424/how-to-wait-for-a-websockets-readystate-to-change
function waitForWebSocketConnection(webSocket)
{
    setTimeout(
        function () {
            if (webSocket.readyState == WebSocket.OPEN) {
            } else {
                //console.log("Attempting connection of websocket...")
                webSocket = new WebSocket("ws://localhost:4000");
                setupSocket(webSocket)
                waitForWebSocketConnection(webSocket);
            }

        }, RETRY_CONNECTION_TIMEOUT); // wait 5 milisecond for the connection...
}


var CLIENT_ENABLED = true;
disableClient("Waiting for socket connection");
var webSocket = new WebSocket("ws://localhost:4000");
setupSocket(webSocket)
waitForWebSocketConnection(webSocket);


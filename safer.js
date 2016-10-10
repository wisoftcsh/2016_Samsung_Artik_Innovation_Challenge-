//개발에 필요한 nodejs 모듈 추출
var express = require('express');
var http = require('http');

var path = require('path');
var favicon = require('serve-favicon');
var logger = require('morgan');
var cookieParser = require('cookie-parser');
var bodyParser = require('body-parser');

var routes = require('./routes/index');
var users = require('./routes/users');
var app = express();

var WebSocket = require('ws');
var isWebSocketReady = false;
var ws = null;

var puts = require('sys').puts;

//ARTIK클라우드와의 연동
var webSocketUrl = "wss://api.artik.cloud/v1.1/websocket?ack=true";
var device_id = "e450c3a81f3f4878b1d2327d8f327c89";
var device_token = "b8632eccf6f94f2284e0607aec03e592";

setInterval(function() {
    sendSensorValueToArtikCloud();
}, 2000);

function getTimeMillis(){
    return parseInt(Date.now().toString());
}

function start() {
    //Create the WebSocket connection
    isWebSocketReady = false;
    ws = new WebSocket(webSocketUrl);
    ws.on('open', function() {
        console.log("WebSocket connection is open ....");
        register();
    });
    ws.on('message', function(data) {
       console.log("Received message: " + data + '\n');
         handleRcvMsg(data);
    });
    ws.on('close', function() {
        console.log("WebSocket connection is closed ....");
    //exitClosePins();
    });
}

function register(){
    console.log("Registering device on the WebSocket connection");
    try{
        var registerMessage = 
           '{"type":"register", "sdid":"'+device_id+'", "Authorization":"bearer '+device_token+'", "cid":"'+getTimeMillis()+'"}';
        console.log('Sending register message ' + registerMessage + '\n');
        ws.send(registerMessage, {mask: true});
        isWebSocketReady = true;
    }
    catch (e) {
        console.error('Failed to register messages. Error in registering message: ' 
        + e.toString());
    }    
}

function handleRcvMsg(msg){
    var msgObj = JSON.parse(msg);
    if (msgObj.type != "action") return; //Early return;

    var actions = msgObj.data.actions;
    var actionName = actions[0].name; 
    console.log("The received action is " + actionName);
    var newState;
    if (actionName.toLowerCase() == "seton") { newState = 1; }
    else if (actionName.toLowerCase() == "setoff") { newState = 0; } 
    else {
        console.log('Do nothing since receiving unrecognized action ' + actionName);
        return;
    }
    console.log('toggled');
    sendStateToArtikCloud();
}

function sendSensorValueToArtikCloud(){
    try{
        ts = ', "ts": '+getTimeMillis();
        var data = {
              "latitude": latitude, "longitude": longitude, "temperature": temperature
            };
        
        var payload = '{"sdid":"'+device_id+'"'+ts+', "data": '+JSON.stringify(data)+', "cid":"'+getTimeMillis()+'"}';
                
        console.log('Sending payload ' + payload + '\n');
        ws.send(payload, {mask: true});
    } catch (e) {
        console.error('Error in sending a message: ' + e.toString() +'\n');
    }    
}

function exitClosePins() {
    console.log('Exit and destroy all pins!');
    process.exit();
}

start();
////////////////////

///////////////serial port
var serialport = require("serialport");
var gpsPortName = '/dev/ttyACM1';
var temPortName = '/dev/ttyACM0';

var spGPS = new serialport(gpsPortName, {
    baudRate: 115200,
    dataBits: 8,
    parity: 'none',
    stopBits: 1,
    flowControl: false
});

var spTemp = new serialport(temPortName, {
    baudRate: 115200,
    dataBits: 8,
    parity: 'none',
    stopBits: 1,
    flowControl: false
});
///////////////

// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'jade');

// uncomment after placing your favicon in /public
//app.use(favicon(path.join(__dirname, 'public', 'favicon.ico')));
app.use(logger('dev'));
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({extended: false}));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, 'public')));

app.use('/', routes);
app.use('/users', users);

// catch 404 and forward to error handler
app.use(function (req, res, next) {
    var err = new Error('Not Found');
    err.status = 404;
    next(err);
});

// error handlers

// development error handler
// will print stacktrace
if (app.get('env') === 'development') {
    app.use(function (err, req, res, next) {
        res.status(err.status || 500);
        res.render('error', {
            message: err.message,
            error: err
        });
    });
}

// production error handler
// no stacktraces leaked to user
app.use(function (err, req, res, next) {
    res.status(err.status || 500);
    res.render('error', {
        message: err.message,
        error: {}
    });
});

var server = http.createServer(app).listen(3000, function () {
    console.log("Express server listening on port " + 3000);
});

var io = require('socket.io').listen(server);

io.sockets.on('connection', function (socket) {
    console.log("Socket connected");
    socket.emit('connected', 123); //emit() 소켓 이벤트를 발생시킨다. 'connected' -> index.html응답.
});

spGPS.on('open', function () {
    console.log('GPS_open');
});

spTemp.on('open', function () {
    console.log('Temp_open');
});


//아틱으로 적정시간 마다 전송하기위한 전역변수 선언
var longitude, latitude;
//gps센서값 센싱 후 가공.
spGPS.on('data', function (data) {
    var time, date, condition;
    var mbRec = new Buffer(data, 'utf');
    var string = mbRec.toString('ascii').trim();
    var index = string.indexOf("$GPRMC");

    if (index == 0) {
        string = string.substring(0, string.indexOf(",,,") + 3);
        var idx = string.lastIndexOf(",,,");
        var errorFlag = string.substring(17, 18);
        if (errorFlag == 'A') {
            //time
            time = string.substring(7, 16);
            var hh = time.substring(0, 2) * 1 + 9;
            if (hh > 24) {
                hh = hh - 24;
            }
            var mm = time.substring(2, 4);
            var ss = time.substring(4, 9);
            //date
            date = string.substring(idx - 6, idx);
            var y = 20 + date.substring(4, 6);
            var m = date.substring(2, 4);
            var d = date.substring(0, 2);
            date = y + "년 " + m + "월 " + d + "일 " + hh + "시 " + mm + "분 " + ss + "초";
            //latitude(위도)
            latitude = "" + string.substring(string.indexOf('N') - 11, string.indexOf('N') - 1);
            //longitude(경도)
            longitude = "" + string.substring(string.indexOf('N') + 2, string.indexOf('E') - 1);
            console.log(string);
            console.log(date + latitude + longitude);
            io.sockets.emit('gps_latitude', latitude);
            io.sockets.emit('gps_longitude', longitude);
            string = "";
        } else {
            console.log("GPS ERROR!");
            io.sockets.emit('gps_latitude', "GPS ERROR!");
            io.sockets.emit('gps_longitude', "GPS ERROR!");
            string = "";
        }
    }
});

String.prototype.startsWith = function (prefix) {
    return this.indexOf(prefix) === 0;
}

String.prototype.endsWith = function (suffix) {
    return this.match(suffix + "$") == suffix;
};


//아틱으로 적정시간 마다 전송하기위한 전역변수 선언.
var temperature;
//방수온습도 센서와 모션트레킹센서 z,x,y축 value 측정.
spTemp.on('data', function (data) {
    var mbRec = new Buffer(data, 'utf');
    var temp = mbRec.toString('ascii').trim('\n');
    if (temp.startsWith("T") && temp.endsWith("!")) {
        temperature = temp.substring(temp.indexOf('T') + 1, temp.indexOf('!'));
        temperature.trim();
        console.log(temperature);
        io.sockets.emit('temp', temperature);
        string = "";
    }
    if(temp.startsWith('Z') && temp.endsWith('@')) {
        var z = temp.substring(temp.indexOf('Z') + 1, temp.indexOf('X')).trim();
        var x = temp.substring(temp.indexOf('X') + 1, temp.indexOf('Y')).trim();
        var y = temp.substring(temp.indexOf('Y') + 1, temp.indexOf('@')).trim();
        console.log(z);
        console.log(x);
        console.log(y);
        io.sockets.emit('tracking_z', z);
        io.sockets.emit('tracking_x', x);
        io.sockets.emit('tracking_y', y);
    }   
});

module.exports = app;
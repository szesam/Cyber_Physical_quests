//Create UDP datagram and server
var dgram = require('dgram');
var server = dgram.createSocket('udp4');
var app = require('express')();
var http = require('http').Server(app);
// initialize data structures to store sensor data
var voltage =[], temp = [], xaccel = [], yaccel = [], zaccel = [], roll = [], pitch = [];
//create stream to write sensor data into csv file called out.csv in same directory.
var fs = require('fs')
var stream = fs.createWriteStream("out.csv");
stream.write("Voltage(mV), Thermistor(C), Ultrasonic(m), IR(m)\n")

//Create acknowledge packet to send back to udp_client upon message received
const msg = Buffer.from('Acknowledged');

//When opening the udp socket. 
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

//when receiving a packet from udp_client (esp32)
server.on('message', function (message, remote) {
        console.log(remote.address + ':' + remote.port +' - ' + message.toString());
        server.send(msg, remote.port, remote.address);
        push_sensor_data(message);
});
// i is x-axis
var i = 0; 
function push_sensor_data(data)
{
    data = data.toString();
    data = data.split(",");
    voltage = data[0];
    i = i + 0.5;
}
//Bind server to PI local address on router with port at 3333
server.bind(3333,'192.168.1.136');

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// socket io script connect server to client (web browser) to display canvasjs charts.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// declare socket io for http
var io = require('socket.io')(http);

//interface with index.html directory to plot chart
app.get('/', function(req, res){
    res.sendFile(__dirname + '/index.html');
});

// emit data every 1second
setInterval(function(){
	io.emit('dataMsg3',	voltage);
 }, 1000);


//Socket io on user connection
io.on('connection', function(socket){
	console.log('a user connected');
	io.emit('dataMsg3', voltage);
});

// // keep channel open on ipaddress:3334
http.listen(3334, function(){
});
// keep channel open.
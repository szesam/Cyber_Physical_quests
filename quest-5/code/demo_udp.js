/*Carmen Hurtado and Samuel Sze 03-05-2021 
EC444 Quest 3: Hurricane Box 
Code adapted from udp server example from node js */

//Create UDP datagram and server
var dgram = require('dgram');
var server = dgram.createSocket('udp4');

var app = require('express')();

var http = require('http').Server(app);

// initialize data structures to store sensor data
var  speed = [], distance = [], alert = [];

//Create acknowledge packet to send back to udp_client upon message received
const msg = Buffer.from('Acknowledged');

//When opening the udp socket. 
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

//when receiving a packet from udp_client (esp32)
var remote_address; //store remote client's address to send additional message data
var remote_port; //store remote client's port to send additional message data
server.on('message', function (message, remote) {
        console.log(remote.address + ':' + remote.port +' - ' + message.toString());
        remote_address = remote.address;
        remote_port = remote.port;
        server.send(msg, remote.port, remote.address);
        push_sensor_data(message);
});

// i is x-axis
var i = 0; 
function push_sensor_data(data)
{
    data = data.toString();
	if(data == 'Alert') {
		alert = data;
	}
	else{
		data = data.split(",");
		speed = data[2];
		distance = data[3];
	}

}
//Bind server to PI local address on router with port at 3333
server.bind(3333,'192.168.1.126');

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
	// io.emit('dataMsg1', chartOptions1); //xaccel
	// io.emit('dataMsg2', chartOptions2); //yaccel
	io.emit('dataMsg3',	speed); //speed
	io.emit('dataMsg4',	distance); //distance traveled
	io.emit('dataMsg5',	alert); //when there is an object in the way send alert messge to web server
 }, 1000);

//socket io creation. 

var msg2;//second message to be sent to client to toggle LED 
var msg3;
var msg4;
var msg5;
io.on('connection', function(socket){
	//console.log('a user connected');
	// io.emit('dataMsg1', chartOptions1);
	// io.emit('dataMsg2', chartOptions2);
	io.emit('dataMsg3',	speed);
	io.emit('dataMsg4',	distance);
	io.emit('dataMsg5',	alert);
    
    //send start car message to client 
    socket.on('data1', function (data) {
        msg2 = Buffer.from(data.buttondata);
        server.send(msg2, remote_port, remote_address);
        console.log(msg2);
    });

    //send stop car message to client 
    socket.on('data2', function (data) {
        msg3 = Buffer.from(data.buttondata);
        server.send(msg3, remote_port, remote_address);
        console.log(msg3);
    });

	//send steer right car message to client 
    socket.on('data3', function (data) {
        msg4 = Buffer.from(data.buttondata);
        server.send(msg3, remote_port, remote_address);
        console.log(msg3);
    });

	//send steer left car message to client 
    socket.on('data4', function (data) {
        msg5 = Buffer.from(data.buttondata);
        server.send(msg3, remote_port, remote_address);
        console.log(msg3);
    });
});



// // keep channel open on ipaddress:3334
http.listen(3334, function(){
});
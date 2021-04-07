
//Create UDP datagram and server
var dgram = require('dgram');
var server = dgram.createSocket('udp4');
var app = require('express')();
var http = require('http').Server(app);

//Multicast address
const MULTICAST_ADDR = "232.10.11.12";


//Create acknowledge packet to send back to udp_client upon message received
const msg = Buffer.from('e2');

//When opening the udp socket. 
server.on('listening', function () {
    server.addMembership(MULTICAST_ADDR);
    var address = server.address();
    console.log(
      `UDP socket listening on ${address.address}:${address.port} pid: ${
          process.pid
      }`
    );
});

//when receiving a packet from udp_client (esp32)
var remote_address; //store remote client's address to send additional message data
var remote_port; //store remote client's port to send additional message data
var i = 0;
server.on('message', function (message, remote) {
        console.log(remote.address + ':' + remote.port +' - ' + message.toString());
        remote_address = remote.address;
        remote_port = remote.port;
        // if (i == 0)
        // {
        //   server.send(msg, remote.port, remote.address);
        //   i++;
        // }
});

//Bind server to PI local address on router with port at 3333
server.bind(3333);
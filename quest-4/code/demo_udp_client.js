
//Create UDP datagram and server
var dgram = require('dgram');
var client = dgram.createSocket('udp4');
var app = require('express')();
var http = require('http').Server(app);

//Multicast address
const MULTICAST_ADDR = "232.10.11.12";
client.bind(3333, function () {
    client.addMembership(MULTICAST_ADDR);   // Add the HOST_IP_ADDRESS for reliability
});

//Create acknowledge packet to send back to udp_client upon message received
const msg = Buffer.from('e3');

client.on('listening', function () {
    var address = client.address();
    console.log('UDP Client listening on ' + address.address + ":" + address.port);
});

client.on('message', function (message, rinfo) {
    console.log('Message from: ' + rinfo.address + ':' + rinfo.port + ' - ' + message);
});

//sending msg
client.send(msg,3333,MULTICAST_ADDR,function(error,info){
  if(error){
    client.close();
  }else{
    console.log('sent message %s to %s:%d\n',msg, info.address, info.port);
  }
});
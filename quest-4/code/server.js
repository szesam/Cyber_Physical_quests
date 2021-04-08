/*Carmen Hurtado and Samuel Sze */


//initialize server
var app = require('express')();
var http = require('http').Server(app);

//Create UDP datagram and server
var dgram = require('dgram');
var server = dgram.createSocket('udp4');
var app = require('express')();
var http = require('http').Server(app);

//Multicast address
const MULTICAST_ADDR = "232.10.11.12";


//Create acknowledge packet to send back to udp_client upon message received
const msg = Buffer.from('e2');

//importing the schema for creating the vote objects for the collection
const Vote = require('./Models/vote');
//Database connection
const mongoose = require('mongoose');
const url = "mongodb+srv://carmenhg-node:nodeBUDB444@cluster0.t6u91.mongodb.net/quest4?retryWrites=true&w=majority";
mongoose.connect(url, {useNewUrlParser: true, useUnifiedTopology: true})
    .then((result) => console.log('Connected to database')
        
)

//Listening on server
server.on('listening', function () {
    server.addMembership(MULTICAST_ADDR);
    var address = server.address();
    console.log(
    `UDP socket listening on ${address.address}:${address.port} pid: ${
        process.pid
    }`
    );

})

//save message to database when receiving voting message
server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message.toString());
    //save data to database
    savedata(message.toString());
    //if message contains a vote then fetch data from db 
    if(message.toString()[1] == 'l'){
        if(message.toString()[2] == 'R'){
            retrieveR();
        }
        else if(message.toString()[2] == 'G'){
            retrieveG();
        }
        else if(message.toString()[2] == 'Y'){
            retrieveY();
        } 
    }
});

//Bind server to PI local address on router with port at 3333
server.bind(3333);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// socket io script connect server to client (web browser) to display voting results.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// declare socket io for http
var io = require('socket.io')(http);

//interface with index.html directory to plot chart
app.get('/', function(req, res){
    res.sendFile(__dirname + '/main.html');
});

// emit data every 1second to html 
setInterval(function(){
    io.emit('dataMsg1', votes1);
    io.emit('dataMsg2', votes2);
    io.emit('dataMsg3', votes3);
    io.emit('dataMsg4', total_votes1);
    io.emit('dataMsg5', total_votes2);
    io.emit('dataMsg6', total_votes3);
 }, 1000);

io.on('connection', function(socket){
    io.emit('dataMsg1', votes1);
    io.emit('dataMsg2', votes2);
    io.emit('dataMsg3', votes3);
    io.emit('dataMsg4', total_votes1);
    io.emit('dataMsg5', total_votes2);
    io.emit('dataMsg6', total_votes3);

    //get restart election message and clean the database 
    socket.on('data', function () {
        console.log("Button pushed");
        //clear database
        clear_db();
        //refrsh the page contents 
        votes1 = "";
        votes2 = "";
        votes3 = "";
        total_votes1 = 0;
        total_votes2 = 0;
        total_votes3 = 0;
    });
});


// keep channel open on ipaddress:3334
http.listen(3334, function(){
});


//function to retrieve data from database depending on candidate 
//retrieve data from db

var votes1 = "";
var total_votes1 = 0;
//gets data for candidate 1 
function retrieveR(){
    var i = 0;
    const query1 = { "vote": 'R' };
    //start query from LAST time stamp 
    //need to incluse LAST time stamp as well 
    Vote.find(query1).sort({ createdAt: -1 })
        .then((result) => {
            while(i < Object.keys(result).length){
                votes1 = votes1 + ("Time: " + result[i].createdAt + " FobID: " + result[i].fob_id + ","); 
                i = i + 1;    
            }
        }
    );
    //finding total number of votes
    Vote.find(query1).then((result) => total_votes1 = result.length);
}

var votes2 = "";
var total_votes2 = 0;
//gets data for candidate 2
function retrieveG(){
    var j = 0;
    const query2 = { "vote": 'G' };
    Vote.find(query2).sort({ createdAt: -1 })
        .then((result) => {
            while(j < Object.keys(result).length){
                votes2 = votes2 + ("Time: " + result[j].createdAt + " FobID: " + result[j].fob_id + ","); 
                j = j + 1;    
            }
        }
    );
    //finding total number of votes
    Vote.find(query2).then((result) => total_votes2 = result.length);
}

var votes3 = "";
var total_votes3 = 0;
//gets data for candidate 3
function retrieveY(){
    var h = 0;
    const query3 = { "vote": 'Y' };

    Vote.find(query3).sort({ createdAt: -1 })
        .then((result) => {
            while(h < Object.keys(result).length){
                votes3 = votes3 + ("Time: " + result[h].createdAt + " FobID: " + result[h].fob_id + ",");
                
                h = h + 1;    
            }
        } 
    );
    //finding total number of votes
    Vote.find(query3).then((result) => total_votes3 = result.length);
}
   
//function to save the data 
function savedata(data) {
    if(data[1] == 'l'){
        var vote = new Vote({
            vote: data[2],
            fob_id: data[3],
         })
         vote.save();
    } 
}
   

//function to clean database and restart talling votes 
// DOES NOT WORK 
function clear_db(){
    Vote.deleteMany({}).then(function(){
        console.log("Data deleted, Reelection started"); // Success
    });
}


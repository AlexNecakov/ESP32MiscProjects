const express = require("express");
const bodyParser = require('body-parser');
const app = express();
const dgram = require('dgram');
const server = dgram.createSocket('udp4');
var fs = require('fs')
var rl = require("readline");

var appPort = 6969;
var udpPort = 4444;
var ipTarget = "192.168.1.104"
var portOfESP = 0
var current_speed =0;
var current_Error=0;
var current_Front =0;
var stop = 0;
//web client communication
app.listen(process.env.PORT || appPort, () => {
    console.log(`Application started and Listening on port ${appPort}`);
});

app.use(express.static(__dirname));
app.use(bodyParser.urlencoded({ extended: true }));
app.use(bodyParser.json());
app.use(bodyParser.raw());

app.get("/", (req, res) => {
    res.sendFile(__dirname + "/index.html");
    res.sendFile(__dirname + "/source.js");
    res.sendFile(__dirname + "/styles.css");
});
// function dataParse(){
//   current_speed = Math.random();
//   return [current_speed];
// }
app.get("/data", async (req, res) => {
    res.sendFile(__dirname + "/index.html");
    res.sendFile(__dirname + "/source.js");
    res.sendFile(__dirname + "/styles.css");

    res.end(JSON.stringify([current_speed,current_Error,current_Front]));
});
app.post('/esp', function (req, res) {
   // console.log("got the football",req.body["data"]);
   level = req.body["break"]
   stop = (level)?1:0;

   server.send(JSON.stringify(req.body["data"]), portOfESP, ipTarget, (err) => {
  console.log("no conection")
});

   res.end(JSON.stringify({ret: "Success!"}));
   console.log("Caught the football:  = ",level);
});

//udp communication
server.on('listening', function () {
    var address = server.address();
    console.log(`Server listening ${address.address}:${address.port}`);
    //parseTXT("./testData.txt");
});
server.on('message', function (msg, info) {
    console.log(typeof(msg),msg.length)
    var msgString = msg.toString();
    console.log(msg[0],msg[2],msg[4]);
    current_speed =((msg[0] << 8)| msg[1])/1000;
    current_Error = ((msg[2] << 8)| msg[3]);
    current_Front = ((msg[4] << 8)| msg[5]);
    console.log(current_speed,current_Error,current_Front)
    // response = Buffer.from('g')
    console.log(info.address)
    message = new Buffer(stop);
    console.log(info.port, info.address );
    portOfESP = info.port;
    server.send(JSON.stringify(stop), info.port, info.address , (err) => {
            console.log("no conection")
    });

});
server.on('error', function (error) {
    console.log('Error: ' + error);
    server.close();
});
server.bind(udpPort);

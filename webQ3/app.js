const express = require("express");
const bodyParser = require('body-parser');
const app = express();
// import dgram from 'dgram';
const dgram = require('dgram');
const server = dgram.createSocket('udp4');
const stream = dgram.createSocket('udp4');
var DEBUG = true;
var portUDP = 4444;
var ipTarget = "192.168.0.100"
var portOfESP = 0;
var level = 0;
app.listen(process.env.PORT || 6969, () => {
  console.log("Application started and Listening on port 6969");
});

app.use(express.static(__dirname));
app.use(bodyParser.urlencoded({ extended: true }));
app.use(bodyParser.json());
app.use(bodyParser.raw());

app.get("/", (req, res) => {
  res.sendFile(__dirname + "/index.html");
  res.sendFile(__dirname + "/source.js");
  res.sendFile(__dirname + "/styles.css");
  res.sendFile(__dirname + "/favicon.ico");

});
var twindow = generateRandomOutput();
function moveWindow(x,y,z,t,v){
  var today = new Date();
  var time = today.getMinutes()*60+today.getSeconds();
  if (twindow.length+1<10){
    twindow.push([time,x,y,z,t,v]);
    return;
  }
  twindow.shift();
  twindow.push([time,x,y,z,t,v]);
};

function generateRandomOutput(){
  var fixedtimeWindow =[];
  for (i=0;i<10;i++){
    fixedtimeWindow.push([i,Math.random()*100,Math.random()*100,Math.random()*100,Math.random()*100,Math.random()*100])
  }
  return fixedtimeWindow;
}

app.get("/data", async(req, res) => {
    res.sendFile(__dirname + "/index.html");
    res.sendFile(__dirname + "/source.js");
    res.sendFile(__dirname + "/styles.css");
    vals = generateRandomOutput();
    res.end(JSON.stringify(twindow));
});
app.post('/esp', function (req, res) {
   console.log("got the football",req.body["data"]);
   level = Number(req.body["data"])
   server.send(JSON.stringify(req.body["data"]), portOfESP, ipTarget, (err) => {
  console.log("no conection")
});
   res.end(JSON.stringify({ret: "Success!"}));
});

server.on('error', (err) => {
  console.log(`server error:\n${err.stack}`);
  server.close();
});

function parseMesage(msg){
  console.log(msg)
  vals = msg.split(',')
  return vals
}
server.on('message', (msg, rinfo) => {
  console.log(`server got: ${msg} from ${rinfo.address}:${rinfo.port}`);
  server.send(JSON.stringify(level), rinfo.port, rinfo.address , (err) => {
          console.log("no conection")
  });
  portOfESP = Number(rinfo.port);
  var numbers = parseMesage(msg.toString())//temp,v/x/y/z
  moveWindow(Number(numbers[2]),Number(numbers[3]),Number(numbers[4]),Number(numbers[0]),Number(numbers[1]))
  console.log(numbers)
});

server.on('listening', () => {
  const address = server.address();
  console.log(`server listening ${address.address}:${address.port}`);
});

server.bind(portUDP,"0.0.0.0", () => {
  const address = server.address();
  console.log("bound");
});

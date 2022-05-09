const express = require("express");
const bodyParser = require('body-parser');
const app = express();
var Engine = require('tingodb')(),
    assert = require('assert');
var db = new Engine.Db('./database', {});
var collect = db.collection("collection.js");
const dgram = require('dgram');
const server = dgram.createSocket('udp4');
var fs = require('fs')
var rl = require("readline");

var appPort = 6969;
var udpPort = 4444;
var start = 0x1C;
var redVotes = 0;
var greenVotes = 0;
var blueVotes = 0;
var voteList = [];

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

app.get("/data", async (req, res) => {
    res.sendFile(__dirname + "/index.html");
    res.sendFile(__dirname + "/source.js");
    res.sendFile(__dirname + "/styles.css");
    findNumMatches();
    readTimeStampOrder();
    console.log("Votes: ", redVotes, greenVotes, blueVotes);
    res.end(JSON.stringify([redVotes, greenVotes, blueVotes, voteList]));
});

app.post('/resetdb', function (req, res) {
    console.log("Command code: ", req.body["data"]);
    if (req.body["data"] == 10) {
        console.log("Clearing records");

        collect.remove({}, function (err, obj) {
            if (err) throw err;
        });
    }
});

//udp communication
server.on('listening', function () {
    var address = server.address();
    console.log(`Server listening ${address.address}:${address.port}`);
    //parseTXT("./testData.txt");
});
server.on('message', function (msg, info) {
    console.log('Data received from client : ' + msg);
    //console.log('Received %d bytes from %s:%d\n', msg.length, info.address, info.port);
    console.log(typeof(msg),msg[1])
    var msgString = msg.toString();
    var vals = msgString.split();
    var startbit = msg[0]
    var ogsender = msg[1]
    var last =msg[2]
    var leader = msg[3]
    var votea = msg[4]
    console.log(startbit,ogsender,last,leader,votea)
    if (startbit == start) { //check start byte
        if (last == leader&& votea != 0 ) { //check if we are receiving from leader
            var today = new Date();
            var time = today.getMinutes() * 60 + today.getSeconds();
            var myobj = { fob_id: ogsender, vote: votea, timestamp: time };
            collect.insert(myobj, function (err, res) {
                if (err) throw err;
                console.log("Inserted from msg" + myobj.toString());
            });
        }
    }
    // //sending msg to the client
    // var response = Buffer.from('From server : your msg is received');
    // server.send(response, info.port, "255.255.255.255", function (error) {
    //     if (error) {
    //         client.close();
    //     } else {
    //         console.log('Data sent !');
    //     }
    // });
});
server.on('error', function (error) {
    console.log('Error: ' + error);
    server.close();
});
server.bind(udpPort);

//find num matches of each vote in db
function findNumMatches() {
    var queryRed = { vote: 1 };
    var queryGreen = { vote: 2 };
    var queryBlue = { vote: 3 };
    collect.find(queryRed).toArray(function (err, result) {
        if (err) throw err;
        redVotes = result.length;
    });
    collect.find(queryGreen).toArray(function (err, result) {
        if (err) throw err;
        greenVotes = result.length;
    });
    collect.find(queryBlue).toArray(function (err, result) {
        if (err) throw err;
        blueVotes = result.length;
    });
}

//read out the whole collection in timestamp order
function readTimeStampOrder() {
    var timeStampDesc = { timestamp: -1 };
    collect.find().sort(timeStampDesc).toArray(function (err, result) {
        if (err) throw err;
        voteList = result;
    });
}

function parseTXT(dataPath) { //test on dummy data set
    const reader = rl.createInterface({
        input: fs.createReadStream(dataPath)
    });
    var arr = [];
    reader.on("line", (row) => {
        rowArray = row.split(",")
        var today = new Date();
        var time = today.getMinutes() * 60 + today.getSeconds();
        line = { fob_id: rowArray[0], vote: rowArray[1], timestamp: time };
        arr.push(line);
    });

    // (D) DONE - FULL ARRAY
    reader.on("close", () => {
    });
    collect.insert(arr, function (err, res) {
        if (err) throw err;
        console.log("Inserted from txt" + arr.toString());
    });
    return arr
}
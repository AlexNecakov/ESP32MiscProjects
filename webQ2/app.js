const express = require("express");
const bodyParser = require('body-parser');
const app = express();
var DEBUG = true;

const { SerialPort, ReadlineParser } = require('serialport')
const parser = new ReadlineParser()
const port = new SerialPort({ path:"COM3", baudRate:115200 })//check com port per pc
port.pipe(parser)
const fs = require("fs"), rl = require("readline");

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
count = 0;
aveUS =0;
aveIR=0;
aveT=0;
  async function getSerialVals(){
    let rawdata = fs.readFileSync('data.json');
    let data = JSON.parse(rawdata);
    var US =data[0];
    var IR = data[1];
    var TEMP = data[2];
    console.log("Temp:",US,"IR:",TEMP,"US:",IR)
    count = 0;
    aveUS =0;
    aveIR=0;
    aveT=0;
    return [US,IR,TEMP]//US,TEMP,
  }


app.get("/test", async(req, res) => {

    res.sendFile(__dirname + "/index.html");
    res.sendFile(__dirname + "/source.js");
    res.sendFile(__dirname + "/styles.css");
    vals = await getSerialVals();
    // vals = [Math.random()*100,Math.random()*100,Math.random()*100]
    res.end(JSON.stringify({"US":vals[1],"IR":vals[2],"TEMP":vals[0]}));


});
function writejson(data){
    val = JSON.parse(data)
    aveUS = (val[0]*1+aveUS*count)/(count+1)
    aveIR=(val[1]*1+aveIR*count)/(count+1);
    aveT=(val[2]*1+aveT*count)/(count+1);
    datas = [aveUS,aveIR,aveT]
    stringput = JSON.stringify(datas);
    fs.writeFile("data.json", stringput, 'utf8', function (err) {
        if (err) {
            console.log("An error occured while writing JSON Object to File.");
            return console.log(err);
        }

        console.log("JSON file has been saved.");
    });

}
parser.on('data', writejson)

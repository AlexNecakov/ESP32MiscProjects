var speedvtime = [[],[]];
var errorCvtime = [[],[]];
var maxE = 0;
var minE = 0;

var FrontD = 0;
var lastTime = 0;
var queryTime = 1000;
var F_TRESH = 20;//20 cm
var S_THRESH = 0.21 ;
var maxS = S_THRESH;
var minS = 0;
var Ebreak = false;
window.onload = async function () {
  var speedInput = document.querySelector('#speedsp');
  speedInput.addEventListener('input', function () {
              S_THRESH = this.value;
          });

  var dInput = document.querySelector('#frontsp');
  dInput.addEventListener('input', function () {
              F_TRESH = this.value;
          });
    draw_speed();
}


const fetchs = setInterval(async function () {
    vals = await getData();
    console.log("data:",vals)
    lastTime += 1;
    if (speedvtime[0].length>100) {
      speedvtime = [[],[]];
      errorCvtime = [[],[]];
      FrontD = [[],[]];
      maxS = S_THRESH;
      minS = 0;
    }
    maxE = (maxE>vals[1])?maxE:vals[1];
    minE = (minE<vals[1])?minE:vals[1];
    maxS = (maxS>vals[0])?maxS:vals[0];
    minS = (minS<vals[0])?minS:vals[0];
    var dInput = document.querySelector('#frontsp');
    var speedInput = document.querySelector('#speedsp');
    speedInput.step = (maxS-minS)/20;
    dInput.step = (maxE-minE)/20;
    speedvtime[0].push(vals[0]);
    speedvtime[1].push(lastTime);
    errorCvtime[0].push(vals[1]);
    errorCvtime[1].push(lastTime);
    FrontD = vals[2];
    draw_speed();
    draw_steer();
    draw_stop();
}, queryTime);

async function getData() {
    const response = await fetch("/data");
    return response.json();
}
function draw_speed(){
  var data = [];
  for(var i = 0; i<speedvtime[0].length;i++ ){
    var obj = { x: speedvtime[1][i], y: speedvtime[0][i] };
    data.push(obj)
  }
  var ms = (S_THRESH>maxS)?S_THRESH:maxS;
  var mns = (S_THRESH<minS)?S_THRESH:minS;
  var chart = new CanvasJS.Chart("graph1", {
              animationEnabled: false,
              title: {
                text: "Crawler Speed"
              },
              axisX: {
                title: "Time"
              },
              axisY: {
                title: "M/s",
                suffix: "",
                includeZero: true,
                gridThickness: 0,
                stripLines: [
                {

                    value: S_THRESH,
                    thickness:3,
                    color:"#FF0000",
                    label : "setpoint",
                    labelFontColor: "#a8a8a8",
                }

            ],
            maximum: (ms)*1.15+0.001,
            minimum: (mns)*1.15-0.001,
              },
              data: [{
                type: "line",
                name: "Speed",
                connectNullData: true,
                xValueType: "speed",
                markerSize: 0,
                dataPoints: data,
               }]
              });
              chart.render();
}
function draw_steer(){
  var data = [];
  for(var i = 0; i<errorCvtime[0].length;i++ ){
    var obj = { x: errorCvtime[1][i], y: errorCvtime[0][i] };
    data.push(obj)
  }
  var chart = new CanvasJS.Chart("graph2", {
              animationEnabled: false,
              title: {
                text: "Crawler Centering"
              },
              axisX: {
                title: "Time"
              },
              axisY: {

                title: "Distance from center ",
                suffix: "cm",
                includeZero: true,
                maximum: maxE*1.15+0.001,
                minimum: minE*1.15-0.001,
                gridThickness: 0,
                stripLines: [
                {

                    value: 0,
                    thickness:3,
                    color:"#FF0000",
                    label : "setpoint",
                    labelFontColor: "#a8a8a8",
                }]
              },
              data: [{
                type: "line",
                name: "Distance From center",
                connectNullData: true,
                //nullDataLineDashType: "solid",
                xValueType: "distance cm",
                markerSize: 0,
                dataPoints: data,
               }]
              });
              chart.render();
}
function draw_stop(){
  var ms =(F_TRESH>FrontD)?F_TRESH:FrontD;
  var chart = new CanvasJS.Chart("graph3", {
              animationEnabled: false,
              title: {
                text: "Distance From Stop"
              },
              axisX: {
                title: "Lidar "
              },
              axisY: {
                title: "distnace cm",
                suffix: "",
                includeZero: true,
                gridThickness: 0,
                stripLines: [
                {

                    value: F_TRESH,
                    thickness:3,
                    color:"#FF0000",
                    label : "setpoint",
                    labelFontColor: "#a8a8a8",
                }

            ],
            maximum: (ms)*1.15+0.001,
            minimum: 0,
              },
              data: [{
                type: "column",

                dataPoints: [{y: FrontD, label: "Lidar1"}],
               }]
              });
              chart.render();
}
async function postData( data = {}) {
  // Default options are marked with *
  parsedData = data
  url = '/esp';
  const response = await fetch(url, {

    method: 'POST', // *GET, POST, PUT, DELETE, etc.
    // mode: 'cors', // no-cors, *cors, same-origin
    // cache: 'no-cache', // *default, nofalse-cache, reload, force-cache, only-if-cached
    // credentials: 'same-origin', // include, *same-origin, omit
    headers: {
      'Content-Type': 'application/json'
      // 'Content-Type': 'application/x-www-form-urlencoded',
    },
    body: parsedData // body data type must match "Content-Type" header
  }).then(response => response.json())
    .then(data => {
      console.log('Success:', data);
    })
    .catch((error) => {
      console.error('Error:', error);
    });

}
async function clicked(){
  Ebreak = !Ebreak;
  img = document.getElementById("but");
  await postData(JSON.stringify({break: Ebreak}));
  if(Ebreak){
    img.src = "button1.png";
  }
  else{
      img.src = "button2.png"
  }
}

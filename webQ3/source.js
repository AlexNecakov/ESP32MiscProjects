var fixedtimeWindow = [[0,5,5,5,3,5],
                       [1,3,4,5,3,5],
                       [2,6,2,6,3,5],
                       [3,2,4,5,3,5],
                       [4,4,8,7,3,5],
                       [5,7,5,6,3,5],
                       [6,2,8,4,3,5],
                       [7,8,5,0,3,5],
                       [8,3,3,1,3,5],
                       [9,5,6,1,3,5],
                       [10,3,4,8,3,5],
                  ];
var queryTime = 1000;
var level = 0;
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
// sends data to web server

window.onload = async function(){
  await main();
  var slider = document.getElementById("myRange");
  slider.oninput = function() {
    console.log(this.value);
    postData(JSON.stringify({data:this.value}));
  }
  // moleGame.onclick = () => {
  //   console.log(gametoggle)
  //   gametoggle = !gametoggle;
  //   if(gametoggle){
  //     line_graph.style.display = "none";
  //     stripe_graph.style.display = "none";
  //     text.style.display = "none";
  //     specChart.style.display = "none";
  //     gameDiv.style.display = "block";
  //   }
  //   else{
  //     line_graph.style.display = "block";
  //     stripe_graph.style.display = "none";
  //     text.style.display = "block";
  //     specChart.style.display = "none";
  //     gameDiv.style.display = "none";
  //     draw_graph();
  //   }
  // }
  // line_graph.onclick = () => {
  //   running = !running;
  // }
  // imagemenu.onmouseover = () => {
  //   if(gametoggle)return;
  //   imagemenu.src = "goodbye.png"
  //   line_graph.style.display = "none";
  //   stripe_graph.style.display = "block";
  //   gameDiv.style.display = "none";
  //   draw_strip();
  // }
  // imagemenu.onmouseout = () => {
  //   if(gametoggle)return;
  //     imagemenu.src = "Hello.png"
  //     line_graph.style.display = "block";
  //     text.style.display = "block";
  //     stripe_graph.style.display = "none";
  //     gameDiv.style.display = "none";
  //     draw_graph()
  //
  //   }
}

async function getData(){
  const response = await fetch("/data")

  return response.json();
}
const random = (min, max) => Math.random() * (max - min) + min;

function delay(delay) {
  return new Promise(resolve => {
    setTimeout(() => {
      resolve(delay);
    }, delay);
  });
}

function toggleDataSeries(e){
	if (typeof(e.dataSeries.visible) === "undefined" || e.dataSeries.visible) {
		e.dataSeries.visible = false;
	}
	else{
		e.dataSeries.visible = true;
	}
	chart.render();
}
function draw_accel(){
  var len = fixedtimeWindow.length
  var XdataList = [];
  var YdataList = [];
  var ZdataList = [];
  for (i=0;i<len;i++){
    XdataList.push({x:fixedtimeWindow[i][0],y:fixedtimeWindow[i][1]})
    YdataList.push({x:fixedtimeWindow[i][0],y:fixedtimeWindow[i][2]})
    ZdataList.push({x:fixedtimeWindow[i][0],y:fixedtimeWindow[i][3]})
  }
  var chart = new CanvasJS.Chart("acceleration", {
  	animationEnabled: false,
  	title:{
  		text: "Acceleration Meseasurements"
  	},
  	axisX: {
  		title: "Time ",
      suffix: " S",
      minimum:fixedtimeWindow[0][0],
      maximum:fixedtimeWindow[9][0],
  	},
  	axisY: {
  		title: "Acceleration  ",
  		suffix: " m/(S^2)"
  	},
  	legend:{
  		cursor: "pointer",
  		fontSize: 16,
  		itemclick: toggleDataSeries
  	},
  	toolTip:{
  		shared: true
  	},
  	data: [{
  		name: "X Accel",
  		type: "spline",
  		yValueFormatString:  "#.##m/(S^2)",
  		showInLegend: true,
  		dataPoints: XdataList,
  	},
  	{
  		name: "Y Accel",
  		type: "spline",
  		yValueFormatString: "#.##m/(S^2)",
  		showInLegend: true,
  		dataPoints: YdataList,
  	},
    {
  		name: "Z Accel",
  		type: "spline",
  		yValueFormatString: "#.##m/(S^2)",
  		showInLegend: true,
  		dataPoints: ZdataList,
  	},
    ]

  });
  chart.render();
}
function draw_temp(){
  var len = fixedtimeWindow.length
  var TempList = [];
  for (i=0;i<len;i++){
    TempList.push({x:fixedtimeWindow[i][0],y:fixedtimeWindow[i][4]})
  }
  var chart = new CanvasJS.Chart("Temp", {
  	animationEnabled: false,
  	title:{
  		text: "Temp Meseasurements"
  	},
  	axisX: {
  		title: "Time ",
      suffix: " S",
      minimum:fixedtimeWindow[0][0],
      maximum:fixedtimeWindow[9][0],
  	},
  	axisY: {
  		title: "Temp  ",
  		suffix: " radians"
  	},
  	legend:{
  		cursor: "pointer",
  		fontSize: 16,
  		itemclick: toggleDataSeries
  	},
  	toolTip:{
  		shared: true
  	},
  	data: [{
  		name: "Tampabay",
  		type: "spline",
  		yValueFormatString:  "#.##&deg",
  		showInLegend: true,
  		dataPoints: TempList,
  	},
    ]

  });
  chart.render();
}
function draw_Voltage(){
  var len = fixedtimeWindow.length
  var voltList = [];
  for (i=0;i<len;i++){
    voltList.push({x:fixedtimeWindow[i][0],y:fixedtimeWindow[i][5]})
  }
  var chart = new CanvasJS.Chart("bat", {
  	animationEnabled: false,
  	title:{
  		text: "BatLevel"
  	},
  	axisX: {
  		title: "Time ",
      suffix: " S",
      minimum:fixedtimeWindow[0][0],
      maximum:fixedtimeWindow[9][0],
  	},
  	axisY: {
  		title: "Temp  ",
  		suffix: "I/R"
  	},
  	legend:{
  		cursor: "pointer",
  		fontSize: 16,
  		itemclick: toggleDataSeries
  	},
  	toolTip:{
  		shared: true
  	},
  	data: [{
  		name: "boltz",
  		type: "spline",
  		yValueFormatString:  "#.##V",
  		showInLegend: true,
  		dataPoints: voltList,
  	},
    ]

  });
  chart.render();
}
async function main(){

  draw_accel();
  draw_temp();
  draw_Voltage();

}
const fetchs = setInterval(async function() {
    fixedtimeWindow = await getData();
    main();
    }
, queryTime);

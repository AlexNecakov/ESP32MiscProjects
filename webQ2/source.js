TEMP = 26;
BASETEMP = 25;
IRList = [0];
ULTRAList = [0];
TIMEList = [0];
TempList = [0];
running = false;
mousedTemp = false;
gametoggle=false;
queryTime = 1000;
moleimg = new Image();
moleimg.src = "MoleBig.png"

window.onload = async function(){
  await main();
  var imagemenu = document.getElementById("controller");
  var stripe_graph = document.getElementById("stupid_strip");
  var line_graph = document.getElementById("goodVeversion");
  var text = document.getElementById("temp");
  var specChart = document.getElementById("chartContainer1");
  var moleGame = document.getElementById("game");
  var gameDiv = document.getElementById("gameDiv");
  stripe_graph.style.display = "none";
  gameDiv.style.display = "none";
  specChart.style.display = "none";
  moleGame.onclick = () => {
    console.log(gametoggle)
    gametoggle = !gametoggle;
    if(gametoggle){
      line_graph.style.display = "none";
      stripe_graph.style.display = "none";
      text.style.display = "none";
      specChart.style.display = "none";
      gameDiv.style.display = "block";
    }
    else{
      line_graph.style.display = "block";
      stripe_graph.style.display = "none";
      text.style.display = "block";
      specChart.style.display = "none";
      gameDiv.style.display = "none";
      draw_graph();
    }
  }
  line_graph.onclick = () => {
    running = !running;
  }
  text.onclick = () => {
    if(mousedTemp)specChart.style.display = "none";
    else specChart.style.display = "block";
    mousedTemp=!mousedTemp;
  }
  imagemenu.onmouseover = () => {
    if(gametoggle)return;
    imagemenu.src = "goodbye.png"
    line_graph.style.display = "none";
    stripe_graph.style.display = "block";
    gameDiv.style.display = "none";
    draw_strip();
  }
  imagemenu.onmouseout = () => {
    if(gametoggle)return;
      imagemenu.src = "Hello.png"
      line_graph.style.display = "block";
      text.style.display = "block";
      stripe_graph.style.display = "none";
      gameDiv.style.display = "none";
      draw_graph()

    }
}

async function getData(){

  const response = await fetch("/test")

  return response.json();
}
const random = (min, max) => Math.random() * (max - min) + min;
function Temp(baseTemp,newTemp){
  // get css style stuff
  cPos = "rgb(242, 27, 90)";
  cNeg = "rgb(0,27,90)";
  var basePercentages = 50;
  var diff =-Math.abs(baseTemp-newTemp);
  var gain = 0.05;
  var multFactor = 1/(1+Math.exp(-diff*gain));
  var multiPercentage = multFactor*100;


  var newLow = basePercentages*multFactor
  var newhigh= 100-newLow
  var col = (newTemp>baseTemp)?cPos:cNeg;

  document.body.style.background= "linear-gradient(90deg,  rgba(0,0,0,1) 0%,"+col+" "+newLow.toString()+"%, " +col+" "+ newhigh.toString()+"%, rgba(0,0,0,1) 100%)"
  document.getElementById("temp").innerHTML = newTemp.toString()+"&degC";
}
async function setTemp(baseTemp,newTemp){
  var i = TEMP
  var dif = Math.abs(newTemp-TEMP)
  var totalTime = queryTime/2;
  if (newTemp>i){
    for (i;i<Math.floor(newTemp);i++){
      Temp(BASETEMP,i);
      let delayres = await delay(totalTime/dif);
    }
  }
  else if(newTemp<i){
    for (i;i>Math.floor(newTemp);i--){
      Temp(BASETEMP,i);
      let delayres = await delay(totalTime/dif);
    }
  }
  else{
    Temp(BASETEMP,newTemp);
  }
  TEMP = newTemp;
}

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
function formatDataDistanceStrip(divVal){
  var len = TIMEList.length
  var i = 0
  var MainList = []
  var greaterthanVal =divVal
  for (i=0;i<len;i++){
    MainList.push({x:(1+random(-greaterthanVal,greaterthanVal)),y:ULTRAList[i]})
    MainList.push({x:(2+random(-greaterthanVal,greaterthanVal)),y:IRList[i]})
  }
  return MainList;
}
function formatDataTempStrip(divVal){
  var len = TIMEList.length
  var i = 0
  var MainList = []
  var greaterthanVal =divVal
  for (i=0;i<len;i++){
    MainList.push({x:(1+random(-greaterthanVal,greaterthanVal)),y:TempList[i]})

  }
  return MainList;
}
function draw_graph(){
  var len = TIMEList.length
  var i = 0
  var irList = []
  var usList = []
  var tempList =[]
  for (i=0;i<len;i++){
    irList.push({x:TIMEList[i],y:IRList[i]})
    usList.push({x:TIMEList[i],y:ULTRAList[i]})
  }
  for (i=0;i<len;i++){
    tempList.push({x:TIMEList[i],y:TempList[i]})
  }
  var windowsize =10;
  var min = (running)?len-windowsize*1.5:0;
  var max = (running)?len+windowsize/2:len;
  var chart = new CanvasJS.Chart("chartContainer", {
  	animationEnabled: false,
  	title:{
  		text: "Distance Meseasurements"
  	},
  	axisX: {
  		title: "Time ",
      suffix: " S",
      minimum:min,
      maximum:max,
  	},
  	axisY: {
  		title: "Distance ",
  		suffix: " m"
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
  		name: "Ultra Sonic",
  		type: "spline",
  		yValueFormatString:  "#.##m",
  		showInLegend: true,
  		dataPoints: irList
  	},
  	{
  		name: "IR Range Finder",
  		type: "spline",
  		yValueFormatString: "#.##m",
  		showInLegend: true,
  		dataPoints: usList
  	}]
  });
  chart.render();
  if(mousedTemp){
  var chart1 = new CanvasJS.Chart("chartContainer1", {
  	animationEnabled: false,
  	title:{
  		text: "Temp Meseasurements"
  	},
  	axisX: {
  		title: "Time ",
      suffix: " S",
      minimum:min,
      maximum:max,
  	},
  	axisY: {
  		title: "Temp ",
  		suffix: " C"
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
  		name: "Temp",
  		type: "spline",
  		yValueFormatString:  "#0.##C",
  		showInLegend: true,
  		dataPoints: tempList
  	}]
  });
  chart1.render();
}
}
function draw_strip(){
  var len = TIMEList.length
  var i = 0
  var irList = []
  var usList = []
  var divVal = 0.1
  DistanceStrip = formatDataDistanceStrip(divVal);
  TempStri= formatDataTempStrip(divVal)
  var chart = new CanvasJS.Chart("DistanceStrip", {
  title:{
    text: "Distance strip-chart"
  },
  axisX: {
    title:"Class: US,IR",
    minimum:0,
    maximum: 3,
    gridThickness: 0,
    tickLength: 0,
    lineThickness: 0,
    labelFormatter: function(){
      return " ";
    }
  },
  axisY:{
    title: "Distance in m",
    valueFormatString: "#.##0m"
  },
  data: [{
    type: "scatter",
    toolTipContent: "<b>Distance</b>{y} m<br/>",
    color: "blue",
    dataPoints: DistanceStrip,
  }]
});
  chart.render();
  var chart1 = new CanvasJS.Chart("TempStrip", {
    title:{
  		text: "Temp strip-chart"
  	},
  	axisX: {
  		title:"Class: Temp",
      minimum:0,
		  maximum: 2,
      gridThickness: 0,
      tickLength: 0,
      lineThickness: 0,
      labelFormatter: function(){
        return " ";
      }
  	},
  	axisY:{
  		title: "Temp in C",
  		valueFormatString: "#.##0C°"
  	},
  	data: [{
  		type: "scatter",
  		toolTipContent: "<b>Temp</b>{y} c<br/>",
      color: "red",
  		dataPoints: TempStri
  	}]
  });
  chart1.render();
}
async function main(){

  draw_graph();
  await setTemp(BASETEMP,TEMP)
  document.getElementById("temp").innerHTML = TEMP.toString()+"&degC";

}
var moleRight=false;
var moleLeft=false;
var score = 0;

function check_moles(){
var distanceThresh =0.3;
  var randomThresh = 0.3;
  if(moleRight){
    var last_val = IRList[IRList.length-1]
    if(last_val<distanceThresh && last_val>0.05){
      moleRight=false;
      score++;
    }
  }
  else{
    if(randomThresh>Math.random()){
      moleRight=true;
    }
  }
  if(moleLeft){
    var last_val = ULTRAList[ULTRAList.length-1]
    if(last_val<distanceThresh&& last_val>0.05){
      moleLeft=false;
      score++;
      return;
    }
  }
  else{
    if(randomThresh>Math.random()){
      moleLeft=true;
    }
  }
}
function draw_game(){
  check_moles();
  let box = document.querySelector('.mole');
  let w = box.clientWidth;
  let h = box.clientHeight;
  var canvas = document.getElementById('canvas');
  canvas.width=w;
  canvas.height=h;
  var ctx = canvas.getContext('2d');
  var width = canvas.width;
  var height = canvas.height;
  ctx.clearRect(0, 0, width,height);
  ctx.fillStyle = "green";
  ctx.beginPath();
  ctx.fillRect(0, height/2, width,height/2)
  ctx.stroke();

  ctx.fillStyle = "blue";
  ctx.beginPath();
  ctx.fillRect(0, 0, width,height/2)
  ctx.stroke();

  if(moleLeft)ctx.drawImage(moleimg,0,height/4,moleimg.width, (moleimg.height))
  if(moleRight)ctx.drawImage(moleimg,width-moleimg.width,height/4,moleimg.width, (moleimg.height))
  ctx.fillStyle = "white";
  ctx.font = "20px Arial";
  var str2 = "Score: "+score.toString();
  ctx.fillText(str2, 0,50);



}
const fetchs = setInterval(async function() {
    var value = await getData()
    TIMEList.push(TIMEList[TIMEList.length-1]+1)
    ULTRAList.push(value.US)
    IRList.push(value.IR)
    TempList.push(value.TEMP)
    await setTemp(BASETEMP,value.TEMP)
    if(gametoggle){
      draw_game();
    }
    else{
      // if (TIMEList.length-1>150){
      //   TIMEList=[TIMEList[TIMEList.length-1]]
      //   ULTRAList=[ULTRAList[TIMEList.length-1]]
      //   IRList=[IRList[TIMEList.length-1]]
      //   TempList=[TempList[TIMEList.length-1]]
      // }
      draw_graph();
      draw_strip();
    }
}, queryTime);

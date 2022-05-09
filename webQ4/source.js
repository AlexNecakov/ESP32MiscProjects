var voteCount = [1, 2, 3];
var voteList = [];
var queryTime = 1000;

window.onload = async function () {
    await main();
}

const fetchs = setInterval(async function () {
    vals = await getVotes();
    voteCount[0] = vals[0];
    voteCount[1] = vals[1];
    voteCount[2] = vals[2];
    voteList = vals[3];
    main();
}, queryTime);

async function getVotes() {
    const response = await fetch("/data");
    return response.json();
}

function draw_voteList() {
    var mainContainer = document.getElementById("voteListArea");
    mainContainer.innerHTML = '';
    for (var i = 0; i < voteList.length; i++) {
        var div = document.createElement("div");
        div.innerHTML = 'fob_id: ' + voteList[i].fob_id + ' vote: ' + voteList[i].vote + ' timestamp: ' + voteList[i].timestamp;
        mainContainer.appendChild(div);
    }
}

function draw_voteCount() {
    var chart = new CanvasJS.Chart("voteCountChart", {
        animationEnabled: false,
        theme: "light2", // "light1", "light2", "dark1", "dark2"
        title: {
            text: "Votes by Candidate"
        },
        axisY: {
            title: "Votes"
        },
        data: [{
            type: "column",
            dataPoints: [
                { y: voteCount[0], label: "Red", color: "red" },
                { y: voteCount[1], label: "Green", color: "green" },
                { y: voteCount[2], label: "Blue", color: "blue" },
                { y: voteList.length, label: "Total", color: "black"}
            ]
        }]
    });
    chart.render();
}

async function resetCount() {
    var parsedData = JSON.stringify({ data: 10 });
    const url = '/resetdb';
    const response = await fetch(url, {

        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: parsedData
    }).then(response => response.json())
        .then(data => {
            //console.log('Success:', data);
        })
        .catch((error) => {
            console.error('Error:', error);
        });
}

async function main() {
    draw_voteCount();
    draw_voteList();
}

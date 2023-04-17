//document.getElementById("datetime").innerHTML = "WebSocket is not connected";

var websocket = new WebSocket('ws://'+location.hostname+'/');
var meter1 = 0;
var meter2 = 0;
var meter3 = 0;
var roll = 0.0;
var yaw = 0.0;
var pitch = 0.0;
var deg2rad = 0.0174533;

function sendText(name) {
	console.log('sendText');
	var data = {};
	data["id"] = name;
	console.log('data=', data);
	json_data = JSON.stringify(data);
	console.log('json_data=' + json_data);
	websocket.send(json_data);
}

websocket.onopen = function(evt) {
	console.log('WebSocket connection opened');
	var data = {};
	data["id"] = "init";
	console.log('data=', data);
	json_data = JSON.stringify(data);
	console.log('json_data=' + json_data);
	websocket.send(json_data);
	//document.getElementById("datetime").innerHTML = "WebSocket is connected!";
}

websocket.onmessage = function(evt) {
	var msg = evt.data;
	console.log("msg=" + msg);
	var values = msg.split('\4'); // \4 is EOT
	//console.log("values=" + values);
	switch(values[0]) {
		case 'HEAD':
			console.log("HEAD values[1]=" + values[1]);
			var h1 = document.getElementById( 'header' );
			h1.textContent = values[1];
			break;

		case 'METER':
			//console.log("gauge1=" + Object.keys(gauge1.options));
			//console.log("gauge1.options.units=" + gauge1.options.units);
			console.log("METER values[1]=" + values[1]);
			console.log("METER values[2]=" + values[2]);
			console.log("METER values[3]=" + values[3]);
			if (values[1] != "") {
				gauge1.options.title = values[1];
				document.getElementById("canvas1").style.display = "inline-block";
				meter1 = 1;
			}
			if (values[2] != "") {
				gauge2.options.title = values[2];
				document.getElementById("canvas2").style.display = "inline-block";
				meter2 = 1;
			}
			if (values[3] != "") {
				gauge3.options.title = values[3];
				document.getElementById("canvas3").style.display = "inline-block";
				meter3 = 1;
			}
			break;

		case 'DATA':
			console.log("DATA values[1]=" + values[1]);
			var val1 = parseFloat(values[1]);
			gauge1.value = val1;
			gauge1.update({ valueText: val1.toFixed(1) });
			roll = val1 * deg2rad;
			if (meter2) {
				console.log("DATA values[2]=" + values[2]);
				var val2 = parseFloat(values[2]);
				gauge2.value = val2;
				gauge2.update({ valueText: val2.toFixed(1) });
				pitch = val2 * deg2rad;
			}
			if (meter3) {
				console.log("DATA values[3]=" + values[3]);
				var val3 = parseFloat(values[3]);
				gauge3.value = val3;
				gauge3.update({ valueText: val3.toFixed(1) });
				yaw = val3 * deg2rad * -1.0;
			}
			break;

		default:
			break;
	}
}

websocket.onclose = function(evt) {
	console.log('Websocket connection closed');
	//document.getElementById("datetime").innerHTML = "WebSocket closed";
}

websocket.onerror = function(evt) {
	console.log('Websocket error: ' + evt);
	//document.getElementById("datetime").innerHTML = "WebSocket error????!!!1!!";
}

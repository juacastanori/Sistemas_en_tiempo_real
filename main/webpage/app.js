/**
 * Add gobals here
 */
var seconds 	= null;
var otaTimerVar =  null;
var wifiConnectInterval = null;

/**
 * Initialize functions here.
 */
$(document).ready(function(){
	getUpdateStatus();
	startDHTSensorInterval();
	startCurrentTimeInterval();
	print("ready")
	/*activate_listener('red_val');
	activate_listener('green_val');
	activate_listener('blue_val');
	
	$("#send_rgb").on("click", function(){
		send_rgb_values();
	});*/
	// *** NUEVOS LISTENERS PARA LOS BOTONES RGB ***
    $("#toggle_red_led").on("click", function(){
        toggle_led_color('/toggle_red.json');
    });
    
    $("#toggle_green_led").on("click", function(){
        toggle_led_color('/toggle_green.json');
    });
    
    $("#toggle_blue_led").on("click", function(){
        toggle_led_color('/toggle_blue.json');
    });
    // ********************************************** 
	$("#apagar_uart").on("click", function(){
		turn_off_uart();
	}); 

});   

/**
 * Gets file name and size for display on the web page.
 */        
function getFileInfo() 
{
    var x = document.getElementById("selected_file");
    var file = x.files[0];

    document.getElementById("file_info").innerHTML = "<h4>File: " + file.name + "<br>" + "Size: " + file.size + " bytes</h4>";
}

/**
 * Handles the firmware update.
 */
function updateFirmware() 
{
    // Form Data
    var formData = new FormData();
    var fileSelect = document.getElementById("selected_file");
    
    if (fileSelect.files && fileSelect.files.length == 1) 
	{
        var file = fileSelect.files[0];
        formData.set("file", file, file.name);
        document.getElementById("ota_update_status").innerHTML = "Uploading " + file.name + ", Firmware Update in Progress...";

        // Http Request
        var request = new XMLHttpRequest();

        request.upload.addEventListener("progress", updateProgress);
        request.open('POST', "/OTAupdate");
        request.responseType = "blob";
        request.send(formData);
    } 
	else 
	{
        window.alert('Select A File First')
    }
}

/**
 * Progress on transfers from the server to the client (downloads).
 */
function updateProgress(oEvent) 
{
    if (oEvent.lengthComputable) 
	{
        getUpdateStatus();
    } 
	else 
	{
        window.alert('total size is unknown')
    }
}

/**
 * Posts the firmware udpate status.
 */
function getUpdateStatus() 
{
    var xhr = new XMLHttpRequest();
    var requestURL = "/OTAstatus";
    xhr.open('POST', requestURL, false);
    xhr.send('ota_update_status');

    if (xhr.readyState == 4 && xhr.status == 200) 
	{		
        var response = JSON.parse(xhr.responseText);
						
	 	document.getElementById("latest_firmware").innerHTML = response.compile_date + " - " + response.compile_time

		// If flashing was complete it will return a 1, else -1
		// A return of 0 is just for information on the Latest Firmware request
        if (response.ota_update_status == 1) 
		{
    		// Set the countdown timer time
            seconds = 10;
            // Start the countdown timer
            otaRebootTimer();
        } 
        else if (response.ota_update_status == -1)
		{
            document.getElementById("ota_update_status").innerHTML = "!!! Upload Error !!!";
        }
    }
}

/**
 * Displays the reboot countdown.
 */
function otaRebootTimer() 
{	
    document.getElementById("ota_update_status").innerHTML = "OTA Firmware Update Complete. This page will close shortly, Rebooting in: " + seconds;

    if (--seconds == 0) 
	{
        clearTimeout(otaTimerVar);
        window.location.reload();
    } 
	else 
	{
        otaTimerVar = setTimeout(otaRebootTimer, 1000);
    }
}

/**
 * Gets DHT22 sensor temperature and humidity values for display on the web page.
 */
function getDHTSensorValues()
{
	$.getJSON('/dhtSensor.json', function(data) {
        $("#temperature_reading").text(data["temp"] + " °C");
        // La lectura de humedad ha sido eliminada.
    });
}

/**
 * Sets the interval for getting the updated DHT22 sensor values.
 */
function startDHTSensorInterval()
{
	setInterval(getDHTSensorValues, 5000);    
}

/**
 * Gets the current time for display on the web page.
 */
function getCurrentTimeValues()
{
    // Asume que el servidor del ESP32 ofrece un endpoint /time.json 
    // que devuelve un JSON con el formato: {"time": "HH:MM:SS"}
    $.getJSON('/time.json', function(data) {
        $("#current_time_reading").text(data["time"]);
    });
}

/**
 * Sets the interval for getting the updated current time.
 */
function startCurrentTimeInterval()
{
    setInterval(getCurrentTimeValues, 5000); // Actualiza cada 5 segundos
}


/**
 * Clears the connection status interval.
 */
function stopWifiConnectStatusInterval()
{
	if (wifiConnectInterval != null)
	{
		clearInterval(wifiConnectInterval);
		wifiConnectInterval = null;
	}
}

/**
 * Gets the WiFi connection status.
 */
function getWifiConnectStatus()
{
	var xhr = new XMLHttpRequest();
	var requestURL = "/wifiConnectStatus";
	xhr.open('POST', requestURL, false);
	xhr.send('wifi_connect_status');
	
	if (xhr.readyState == 4 && xhr.status == 200)
	{
		var response = JSON.parse(xhr.responseText);
		
		document.getElementById("wifi_connect_status").innerHTML = "Connecting...";
		
		if (response.wifi_connect_status == 2)
		{
			document.getElementById("wifi_connect_status").innerHTML = "<h4 class='rd'>Failed to Connect. Please check your AP credentials and compatibility</h4>";
			stopWifiConnectStatusInterval();
		}
		else if (response.wifi_connect_status == 3)
		{
			document.getElementById("wifi_connect_status").innerHTML = "<h4 class='gr'>Connection Success!</h4>";
			stopWifiConnectStatusInterval();
		}
	}
}

/**
 * Starts the interval for checking the connection status.
 */
function startWifiConnectStatusInterval()
{
	wifiConnectInterval = setInterval(getWifiConnectStatus, 2800);
}

/**
 * Send the RGB values.
 */
/*
function send_rgb_values()
{
	// Get the SSID and password
	red_val = $("#red_val").val();
	green_val = $("#green_val").val();
	blue_val = $("#blue_val").val();
	
	
	$.ajax({
		url: '/rgb_vals.json',
		dataType: 'json',
		method: 'POST',
		cache: false,
		headers: {'red_val': red_val, 'green_val': green_val, 'blue_val': blue_val},
		data: {'timestamp': Date.now()}
	});
}
*/

function turn_off_uart()
{

	
	$.ajax({
		url: '/uart_off.json',
		dataType: 'json',
		method: 'POST',
		cache: false,
		//headers: {'my-connect-ssid': selectedSSID, 'my-connect-pwd': pwd},
		//data: {'timestamp': Date.now()}
	});
//	var xhr = new XMLHttpRequest();
//	xhr.open("POST", "/toogle_led.json");
//	xhr.setRequestHeader("Content-Type", "application/json");
//	xhr.send(JSON.stringify({data: "mi información"}));
}


/**
 * Toggle individual LED color function.
 * @param {string} url - The endpoint URL for the specific color toggle (e.g., '/toggle_red.json').
 */
function toggle_led_color(url)
{
    $.ajax({
        url: url,
        dataType: 'json',
        method: 'POST',
        cache: false,
    });
//	var xhr = new XMLHttpRequest();
//	xhr.open("POST", "/toogle_led.json");
//	xhr.setRequestHeader("Content-Type", "application/json");
//	xhr.send(JSON.stringify({data: "mi información"}));
}

/*
function activate_listener( used_id ){
	
	const myInput = document.getElementById( used_id );

	myInput.addEventListener('input', () => {
	if (myInput.value > 255) {
		myInput.value = 255;
	}
	if (!Number.isInteger(Number(myInput.value))) {
		myInput.value = Math.floor(Number(myInput.value));
	}
	});
}


*/

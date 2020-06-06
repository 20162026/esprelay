<html><head><title>Test</title>
<link rel="stylesheet" type="text/css" href="../static/style.css">
</head>
<body>
    <div>
        <div id="borders" height="200">
            <form id="form" action="./settingswifi.cgi.cgi" method="post" >
			<h4>WiFi Client (Station Mode and Mesh)</h4>
            <p>
                SSID: <br>
                <input type="text" name="essid" value="%wifissid%" pattern="[0-9A-Za-z/$@.,_-]{2,31}" title="only 0-9A-Za-z/$@.,_-  2-31 char" required> <br>
                password: <br>
                <input type="password" name="passwd" pattern="[0-9A-Za-z/$@]{8,63}" title="only 0-9A-Za-z/$@  8-63 char" required> <br>
                <input type="submit" value="Submit">
            </p>
            </form>
			
			<form id="form" action="./settingswifi.cgi" method="post" >
			<h4>WiFi AP</h4>
            <p>
                SSID: <br>
                <input type="text" name="apssid" value="%APssid%" pattern="[0-9A-Za-z/$@.,_-]{2,31}" title="only 0-9A-Za-z/$@.,_-  2-31 char" required> <br>
                password: <br>
                <input type="password" name="appasswd" pattern="[0-9A-Za-z/$@]{2,63}" title="only 0-9A-Za-z/$@  2-63 char" required> <br>
                <input type="submit" value="Submit">
            </p>
            </p>
            </form>
			<form id="form" action="./settingswifi.cgi" method="post" >
			<h4>WiFi mode</h4>
            <p>
                Select wifi mode:<br>
                <input type="radio" name="mode" value="APmode" required>APmode<br>
				<input type="radio" name="mode" value="STAmode" required>STAmode<br>
                <input type="submit" value="Submit">
            </p>
            </p>
            </form>
        </div>   
        <div id="borders">
            
            <form id="form" action="./settingsmesh.cgi" method="post">
			<h4>MQTT topic</h4>
            <p>
                <input type="text" name="topic" value="%topic%" pattern="[0-9A-Za-z/]{2,30}" title="only 0-9A-Za-z/  2-30 char" required> <br>
                <input type="submit" value="Submit">
				<br><small>topic</small><b>/</b><small>function</small><b>/</b><small>MAC</small><b>/set</b> <small>[data]</small>
				<br><small>topic</small><b>/</b><small>function</small><b>/</b><small>MAC</small><b>/status</b> <small>[data]</small>
				<br><small>topic</small><b>/heartbeat</b> <small>[data]</small>
				<br><small>topic</small><b>/topo</b> <small>[data]</small>
				
            </p>
            </form>
             <form id="form" action="./settingsmesh.cgi" method="post">
			<h4>MQTT URL</h4>
            <p>
                mqtt:// prefix is mandatory <br>
                <input type="text" name="mqtturl" value="%mqtturl%" pattern="[0-9A-Za-z/-:.]{2,64}" title="only 0-9A-Za-z/-:.  2-64 char" required> <br>
                <input type="submit" value="Submit">
            </p>
            </form>
			<form id="form" action="./settingsmesh.cgi" method="post" >
			<h4>ESPMESH SETTINGS</h4>
            <p>
				ID: <br>
                <input type="text" name="mesh_id" value="%meshid%" id="txt_essid" pattern="[0-9A-Za-z]{6}" title="only 0-9A-Za-z  6 char" required> <br>
                Password: <br>
				<input type="password" name="mesh_password" pattern="[0-9A-Za-z]{2,63}" title="only 0-9A-Za-z  2-63 char" required> <br>
                <input type="submit" value="Submit">
            </p>
            </form>
        </div>   
		<div id="borders">
            <form id="form" action="./settings.cgi" method="post">
			<h4>Factory login</h4>
            <p>
				Login: <b>admin</b> <br>
				Password: <br>
                <input type="password" name="conf_password" pattern="[0-9A-Za-z]{2,32}" title="only 0-9A-Za-z  2-32 char" required> <br>
                <input type="submit" value="Submit">
            </p>
            </form>
             <form id="form" action="./settings.cgi" method="post">
			<h4>Debug mode</h4>
            <p>
                set number 0-6<br>
                None, Error, Warning, Info, Debug, Verbose, Memory debug<br>
                <input type="text" name="debugmode" value="%memdebug%" pattern="[0-6]{1}" title="only 0-6  1 char" required> <br>
                <input type="submit" value="Submit">
            </p>
            </form>
        </div>
    </div>
</body></html>

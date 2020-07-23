#APP partition

Program code responsable for reading HLW sensor data and MQTT communication.
Must be flashed into app partition using flashing tool or factory partition interface.

###MQTT
| Command example | Description |
| --- | --- |
|gBridge/u1/onoff/{device MAC}/set {1 or 0}	| relay control |
|gBridge/u1/factory/{device MAC}/set	{SHA256 password with salt}	| factory reset command, sending command without or with incorrect password will invoke respons with currently used salt|


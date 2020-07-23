# APP partition

Program code responsable for reading HLW sensor data and MQTT communication.

Must be flashed into app partition using flashing tool or factory partition interface.

### MQTT
| Command example | Description |
| --- | --- |
|gBridge/u1/onoff/**{device MAC}**/set **{1 or 0}**	| relay control |
|gBridge/u1/factory/**{device MAC}**/set	**{SHA256 password salted}**	| factory reset command, sending command without or with incorrect password will invoke respons with currently used salt|

| MQTT respons | Description | Called upon |
| --- | --- | --- |
| gBridge/u1/topo **{topology JSON}** | topology of mesh network | changed in the MESH |
| gBridge/u1/heartbeat **{JSON data}** | heartbeat with sensor data | every 3 seconds|
| gBridge/u1/factory/**{device MAC}**/status **{uint32 password salt}** | randomly generated password salt | after receiving factory reset command |
| gBridge/u1/onoff/**{device MAC}**/status **{1 or 0}** | relay state | on relay state change |

### Heartbeat JSON data example 
```
{"self": "3c71bf748d4c", "parent":"f4e3fb6b579e","layer":1,"itemp":62,"V":2.228804e+00,"A":2.680833e-02,"W":0.000000e+00,"kwh":0.000000e+00}
```
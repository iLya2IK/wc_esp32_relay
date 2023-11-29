# HTTP2 Thermo Relay Client Device
The client is designed to work in conjunction with the [REST Web Camera server](https://github.com/iLya2IK/wcwebcamserver).
Device configuration via BLE.

Electronic schematic for current project

![alt text](https://github.com/iLya2IK/wc_esp32_relay/blob/main/esp32_rele_plug_top.png?raw=true)
![alt text](https://github.com/iLya2IK/wc_esp32_relay/blob/main/esp32_rele_plug_btm.png?raw=true)

# Protocol description
Data exchange between devices is carried out according to the HTTP/2 protocol using the POST method. The contents of requests and responses are JSON objects. The description for JSON requests/respones inside sub-protocol you can found [here](https://github.com/iLya2IK/wcwebcamserver/wiki).

# Sub-protocol description
The description of the subprotocol for TRelay can be found in the files:

```
Relay_WCProtoDescription_en.pdf
Relay_WCProtoDescription_ru.pdf
```

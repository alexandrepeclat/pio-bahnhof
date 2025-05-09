### TODO SOFTWARE :
```
 TODO virer les deps inutiles (CRC32 utilisé par librairie)
 TODO voir pour les conflits avec dépendances de command-library
 TODO réorganiser la détection d'erreurs.
- on a emergencyStop et assertThis un peu interchangeables
- on a des checks sur des getters ou à des moments dans la logique du code et dans les fonctions dédiées...
- le state, flag et message sont un peu redondants
 TODO passer les strings en mémoire flash plutôt qu'en RAM 
 TODO s'assurer que tous les "getters" utilisés notamment dans les debug ne change pas les états ou valeurs :-)
 TODO Vérifier les valeurs passées depuis l'api un peu mieux en terme de limites
 TODO sécuriser API ?
```

### Flash to board

- Flash the software : `pio run --target upload`
- Flash static html file : `pio run --target uploadfs`

### Setting up the pulse offset

- Do manual calibration
- Set default panel as the actual panel
    - You should have the same panel on the device and UI
- Move pulse by pulse until the panel changes on the device
    - You may have a different panel on the device and UI
- Tune the pulse offset until both device and UI are on the same panel
    - If the UI shows the next panel : increase pulse offset
    - If the UI shows the previous panel : decrease pulse offset
- Save the settings !

### Hardware

| Ref         | Description                                  | V(in)    | V(out) | Datasheet                                                      |
|-------------|----------------------------------------------|----------|--------|----------------------------------------------------------------|
| LM2596      | Step-down voltage regulator (Buck converter) | 7-30V    | 5.2V   | https://www.ti.com/product/LM2596                              |
| NodeMCU v3  | ESP8266 module                               | 5-9V     |        | https://github.com/nodemcu/nodemcu-devkit-v3.0                 |
| 38S6G5      | Optical encoder (360 steps/rev * 4 pulses)   | 5-24V    |        | https://www.mantech.co.za/Datasheets/Products/e38s_jz-yeba.pdf |
| MG996R      | Continuous rotation servo motor              | 4.8-6.6V |        | https://www.towerpro.com.tw/product/mg996r                     |
| KY-010      | Optical sensor                               | 3.3-5V   |        | https://sensorkit.joy-it.net/fr/sensorsky-010                  |

![Breadboard scheme](refs/electronics_bb.png)
![Electro scheme](refs/electronics_scheme.svg)


### Mechanical stuff

https://geargenerator.com/beta/#hk3pLkIpT1P$9k5L5mjM4YitFARO9K3T5M8U4yJl4H8tBo4McAjLb9iubwjmh9Cub5pTchRmcYpu7AROJcRNgD3tKqeNHM3WisjGvXcw6DQBUl1fZZkQOW5HZKNNzp3rGI1YZ$7LastDekNFMcbxdNVGW7Z1knDcZkTHZKQoYbuqFSH2Y25WZJh$WVwuaH@FJE3

![Gears scheme](refs/gears-scheme.png)

> Notes : 
>
> - Pulses number / panel must be an integer. This is why we have "clean" number of teeth for both gears, related to what is connected to them : 
>   - 62 teeth for panels (divisor of 62 panels, but could be 31, 124...)
>   - 60 teeth for encoder (divisor of 360 steps, but could be 36, 40, 72)
>   - The actual pulses number per panel is `360 steps / rev * 4 pulses / step * (1 / 60 rev) = 24 pulses / panel`
> - However we do not need to have 1 panels full revolution to be equal to 1 encoder revolution
>   - The actual encoder speed is `60/62 = 0.96` of the panels speed 
>   - For each panels full revolution, we have `62/60 = 1.0333` encoder revolutions

### REST call examples

```
curl -Method Get -Uri "http://192.168.0.222/stop"
curl -Method Get -Uri "http://192.168.0.222/panel"
curl -Method Post -Uri "http://192.168.0.222/moveToPanel" -Body @{panel=6} -ContentType "application/x-www-form-urlencoded"
curl -Method Post -Uri "http://192.168.0.222/advancePanels" -Body @{count=2} -ContentType "application/x-www-form-urlencoded"
curl -Method Post -Uri "http://192.168.0.222/setCurrentPanel" -Body @{panel=0} -ContentType "application/x-www-form-urlencoded"
```

### Useful doc
- Split flap wall decoration SBB/CFF : https://commerce.sbb.ch/en/split-flap-display-wall-decoration-different-special-trains-1100.html
- Projet split flap diy : https://github.com/davidkingsman/split-flap
- Lib encoder esp32 (multi modes dont quadrature, avec interruptions) : https:github.com/madhephaestus/ESP32Encoder 
- Exemple encoder (pas quadrature mais directionnel) : https:github.com/sandy9159/How-to-connect-optical-rotary-encoder-with-Arduino
- Lib encodeur RPi Pico : https://github.com/GitJer/Some_RPI-Pico_stuff/tree/main/Rotary_encoder
- Lib encodeur interruptions : https://github.com/gfvalvo/NewEncoder/tree/master
- Connexion encodeur optique esp32 : https://electricdiylab.com/how-to-connect-optical-encoder-with-esp32/#Wiring_of_Optical_encoder_and_ESP32
- Thread sur lecture encodeur rapide : https://forum.arduino.cc/t/arduino-and-high-speed-rotary-encoders/327931/7
- Thread lecture encodeur state table : https://forum.arduino.cc/t/reading-a-quad-encoder-and-converting-to-a-function/272506/2
- Thread encodeur interruptions : https://stackoverflow.com/questions/51297371/arduino-rpm-code-with-quadrature-encoder-600ppr


### Shitty issues 

- Optical encoder 38s6g5 needs >= 5V https://forum.arduino.cc/t/fyi-interfacing-e38s6g5-600b-g24n-600p-r-rotary-encoder/1057892
- noInterrupt() et interrupt() pas utilisés car perturbent la lecture Serial et serveur HTTP. En principe on en a pas besoin car les variables volatiles sont <= 32 bits donc atomiques sur ESP8266.
- EEPROM.read() and EEPROM.write() only work with 1 single byte. Need to use EEPROM.get() / EEPROM.set() for int.
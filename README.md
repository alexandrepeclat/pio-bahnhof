### TODO MECANIQUE :
```
- Fixation roue encodeur : rondelle <0.8mm ?
- Fixation roue optique : 3x rondelles 16mm/0.8mm
- Espacer les roues imprimées de 0.1-2mm (ou les diminuer de taille) pour avoir une tolérance ?
```

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

### Useful doc

- Lib encoder esp32 (multi modes dont quadrature, avec interruptions) : https:github.com/madhephaestus/ESP32Encoder 
- Exemple encoder (pas quadrature mais directionnel) : https:github.com/sandy9159/How-to-connect-optical-rotary-encoder-with-Arduino

### Shitty issues 

- Optical encoder 38s6g5 needs >= 5V https://forum.arduino.cc/t/fyi-interfacing-e38s6g5-600b-g24n-600p-r-rotary-encoder/1057892
- noInterrupt() et interrupt() pas utilisés car perturbent la lecture Serial et serveur HTTP. En principe on en a pas besoin car les variables volatiles sont <= 32 bits donc atomiques sur ESP8266.
- EEPROM.read() and EEPROM.write() only work with 1 single byte. Need to use EEPROM.get() / EEPROM.set() for int.

### Hardware

| Ref         | Description                                  | V(in)    | V(out) | Datasheet                                                      |
|-------------|----------------------------------------------|----------|--------|----------------------------------------------------------------|
| LM2596      | Step-down voltage regulator (Buck converter) | 7-30V    | 5.2V   | https://www.ti.com/product/LM2596                              |
| NodeMCU v3  | ESP8266 module                               | 5-9V     |        | https://github.com/nodemcu/nodemcu-devkit-v3.0                 |
| 38S6G5      | Optical encoder                              | 5-24V    |        | https://www.mantech.co.za/Datasheets/Products/e38s_jz-yeba.pdf |
| MG996R      | Continuous rotation servo motor              | 4.8-6.6V |        | https://www.towerpro.com.tw/product/mg996r                     |
| KY-010      | Optical sensor                               | 3.3-5V   |        | https://sensorkit.joy-it.net/fr/sensorsky-010                  |

  

### V4 

Roue panneaux

- 62 panneaux
- 48 dents -> 48

Roue optique + changement rapport encodeur : 

- 1x slot
- 62 dents -> 60
- 48 dents <- 48

Roue encodeur : 

- 60 dents <- 62

1 dent des panneaux = 1 dent encodeur

1 panneau = NPLUSES/60 = 360/60 = 6 impulsions

1 tour panneaux / tour encodeur = 62/60 = 1.0333 tour encodeur / tour panneaux

https://geargenerator.com/beta/#ycejT3Oju3chC3vxoKvFoDYolAE8lBbF@FFRGxYK7cW@KgHdCdXMSvvYNALu9OcwXeZrFrNYvp1

Note : 

- le nb d'impulsions doit être entier, c'est pour ça qu'on a entre les panneaux (via roue optique) et l'encodeur des rapports propres à l'élément connecté : 
    - 62 dents pour les panneaux (diviseur de 62, mais pourrait être 31, 124...)
    - 40 dents pour l'encodeur (diviseur de 80, 360, 600...)
- par contre 1 tour de panneaux ne donne pas un tour d'encodeur complet : 62 / 40 = 1,55 tour d'encodeur par tour de panneaux

Diviseurs de 600 : 1, 2, 3, 4, 5, 6, 8, 10, 12, 15, 20, 24, 25, 30, 40, 50, 60, 75, 100, 120, 150, 200, 300, 600

Diviseurs de 360 : 1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 15, 18, 20, 24, 30, 36, 40, 45, 60, 72, 90, 120, 180, 360

### REST call examples

```
curl -Method Get -Uri "http://192.168.0.222/stop"
curl -Method Get -Uri "http://192.168.0.222/panel"
curl -Method Post -Uri "http://192.168.0.222/moveToPanel" -Body @{panel=6} -ContentType "application/x-www-form-urlencoded"
curl -Method Post -Uri "http://192.168.0.222/advancePanels" -Body @{count=2} -ContentType "application/x-www-form-urlencoded"
curl -Method Post -Uri "http://192.168.0.222/setCurrentPanel" -Body @{panel=0} -ContentType "application/x-www-form-urlencoded"
```
### TODO MECANIQUE :

- Inverser capteur optique ? et mettre trou vis supplémentaire ailleurs (je dois faire le trou de toute manière)
- Voir comment fixer roue encodeur + rondelles
  - Spacer imprimé entre roue et base encodeur ?
  - Rondelle entre roue et plaque métal ?
- Espacer les roues imprimées de 0.1-2mm (ou les diminuer de taille) pour avoir une tolérance ?
- Support de carte + couvercle
- Alim externe + fiche
- Support carte max 92*73mm

 ### TODO SOFTWARE :
 TODO ça semble se mettre en veille au bout d'un moment...... mais ça semble réglé
 TODO réorganiser la détection d'erreurs
 - on a emergencyStop et assertThis un peu interchangeables
 - on a des checks sur des getters ou à des moments dans la logique du code et dans les fonctions dédiées...
 TODO s'assurer que tous les "getters" utilisés notamment dans les debug ne change pas les états ou valeurs :-)
 TODO Vérifier les valeurs passées depuis l'api un peu mieux en terme de limites
 TODO API pour donner la liste des villes ? Dépend du matos après-tout par contre on va pas fournir les traductions pour la reconnaissance vocale ?
 TODO sécuriser API ?

### Flasher
- Flasher le programme : `pio run --target upload`
- Flasher le filesystem : `pio run --target uploadfs`

 ### Procédure de setup
- Calibrer
- Définir le panneau par défaut (actuel)
- Avancer pulse par pulse jusqu'à changement du panneau
- Adapter l'offset (ou le panneau)

### Useful doc
- Lib encoder esp32 (multi modes dont quadrature, avec interruptions) : https:github.com/madhephaestus/ESP32Encoder 
- Exemple encoder (pas quadrature mais directionnel) : https:github.com/sandy9159/How-to-connect-optical-rotary-encoder-with-Arduino

### Shitty issues 
- Optical encoder 38s6g5 needs >= 5V https://forum.arduino.cc/t/fyi-interfacing-e38s6g5-600b-g24n-600p-r-rotary-encoder/1057892
- noInterrupt() et interrupt() pas utilisés car perturbent la lecture Serial et serveur HTTP. En principe on en a pas besoin car les variables volatiles sont <= 32 bits donc atomiques sur ESP8266.

### Hardware
lm2596 IN 7-30V
lm2596 OUT 5.2V
NodeMcu v3 (5-9 V)
38s6g5 Optical Encoder (5 - 24 V)
MG996R Servo Motor with continuous rotation (4.8 - 6.6 V)
KY-010 Optical sensor (3.3 - 5 V)



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



### V1-2-3 
Roue panneaux
- 62 panneaux
- 48 dents -> 48

Roue optique + changement rapport encodeur : 
- 1x slot
- 62 dents -> 40
- 48 dents <- 48

Roue encodeur : 
- 40 dents <- 62

1 dent des panneaux = 1 dent encodeur
1 panneau = NPLUSES/40 = 360/40 = 9 impulsions
1 tour panneaux / tour encodeur = 62/40 = 1.0333 tour encodeur / tour panneaux
https://geargenerator.com/beta/#Tdy5@vg5ZvUBcv0QPm0APA5kUacGfBcXk6K7vYVOVbX2$fiup3W95Wy1BaSdYn3EiEUjFSPDvQ5



### REST call examples
curl -Method Get -Uri "http://192.168.0.222/stop"
curl -Method Get -Uri "http://192.168.0.222/panel"
curl -Method Post -Uri "http://192.168.0.222/moveToPanel" -Body @{panel=6} -ContentType "application/x-www-form-urlencoded"
curl -Method Post -Uri "http://192.168.0.222/advancePanels" -Body @{count=2} -ContentType "application/x-www-form-urlencoded"
curl -Method Post -Uri "http://192.168.0.222/setCurrentPanel" -Body @{panel=0} -ContentType "application/x-www-form-urlencoded"

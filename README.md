### TODO MECANIQUE :

- Inverser capteur optique ? et mettre trou vis supplémentaire ailleurs (je dois faire le trou de toute manière)
- Voir comment fixer roue encodeur + rondelles
  - Spacer imprimé entre roue et base encodeur ?
  - Rondelle entre roue et plaque métal ?
- Espacer les roues imprimées de 0.1-2mm (ou les diminuer de taille) pour avoir une tolérance ?
- Support de carte + couvercle
- Alim externe + fiche

 ### TODO SOFTWARE :
 TODO ça semble se mettre en veille au bout d'un moment...... mais ça semble réglé
 TODO réorganiser la détection d'erreurs
 - on a emergencyStop et assertThis un peu interchangeables
 - on a des checks sur des getters ou à des moments dans la logique du code et dans les fonctions dédiées...
 - on a souvent besoin des valeurs de la boucle précédente, donc faudrait les mettre à jour dans une fonction dédiée en fin de boucle
 TODO ? stocker la valeur de l'encodeur dans variable et la remettre à zéro avec l'optique, mais laisser la valeur de l'encodeur originale.
 - ou plutôt afficher la valeur de l'encodeur brute lors du passage de l'optique
 - et faire une variable previousPulses pour voir si une boucle ne loupe pas un step
 - vérifier qu'on a bien atteint le nombre de steps max au moment du passage et afficher un warning sinon
 TODO s'assurer que tous les "getters" utilisés notamment dans les debug ne change pas les états ou valeurs :-)
 TODO état ERROR ? en même temps le flag reste nécessaire car il garantit qu'on peut pas setter à nouveau autre chose via transition ou autre... mais la boucle continue de tourner et calculer des trucs pour rien...
 TODO Vérifier les valeurs passées depuis l'api un peu mieux en terme de limites et type
 TODO API pour donner la liste des villes ? Dépend du matos après-tout par contre on va pas fournir les traductions pour la reconnaissance vocale ?
 TODO API pour donner la liste des commandes possibles ? + Commande pour lister les commandes possibles ?
 TODO Refactorer selon un pattern "Command" ? on a une interface qui impose de retourner un nom (pour route et commande), une méthode d'exécution, une méthode de décodage (voire exécution) de l'appel rest, une méthode de décodage (et validation, voire exécution) de l'appel sérial. ça mixe un peu la logique REST/Serial/action mais plus extensible
 TODO sécuriser API ?


 ### Procédure de calibration
- setupStart
  - Appeler pour démarrer la procédure
  - Va jusqu'au front montant optique
  - Set currentPulses = 0
  - Set defaultPulse = 0
- setupNext
  - Appeler plusieurs fois jusqu'à ce que le panneau change physiquement (non compris)
  - Avance d'une pulse (ou 2 en pratique)
  - Set currentPulses++
- setupEnd 13
  - Appeler une fois que le panneau a changé en indiquant le no du nouveau panneau
  - Set currentPulses += 13 * 24 (on dit au système d'avancer de 13 panels car on n'était pas au panel 0 avant)
  - Set defaultPulse = (13 * 24) - currentPulses (maintenant qu'on sait qu'on est à panel 13 pulse ~0, l'optique était currentPulses avant)

### Useful doc
- Lib encoder esp32 (multi modes dont quadrature, avec interruptions) : https:github.com/madhephaestus/ESP32Encoder 
- Exemple encoder (pas quadrature mais directionnel) : https:github.com/sandy9159/How-to-connect-optical-rotary-encoder-with-Arduino

### Shitty issues 
- Optical encoder 38s6g5 needs >= 5V https://forum.arduino.cc/t/fyi-interfacing-e38s6g5-600b-g24n-600p-r-rotary-encoder/1057892

### Hardware
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

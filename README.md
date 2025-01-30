// TODO MECANIQUE :
//- Inverser capteur optique ? et mettre trou vis supplémentaire ailleurs (je dois faire le trou de toute manière)
//- Roue optique plus fine
//- Orientation encodeur (! câble et trous) + voir si trous sont corrects sur le cad
//- Faciliter l'ajustement de la roue optique
//   - fixation par le haut ?
//   - fentes pour voir le trou ?
//- Aligner fente avec dent car ça coince mieux sur cette position
//- Permettre de découpler les roues ? dur
//  - Roue libre à insérer une fois que tout est en place pour permettre de libérer le mécanisme en manuel) ?
//- Trous de fixation corrects ? l'un était trop étroit
//- Trou axe roue optique trop petit il semble
//- Voir comment fixer AXE roue optique + rondelles
//- Voir comment fixer roue encodeur + rondelles
//  - Spacer imprimé entre roue et base encodeur ?
//  - Trou de la roue pas obligé de passer au travers. le bas peut être plein et bloquer pour pas que la roue remonte le shaft
//  - Rondelle entre roue et plaque métal ?
//- Prévoir ENCORE plus de place pour pins capteur optique
//- Espacer les roues imprimées de 0.1-2mm (ou les diminuer de taille) pour avoir une tolérance ?

// TODO SOFTWARE :
// https://github.com/madhephaestus/ESP32Encoder (complet et avec interruptions)
// https://github.com/sandy9159/How-to-connect-optical-rotary-encoder-with-Arduino (pas quadrature mais directionnel)
// TODO ON DOIT tenir compte d'un certain offset pour savoir si on a atteint le target car quand la boucle stoppe le moteur, c'est déjà dépassé de quelques steps
// - Ou alors voir si on gère le stop dans les interruptions encodeur..... tout est dans les interruptions chais pas trop si c'est bien...
// - Vu que l'optique sette l'encodeur à zéro, si c'est fait dans sa propre interruption moui (mais il y a des bouces sur font falling aussi), si c'est fait dans les interruptions encodeur ça a encore du sens (on prend le rising edge au moment d'un step d'encodeur)
// - Vu que lorsqu'on targette on veut s'arrêter dès qu'on a passé un step d'encodeur, ça peut avoir du sens de le faire dans les interruptions encodeur.........
// TODO ça semble se mettre en veille au bout d'un moment......
// TODO réorganiser la détection d'erreurs
// - on a emergencyStop et assertThis un peu interchangeables
// - on a des checks sur des getters ou à des moments dans la logique du code et dans les fonctions dédiées...
// - on a souvent besoin des valeurs de la boucle précédente, donc faudrait les mettre à jour dans une fonction dédiée en fin de boucle
// TODO ? stocker la valeur de l'encodeur dans variable et la remettre à zéro avec l'optique, mais laisser la valeur de l'encodeur originale.
// - ou plutôt afficher la valeur de l'encodeur brute lors du passage de l'optique
// - et faire une variable previousPulses pour voir si une boucle ne loupe pas un step
// - vérifier qu'on a bien atteint le nombre de steps max au moment du passage et afficher un warning sinon
// TODO s'assurer que tous les "getters" utilisés notamment dans les debug ne change pas les états ou valeurs :-)
// TODO état ERROR ? en même temps le flag reste nécessaire car il garantit qu'on peut pas setter à nouveau autre chose via transition ou autre... mais la boucle continue de tourner et calculer des trucs pour rien...
// TODO Vérifier les valeurs passées depuis l'api un peu mieux en terme de limites et type
// TODO go to rising/falling edges via API ?
// TODO garder les N derniers messages (erreur ou warning...)
// TODO API avec commandes plus poussées sur même endpoint ? par ex pour aider à définir les offsets
// - positionner par incréments de steps le panneau 0 à son tout début
// - lancer moteur jusqu'à l'optique et récupérer la valeur de l'encodeur à ce moment
// - calculer la pulse courante (attention au sens) et les constantes DEFAULT...
// - en tenant compte de l'offset et en remettant l'encodeur à 0 on obtient le panel courant et on peut vérifier si c'est OK
// TODO API pour donner la liste des villes ? Dépend du matos après-tout par contre on va pas fournir les traductions pour la reconnaissance vocale ?



/*

KY-040 Rotary Encoder with Servo Motor
MG996R Servo Motor with continuous rotation

1 impulsion par panneau (quadrature) :
Roue des panneaux : 24 mm, 48 dents
Roue de photodiode : 24 mm, 48 dents
Roue de l'encodeur : 32 mm, 64 dents
Roue du moteur : xx mm, 20 dents
Rapport : 0.75:1

2 impulsions par panneau :
Roue des panneaux : 24 mm, 48 dents
Roue de l'encodeur : 16 mm, 32 dents
Rapport : 1.5:1

4 impulsions par panneau (steps mécaniques) :
Roue des panneaux : 24 mm, 48 dents
Roue de l'encodeur : 8 mm, 16 dents
Rapport : 3:1

 */


# V1-2-3 
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

# V4 
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











curl -Method Get -Uri "http://192.168.0.222/stop"
curl -Method Get -Uri "http://192.168.0.222/getCurrentPanel"
curl -Method Post -Uri "http://192.168.0.222/moveToPanel" -Body @{panel=6} -ContentType "application/x-www-form-urlencoded"
curl -Method Post -Uri "http://192.168.0.222/advancePanels" -Body @{count=2} -ContentType "application/x-www-form-urlencoded"
curl -Method Post -Uri "http://192.168.0.222/setCurrentPanel" -Body @{panel=0} -ContentType "application/x-www-form-urlencoded"

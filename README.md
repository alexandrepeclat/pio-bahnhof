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
1 panneau = NPLUSES/40 impulsions
1 panneau = 600/40 = 15 impulsions
1 panneau = 360/40 = 9 impulsions
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

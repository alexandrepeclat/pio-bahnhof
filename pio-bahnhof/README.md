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

//TODO voir EncoderInterrupt pour utiliser les interruptions
//TODO var currentPanel utile si on a l'encodeur ?
//TODO ! si encodeur négatif quoi alors ? 
//TODO dépassement valeur encodeur ? peu probable si capteur optique vérifier qu'on dépasse pas 60 panels ?

curl -Method Get -Uri "http://192.168.0.222/stop"
curl -Method Get -Uri "http://192.168.0.222/getCurrentPanel"
curl -Method Post -Uri "http://192.168.0.222/moveToPanel" -Body @{panel=6} -ContentType "application/x-www-form-urlencoded"
curl -Method Post -Uri "http://192.168.0.222/advancePanels" -Body @{count=2} -ContentType "application/x-www-form-urlencoded"
curl -Method Post -Uri "http://192.168.0.222/setCurrentPanel" -Body @{panel=0} -ContentType "application/x-www-form-urlencoded"

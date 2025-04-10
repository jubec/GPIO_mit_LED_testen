# GPIO_mit_LED_testen

Dieses Projekt testet die Ansteuerung eines GPIO-Pins mit einer LED auf einem ESP32-Board.  
Ziel ist es, zu erkennen, ob die Pin-Ausgabe funktioniert – z. B. durch einfaches Blinken.

---

## 🧰 Verwendete Hardware

- ESP32 DevKitC V4 (AZ-Delivery oder kompatibel)
- LED mit Vorwiderstand (ca. 220–470 Ohm)
- Steckbrett, Jumperkabel

---

## ⚙️ Software & Umgebung

- PlatformIO mit VS Code
- Framework: Arduino
- Board: `esp32dev` oder passendes Board
- monitor_speed 	= 115200

- Monitor auf CRLF einstellen = Abschluß der Eingabe 

---

## 🔌 Anschlussplan (Beispiel)

| Bauteil | Pin ESP32 |
|--------:|-----------|
| LED +   | GPIO 2    |
| LED –   | GND       |

---

## 🧪 Funktion

1. Alle GPIO werden einzeln Ein/Aus geschaltet, 
- im Monitor wird angezeigt welcher GPIO angesteuert wird
- so kann festgestellt werden ob ein "gültiger" GPIO gewählt wurde
- es macht das finden der GPIO einfacher z.B. bei schlechter Beschriftung

2. Ein GPIO wird im Monitor ausgewählt, und der blinkt bis zum abbrechen
- durch das Blinken ist das Signal von z.B. 5V zu unterscheiden 
- das Signal kann am GPIO gesehen werden und in der Schaltung verfolgt werden
- es kann auch vom Zielpunkt gemessen werden, ob die Verbindung vorhanden ist
- Verbindungsfehler, kalte Lötstellen sind einfacher zu finden

---

## 🚀 Upload & Test

1. Projekt in VS Code mit PlatformIO öffnen
2. Board anschließen
3. Upload starten (`>pio run --target upload`)
4. Seriellen Monitor öffnen – Test läuft

---

## 🖼️ Vorschau 

- Umstellung auf andere Boards denkbar
  main.cpp gesamt an chatGPT schicken und auf entsprechendes Board umbauen lassen
  


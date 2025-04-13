# 🔌 GPIO_mit_LED_testen

Dieses Projekt testet die Ansteuerung eines GPIO-Pins mit einer LED –
beispielhaft umgesetzt auf einem ESP32-Board. 
Ziel ist es, zu erkennen, ob die Pin-Ausgabe funktioniert – z. B. durch einfaches Blinken einer LED.

> ℹ️ Dieses Werkzeug eignet sich für alle, die GPIO-Funktionen testen, unbekannte Pins aufspüren oder neue Boards prüfen möchten – unabhängig vom Erfahrungsstand.

---

## 🧰 Verwendete Hardware

- ESP32 DevKitC V4 (AZ-Delivery oder kompatibel)
- LED **mit Vorwiderstand** (ca. 220–470 Ohm)
- Steckbrett, Jumperkabel
- ***Andere Hardware mit Arduino-Software ist denkbar***

---

## ⚙️ Software & Umgebung

### Variante 1: **PlatformIO mit Visual Studio Code**

- Framework: Arduino
- Board: `esp32dev` oder passendes Board
- Serielle Geschwindigkeit: `monitor_speed = 115200`
- Serieller Monitor: Abschlusszeichen auf `CRLF` stellen

### Variante 2: **Arduino IDE**

Auch in der **Arduino IDE** verwendbar:

➡️ Einfach die Datei `main.cpp` öffnen und den gesamten Inhalt **in eine neue Datei mit dem Namen `GPIO_LED_test.ino`** kopieren. Danach wie gewohnt kompilieren und hochladen.

> Hinweis: Die Arduino IDE erwartet eine `.ino` Datei mit gleichem Namen wie der Sketch-Ordner.

---

## 🔌 Anschlussplan (Beispiel)

| Bauteil | Pin ESP32 |
| ------- | --------- |
| LED +   | GPIO 2    |
| LED –   | GND       |

---

## 🧪 Funktionen

- Es werden alle verfügbaren GPIOs nacheinander aktiviert und deaktiviert.
  - Im seriellen Monitor wird angezeigt, **welcher GPIO aktuell angesteuert** wird.
  - Hilfreich zum Auffinden unbeschrifteter oder schwer zugänglicher Pins.

- Ein einzelner GPIO kann gezielt ausgewählt werden – dieser blinkt dann dauerhaft.
  - Durch das sichtbare Blink-Signal können z. B. folgende Probleme erkannt und in Schaltungen bis zum Endpunkt verfolgt werden:
    - Falsche Verdrahtung
    - Kalte Lötstellen
    - Unterbrochene Verbindungen

---

## 🧭 Zusatzfunktion: Pinsuche bei unbekannten Boards

Gerade bei günstigen Boards ohne klare Beschriftung oder abweichender Pinbelegung hilft dieses Tool dabei, **herauszufinden, welcher physische Pin welcher GPIO-Nummer entspricht**.

- Viele Boards (vor allem günstigere oder No-Name-Modelle, von Arduino bis ESP32) haben **abweichende oder fehlende Beschriftungen**.
- Mit dieser Anwendung lässt sich durch einfaches Umstecken oder Abtasten mit einer LED (z. B. über eine Messspitze) leicht erkennen, **welcher GPIO tatsächlich aktiv ist**.
- Besonders hilfreich, wenn Pinbezeichnungen im Datenblatt und auf der Platine nicht übereinstimmen.
- Keine aufwendige Recherche nötig – einfach starten, LED anschließen, Pin suchen.

> 💡 Wer die typische `for (x+1) GPIO[x]`-Schleife nicht selbst schreiben möchte, findet hier eine einfache und direkt nutzbare Lösung – mit Ausgabe im seriellen Monitor und sichtbarem LED-Signal.

> Tipp: Das Tool ersetzt keine Pinout-Grafik, hilft aber beim Verifizieren von Beschriftung oder beim Aufspüren unbekannter Boards.

---

## 🚀 Upload & Test

### In PlatformIO:

1. Projekt in VS Code mit PlatformIO öffnen
2. ESP32 anschließen
3. Upload starten:
   ```bash
   > pio run --target upload
   ```
4. Seriellen Monitor starten
5. Test läuft automatisch

Alternativ kann auch ein neues Projekt angelegt und der Code in eine Datei `main.cpp` eingefügt werden.

---

## 🖼️ Hinweise zur Anpassung

- Das Projekt kann auf andere Boards angepasst werden.
- Den `main.cpp`-Code einfach in [ChatGPT](https://chat.openai.com) einfügen – dort kann eine passende Anpassung für ein anderes Microcontroller-Modell erzeugt werden.
- Das Projekt lässt sich auch auf den ESP8266 oder andere Arduino-kompatible Boards übertragen.

---

## 🛠️ Fehlersuche (Troubleshooting)

Hier findest Du häufige Probleme und Lösungen – von praktischen Stolperfallen bis zu technischen Eigenheiten:

### 🔋 Keine LED-Reaktion?

- **Ist die LED richtig herum angeschlossen?**  
  Die längere Seite ist die Anode (+), die kürzere die Kathode (–).

- ⚠️ **Ist ein Vorwiderstand vorhanden?**  
  Ohne Widerstand kann die LED beschädigt werden oder der ESP32 abstürzen.  
  Da teilweise 5 V am GPIO anliegen können, sollte der Widerstand **mindestens 330 Ohm** betragen.

- **Stromversorgung korrekt?**  
  Überprüfe, ob der ESP32 über USB versorgt wird und ob die LED an GND angeschlossen ist.

### 🧪 Blinkt gar nichts?

- **Serieller Monitor geöffnet?**  
  Ohne geöffneten Monitor kann der ESP32 auf Eingaben warten – oder der Start fehlt.

- **Abschlusszeichen CRLF korrekt eingestellt?**  
  In PlatformIO/Arduino IDE ist `CRLF` als Eingabeabschluss nötig.

- **Falscher GPIO gewählt?**  
  Nicht jeder GPIO ist für Ausgang geeignet (z. B. GPIO 34–39 sind *input only*).

### 🧰 Programm lässt sich nicht hochladen?

- **COM-Port korrekt?**  
  In der Arduino IDE oder VS Code den richtigen COM-Port wählen.

- **Treiber installiert?**  
  Für viele ESP32-Boards braucht man den CP210x oder CH340 Treiber.

- **Boot-Taster nötig?**  
  Manche Boards müssen beim Upload einen Boot-Taster gedrückt halten.

### ⚡ LED blinkt, aber nicht erwartungsgemäß?

- **Ungewollter Reset?**  
  Prüfe die Spannungsversorgung. Blinkendes Verhalten kann auf Neustarts hindeuten.

- **Signal zu schwach?**  
  Bei langen Kabeln oder schlechten Verbindungen leuchtet die LED evtl. nur schwach.

- **Störung durch anderen Sketch?**  
  Stelle sicher, dass kein anderer Code den GPIO beeinflusst (z. B. interner Pullup/ADC).  
  Bei PWM-Signalen kann der Tastgrad so gering sein, dass die LED kaum oder gar nicht sichtbar leuchtet.

### 🤖 Hilfe durch ChatGPT

- **Fehlermeldung im Terminal oder in der Arduino IDE?**  
  Den **Fehlertext einfach kopieren** und bei [ChatGPT](https://chat.openai.com) einfügen. Dort bekommst Du verständliche Erklärungen und mögliche Lösungen – auch in anderen Sprachen.

- **Tipp:** Je mehr Details Du beschreibst (z. B. Board-Typ, verwendete IDE, was Du probiert hast), desto besser kann Dir geholfen werden.


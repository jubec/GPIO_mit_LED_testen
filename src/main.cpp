/*
GPIO-Test- und Analyse-Tool für den ESP32

🔧 Dieses Werkzeug dient der schnellen Analyse und
Funktionsprüfung von GPIOs direkt über den seriellen Monitor.

Ideal geeignet für
Einsteiger, Maker und Entwickler
- zum Aufbau von Schaltungen,
- zum Erkennen von I2C- und SPI-Geräten und
- zum gezielten Test einzelner Pins sowie der Leitungen bis zum Endpunkt.

Anwendung:
Mit einer LED (mit Messspitze) wird am GPIO die Funktion geprüft oder
eine GPIO-Verbindung kann in der Schaltung verfolgt werden.
So lassen sich z. B. falsch gewählte GPIOs,
fehlerhafte Anschlüsse oder Kontaktprobleme schneller erkennen.

✅ Funktionen im Überblick:
Übersichtliche Liste wichtiger ESP32-GPIOs mit Nutzungshinweisen
Automatischer Blinktest aller unkritischen GPIO-Ausgänge
Interaktiver Modus: Nutzer kann GPIO-Nummer eingeben → LED blinkt
I2C-Geräte werden beim Start automatisch erkannt und angezeigt
Einfache SPI-Kommunikation wird getestet (z. B. SD-Karte, OLED)

📐 Voraussetzungen:
ESP32 (z. B. DevkitC, NodeMCU) – andere Geräte: siehe unten
Serielle Verbindung (115200 Baud), Monitor-Eingabe mit CRLF
Ein LED mit > ca. 330 Ohm Vorwiderstand und einer Prüfleitung zu abtasten

🕹 Bedienung:
Startet automatisch mit GPIO-Test und Scan
Danach interaktiver Modus (GPIO-Nummer per Tastatur in Konsole eingeben)
Beenden mit Leertaste (Blinktest) oder 'x' (automatischer Durchlauf)
LED auf Messpunkt halten oder anschliessen

🛠 Anpassung an andere Mikrocontroller:
Dieses Tool ist speziell für den ESP32 entwickelt worden.
Es kann aber leicht an andere Boards angepasst werden, zum Beispiel:
- ESP8266 (z. B. D1 Mini)
- Arduino Uno / Nano / Mega
- Raspberry Pi Pico (mit Arduino-Unterstützung)
- Weitere Plattformen wie Teensy, STM32, Seeeduino usw.

💬 Hilfe gewünscht?
Das Projekt wurde in Zusammenarbeit mit ChatGPT entwickelt.
Die Anpassung an ein anderes Board kann direkt dort erfolgen:
Einfach den kompletten Code bei ChatGPT einfügen
Dazu schreiben: „Bitte passe das GPIO-Testtool für [Boardname] an.“
ChatGPT analysiert die Pins und gibt den passenden Code zurück – auf Wunsch mit Erklärungen.

🌐 Dieses Tool ist bewusst schlank gehalten – keine Bibliotheken, keine Weboberfläche.
Es richtet sich an alle, die GPIOs schnell testen oder erste Hardwarekontakte prüfen möchten.

Auf dass euch eine LED richtig leuchte!
*///////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// 🕒 Zentrale Steuerung aller Delay-Wartezeiten im Programm (ms)
const unsigned long BLINK_DELAY = 500; // Zeit für LED-Blink und Wartepausen in Millisekunden
const int spiCSPin = 5;                // Beispiel: SPI CS-Pin

// Liste der GPIOPins und deren Eigenschaften
struct GpioInfo
{
  int pin;
  const char *hinweis;
  bool testOutput;
};

GpioInfo gpioListe[] = {
    {0, "⚠️ GPIO 0 – Boot-Modus, nur mit Pullup verwenden", true},
    {1, "🔒 GPIO 1 – TX0 (Seriell), beim Flashen belegt", false},
    {2, "✅ GPIO 2 – Oft Onboard-LED (invertiert), Boot-kritisch", true},
    {3, "🔒 GPIO 3 – RX0 (Seriell), beim Flashen belegt", false},
    {4, "✅ GPIO 4 – Normal verwendbar", true},
    {5, "✅ GPIO 5 – Normal verwendbar", true},
    {12, "⚠️ GPIO 12 – Boot-Modus empfindlich", true},
    {13, "✅ GPIO 13 – Normal verwendbar", true},
    {14, "✅ GPIO 14 – Normal verwendbar", true},
    {15, "✅ GPIO 15 – Normal verwendbar", true},
    {16, "✅ GPIO 16 – Normal verwendbar", true},
    {17, "✅ GPIO 17 – Normal verwendbar", true},
    {18, "✅ GPIO 18 – PWM, SPI MOSI", true},
    {19, "✅ GPIO 19 – PWM, SPI MISO", true},
    {21, "✅ GPIO 21 – I2C SDA", true},
    {22, "✅ GPIO 22 – I2C SCL", true},
    {23, "✅ GPIO 23 – SPI SCLK", true},
    {25, "✅ GPIO 25 – DAC1", true},
    {26, "✅ GPIO 26 – DAC2", true},
    {27, "✅ GPIO 27 – ADC, PWM", true},
    {32, "✅ GPIO 32 – ADC", true},
    {33, "✅ GPIO 33 – ADC", true},
    {34, "ℹ️ GPIO 34 – Nur Eingang", false},
    {35, "ℹ️ GPIO 35 – Nur Eingang", false},
    {36, "ℹ️ GPIO 36 – ADC, Eingang", false},
    {37, "ℹ️ GPIO 37 – Nur Eingang", false},
    {38, "ℹ️ GPIO 38 – Nur Eingang", false},
    {39, "ℹ️ GPIO 39 – ADC, Eingang", false}};

// 🔍 Einmaliger I2C-Scan beim Setup
void scanI2C()
{
  byte error, address;
  int nDevices = 0;

  Serial.println("\n🔍 Starte I2C-Scan...");
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("✅ I2C-Gerät gefunden bei Adresse 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("⚠️ Fehler bei Adresse 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("❌ Keine I2C-Geräte gefunden.");
  }
  Serial.println("✅ I2C-Scan abgeschlossen.\n");
} // Ende scanI2C()

// 🔍 Einfacher SPI-Test
void testSPI(int csPin)
{
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
  delay(BLINK_DELAY);

  Serial.println("🔍 Teste SPI-Verbindung...");

  digitalWrite(csPin, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  byte response = SPI.transfer(0xFF); // Dummy
  SPI.endTransaction();
  digitalWrite(csPin, HIGH);

  Serial.print("📨 SPI-Antwort: ");
  Serial.println(response, HEX);
  if (response != 0xFF)
  {
    Serial.println("✅ Möglicherweise Gerät erkannt. SPI genauer prüfen!");
  }
  else
  {
    Serial.println("❌ Keine sinnvolle Antwort von SPI-Geräten.");
  }
} // Ende testSPI()

// 🔧 Interaktiver Blink-Test für GPIOs
void interaktiveBlinkTests()
{
  Serial.println("\n🔧 Interaktiver Blink-Test:");
  Serial.println("Gib eine GPIO-Nummer ein (z.B. 2 oder 26) und drücke Enter.");
  Serial.println("Mit der Leertaste beendest du den Blink-Modus.");

  while (true)
  {
    Serial.println("\nEingabe: ");
    String eingabe = "";
    unsigned long startWartezeit = millis();

    while ((millis() - startWartezeit < 10000) && Serial.available() == 0)
    {
      delay(10);
    }

    if (Serial.available() == 0)
    {
      Serial.println("Keine Eingabe erkannt. Zurück in den GPIO An/Aus Loop.");
      Serial.println("Drücke 'x' um den GPIO-Durchlauf zu beenden.");
      break;
    }

    while (true)
    {
      if (Serial.available())
      {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
          break;
        eingabe += c;
        Serial.print(c);
      }
    }

    Serial.println();
    eingabe.trim();
    if (eingabe == "")
    {
      Serial.println("Interaktiver Blink-Modus beendet.");
      break;
    }
    if (eingabe.equalsIgnoreCase("x"))
    {
      Serial.println("❌ Zurück zum Hauptmenü.");
      break;
    }
    int blinkPin = eingabe.toInt();
    if (blinkPin < 0 || blinkPin > 39)
    {
      Serial.println("❌ Ungültige GPIO-Nummer!");
      continue;
    }

    bool erlaubt = false;
    for (int i = 0; i < sizeof(gpioListe) / sizeof(GpioInfo); i++)
    {
      if (gpioListe[i].pin == blinkPin && gpioListe[i].testOutput)
      {
        erlaubt = true;
        break;
      }
    }

    if (!erlaubt)
    {
      Serial.print("❌ GPIO ");
      Serial.print(blinkPin);
      Serial.println(" darf nicht verwendet werden!");
      continue;
    }

    Serial.print("⚡ GPIO ");
    Serial.print(blinkPin);
    Serial.println(" blinkt jetzt. Leertaste = Stop.");
    pinMode(blinkPin, OUTPUT);

    while (true)
    {
      digitalWrite(blinkPin, LOW);
      delay(BLINK_DELAY / 2);
      digitalWrite(blinkPin, HIGH);
      delay(BLINK_DELAY / 2);

      if (Serial.available())
      {
        char stop = Serial.read();
        if (stop == ' ')
        {
          Serial.print("🛑 Blink-Modus für GPIO ");
          Serial.print(blinkPin);
          Serial.println(" beendet.");
          digitalWrite(blinkPin, LOW);
          break;
        }
      }
    }
  }
} // Ende interaktiveBlinkTests()

// 🔧 Hauptsetup-Funktion
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n🚀 Starte erweiterten GPIO-Test mit I2C/SPI-Anzeige...");

  Wire.begin(); // SDA = GPIO21, SCL = GPIO22 (ESP32 Standard)
  SPI.begin();  // SPI starten

  scanI2C();         // Alle I2C-Geräte anzeigen
  testSPI(spiCSPin); // Einfachen SPI-Test
  delay(3000);       // Wartezeit für I2C/SPI-Scan
  interaktiveBlinkTests();
} //  Ende Setup-Funktion

// 🔧 Hauptloop-Funktion
void loop()
{
  for (int i = 0; i < sizeof(gpioListe) / sizeof(GpioInfo); i++)
  {
    if (Serial.available())
    {
      char abbrechTaste = Serial.read();
      if (abbrechTaste == 'x' || abbrechTaste == 'X')
      {
        Serial.println("\n❌ GPIO-Durchlauf abgebrochen – zurück zum interaktiven Modus.");
        break;
      }
    }

    int pin = gpioListe[i].pin;
    Serial.println();
    Serial.print("🔎 Teste GPIO ");
    Serial.print(pin);
    Serial.print(": ");
    Serial.println(gpioListe[i].hinweis);

    if (gpioListe[i].testOutput)
    {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
      delay(BLINK_DELAY / 2);
      digitalWrite(pin, HIGH);
      delay(BLINK_DELAY / 2);
      digitalWrite(pin, LOW);
    }
    else
    {
      Serial.println("⏭️ Kein Output-Test");
      delay(BLINK_DELAY);
    }
  }

  Serial.println("\n📅 GPIO-Test abgeschlossen.");
  interaktiveBlinkTests();
  delay(100);
} // Ende Loop-Funktion

/*
Dieser LED_test kann:
- einzelne GPIOs des ESP32 zum Blinken bringen 
- alle GPIOs des ESP32 ansteuern
- eine Liste der GPIOs mit Hinweisen ausgeben

So kann mit einem LED (mit Vorwiederstand ca 330 Ohm) 
an Punkten in der Schaltung erkannt werden, ob ein Signal ankommt 
- weil alle GPIOs angesteuert werden
- oder ein ausgewählter GPIO Blinksignale sendet (im Unterschied zum 5V Pin)
*/

#include <Arduino.h>

struct GpioInfo
{
  int pin;
  const char *hinweis;
  bool testOutput;
};

GpioInfo gpioListe[] = {
    {0, "⚠️ GPIO 0 – Boot-Modus abhängig, nur mit Pullup verwenden", true},
    {1, "🔒 GPIO 1 – TX0 (Seriell), beim Flashen belegt", false},
    {2, "✅ GPIO 2 – Oft Onboard-LED (invertiert), Boot-kritisch", true},
    {3, "🔒 GPIO 3 – RX0 (Seriell), beim Flashen belegt", false},
    {4, "✅ GPIO 4 – Normal verwendbar, PWM, ADC", true},
    {5, "✅ GPIO 5 – Normal verwendbar, PWM, ADC", true},
    {12, "⚠️ GPIO 12 – Boot-Modus empfindlich, nur mit PullDown", true},
    {13, "✅ GPIO 13 – Normal verwendbar, PWM, ADC", true},
    {14, "✅ GPIO 14 – Normal verwendbar, PWM, ADC", true},
    {15, "✅ GPIO 15 – Normal verwendbar, PWM, ADC", true},
    {16, "✅ GPIO 16 – Normal verwendbar, PWM", true},
    {17, "✅ GPIO 17 – Normal verwendbar, PWM", true},
    {18, "✅ GPIO 18 – PWM, SPI MOSI", true},
    {19, "✅ GPIO 19 – PWM, SPI MISO", true},
    {21, "✅ GPIO 21 – I2C SDA", true},
    {22, "✅ GPIO 22 – I2C SCL", true},
    {23, "✅ GPIO 23 – PWM, SPI SCLK", true},
    {25, "✅ GPIO 25 – DAC1", true},
    {26, "✅ GPIO 26 – DAC2", true},
    {27, "✅ GPIO 27 – PWM, ADC", true},
    {32, "✅ GPIO 32 – ADC", true},
    {33, "✅ GPIO 33 – ADC", true},
    {34, "ℹ️ GPIO 34 – Nur Eingang, ADC", false},
    {35, "ℹ️ GPIO 35 – Nur Eingang, ADC", false},
    {36, "ℹ️ GPIO 36 – Nur Eingang, ADC (Sensor Voltage Positive)", false},
    {37, "ℹ️ GPIO 37 – Nur Eingang (nicht immer vorhanden)", false},
    {38, "ℹ️ GPIO 38 – Nur Eingang (nicht immer vorhanden)", false},
    {39, "ℹ️ GPIO 39 – Nur Eingang, ADC (Sensor Voltage Negative)", false}};

// Diese Funktion erlaubt es, in den interaktiven Blink-Modus zu gehen,
// in dem du eine oder mehrere PIN-Nummern (als Zahleneingabe) eingeben kannst.
// Für jede gültige PIN startet der Blinkmodus (100ms an/aus), bis du mit der Leertaste beendest.
void interaktiveBlinkTests()
{
  Serial.println("\n🔧 Interaktiver Blink-Test:");
  Serial.println("Gib eine PIN-Nummer ein (z.B. 2 oder 26) und drücke Enter,");
  Serial.println("um diese schnell blinken zu lassen.");
  Serial.println("Drücke Leertaste als eigenständige Eingabe, um den interaktiven Modus zu verlassen.");

  while (true)
  {
    Serial.println("\nEingabe: ");
    String eingabe = "";
    unsigned long startWartezeit = millis();

    // Warte maximal 10 Sekunden auf eine Eingabe (oder solange nichts ankommt, kann man auch endlos warten)
    while ((millis() - startWartezeit < 10000) && (Serial.available() == 0))
    {
      delay(50);
    }

    // Falls nichts eingegeben wurde, brechen wir ab und kehren in den Loop zurück
    if (Serial.available() == 0)
    {
      Serial.println("Keine Eingabe, zurück in den normalen Loop.");
      break;
    }

    // Lese alle Zeichen der Eingabe (bis zum Zeilenumbruch)
    while (Serial.available())
    {
      char c = Serial.read();
      if (c == '\n' || c == '\r')
        break;
      eingabe += c;
      delay(10); // kleine Pause, um das Buffering zu unterstützen
    }

    // Falls der Benutzer nur ein Leerzeichen eingegeben hat, verlassen wir den interaktiven Modus.
    eingabe.trim(); // Entfernt führende und endende Leerzeichen
    if (eingabe == "")
    {
      Serial.println("Interaktiver Blink-Modus beendet.");
      break;
    }

    int blinkPin = eingabe.toInt();
    if (blinkPin <= 0)
    {
      Serial.println("Ungültige PIN-Eingabe. Bitte eine Zahl > 0 eingeben.");
      continue;
    }

    // Ausgabe, dass der Blinkmodus für die angegebene PIN gestartet wird
    Serial.print("⚡ GPIO ");
    Serial.print(blinkPin);
    Serial.println(" blinkt jetzt schnell (100ms an/aus).");
    Serial.println("Drücke Leertaste, um diesen Blink-Modus zu beenden.");

    // Setze den Pin als Output
    pinMode(blinkPin, OUTPUT);

    // Blinkmodus: Blinke so lange, bis eine Leertaste gedrückt wird.
    while (true)
    {
      digitalWrite(blinkPin, LOW); // Bei invertierten Schaltungen: LOW = LED an
      delay(100);
      digitalWrite(blinkPin, HIGH); // HIGH = LED aus
      delay(100);

      if (Serial.available())
      {
        String stopEingabe = Serial.readStringUntil('\n');
        stopEingabe.trim();

        if (stopEingabe == "" || stopEingabe == " ")
        {
          Serial.print("Blink-Modus für GPIO ");
          Serial.print(blinkPin);
          Serial.println(" beendet.");
          digitalWrite(blinkPin, HIGH); // aus
          break;
        }
      }
    }


    
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("  ");
  Serial.println("🚀 Starte erweiterten GPIO-Test mit Hinweisen");

  interaktiveBlinkTests(); // Aufruf des interaktiven Blink-Test-Modus
}

void loop()
{
  for (int i = 0; i < sizeof(gpioListe) / sizeof(GpioInfo); i++)
  {
    int pin = gpioListe[i].pin;
    Serial.println();
    Serial.print("🔎 Teste GPIO ");
    Serial.print(pin);
    Serial.print(": ");
    Serial.println(gpioListe[i].hinweis);

    if (gpioListe[i].testOutput)
    {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW); // LED an (falls invertiert)
      delay(500);
      digitalWrite(pin, HIGH); // LED aus
      delay(500);
      digitalWrite(pin, LOW); // Zurücksetzen
    }
    else
    {
      Serial.println("⏭️  Kein Output-Test (Eingang oder reserviert)");
      delay(500);
    }
  }

  Serial.println("\n✅ GPIO-Test abgeschlossen. ");
  interaktiveBlinkTests(); // Aufruf des interaktiven Blink-Test-Modus
  delay(100);
}

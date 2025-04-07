/*
LED GPIO-Testtool für den ESP32
----------------------------------
Dieser LED-Test kann:
- einzelne GPIOs des ESP32 zum Blinken bringen
- alle GPIOs des ESP32 ansteuern
- eine Liste der GPIOs mit Hinweisen ausgeben

Hinweis:
- Verwende einen Vorwiderstand (ca. 330 Ohm) mit der LED
- Test auch an offenen Kontakten (Spannungserkennung)
*/

#include <Arduino.h>

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

    // Warte bis zu 10 Sekunden auf Eingabe
    while ((millis() - startWartezeit < 10000) && Serial.available() == 0)
    {
      delay(50);
    }

    if (Serial.available() == 0)
    {
      Serial.println("Keine Eingabe erkannt. Zurück in den normalen Loop.");
      break;
    }

    // Eingabe lesen
    while (Serial.available())
    {
      char c = Serial.read();
      if (c == '\n' || c == '\r')
        break;
      eingabe += c;
      delay(10);
    }

    eingabe.trim();
    if (eingabe == "")
    {
      Serial.println("Interaktiver Blink-Modus beendet.");
      break;
    }

    int blinkPin = eingabe.toInt();
    if (blinkPin < 0 || blinkPin > 39)
    {
      Serial.println("❌ Ungültige GPIO-Nummer!");
      continue;
    }

    // Nur freigegebene GPIOs zulassen
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
      delay(100);
      digitalWrite(blinkPin, HIGH);
      delay(100);

      if (Serial.available())
      {
        String stopEingabe = Serial.readStringUntil('\n');
        stopEingabe.trim();

        if (stopEingabe == "" || stopEingabe == " ")
        {
          Serial.print("🛑 Blink-Modus für GPIO ");
          Serial.print(blinkPin);
          Serial.println(" beendet.");
          digitalWrite(blinkPin, HIGH);
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
  Serial.println("\n🚀 Starte erweiterten GPIO-Test mit Hinweisen...");
  interaktiveBlinkTests();
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
      digitalWrite(pin, LOW);
      delay(500);
      digitalWrite(pin, HIGH);
      delay(500);
      digitalWrite(pin, LOW);
    }
    else
    {
      Serial.println("⏭️ Kein Output-Test");
      delay(500);
    }
  }

  Serial.println("\n📅 GPIO-Test abgeschlossen.");
  interaktiveBlinkTests();
  delay(100);
}

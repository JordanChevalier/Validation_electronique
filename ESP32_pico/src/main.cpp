#include <Arduino.h>

// Définition des broches
#define LED_VERTE 14    // CMD_LED_GREEN
#define LED_ROUGE 15    // CMD_LED_RED
#define BUZZER    13    // Buzzer-PWM

// Canal PWM pour le buzzer
#define BUZZER_CHANNEL 0
#define PWM_FREQUENCY  2000    // 2 kHz
#define PWM_RESOLUTION 8       // 8 bits = 0-255

void setup() {
  // Configuration LED en sortie classique
  pinMode(LED_VERTE, OUTPUT);
  pinMode(LED_ROUGE, OUTPUT);

  // Initialisation de la PWM pour le buzzer
  ledcSetup(BUZZER_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(BUZZER, BUZZER_CHANNEL);

  // Éteint tout au départ
  digitalWrite(LED_VERTE, LOW);
  digitalWrite(LED_ROUGE, LOW);
  ledcWrite(BUZZER_CHANNEL, 0); // pas de signal
}

void loop() {
  // Allume les LEDs
  digitalWrite(LED_VERTE, HIGH);
  digitalWrite(LED_ROUGE, HIGH);

  // Active le buzzer à 50% de duty cycle
  ledcWrite(BUZZER_CHANNEL, 128);  // 128/255 ≈ 50%

  delay(4000); // 4 secondes ON

  // Éteint tout
  digitalWrite(LED_VERTE, LOW);
  digitalWrite(LED_ROUGE, LOW);
  ledcWrite(BUZZER_CHANNEL, 0); // arrête le son

  delay(4000); // 4 secondes OFF
}

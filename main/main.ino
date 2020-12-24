

#include "Adafruit_FONA.h"

#define FONA_RX 11
#define FONA_TX 12
#define FONA_RST 13

char buffer_resposta[256]; // Resposta
char fonaNotificationBuffer[64]; //Notificacions
char smsBuffer[250]; // Lectura sms

void(* resetFunc) (void) = 0;  //declare reset function at address 0

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuf, uint16_t timeout = 0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Prova serial debugging...");

  // we use builtin led for status
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println("No s'ha pogut connectar...");
    configError();
  }
  
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Connectat a FONA correctament!");
  char PIN[5] = "9894"; 
  if (! fona.unlockSIM(PIN)) {
      Serial.println(F("La SIM no s'ha pogut desbloquejar correctament :("));
      configError();
  } else {
      Serial.println(F("El PIN és correcte!"));
  }
}

void configError() {
  while(true) {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);    
    }
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
}

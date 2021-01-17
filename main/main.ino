

#include "Adafruit_FONA.h"

#define FONA_RX 11
#define FONA_TX 12
#define FONA_RST 13

#define STATUS_LED 4
#define TEMP_READ 9
#define CTRL_BUTTON 3
#define RELAY_OUT 5
#define FONA_PS A0
#define FONA_KEY 10

#define PINCODE "0733"

#define DEBUG 1

char buffer_resposta[256]; // Resposta
char fonaNotificationBuffer[64]; //Notificacions
char smsBuffer[250]; // Lectura sms

// For reset button check
unsigned long prevMillisRst = 0;
const long rstBtnInterval = 4000;
int lastBtnState = LOW;

// Mirar-se aquest metode tambe: https://www.codeproject.com/articles/1012319/arduino-software-reset
void(* resetFunc) (void) = 0x0000;  //declare reset function at address 0

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuf, uint16_t timeout = 0);

void setup() {
  #ifdef DEBUG
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Prova serial debugging...");
  #endif

  // we use builtin led for status
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(STATUS_LED, OUTPUT);
  pinMode(CTRL_BUTTON, INPUT); // Possibilitat de posar Hardware Interrupt #1
  pinMode(RELAY_OUT, OUTPUT);

  pinMode(FONA_PS, INPUT);
  pinMode(FONA_KEY, OUTPUT);

  digitalWrite(FONA_KEY, LOW);
  
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    #ifdef DEBUG
    Serial.println("No s'ha pogut connectar...");
    #endif
    configError();
  }
  
  #ifdef DEBUG
  Serial.println("Connectat a FONA correctament!");
  //Verbose errors
  fona.println("AT+CMEE=2");

  uint16_t vbat;
  if (! fona.getBattVoltage(&vbat)) {
    Serial.println(F("Failed to read Batt"));
  } else {
    Serial.print(F("VBat = ")); Serial.print(vbat); Serial.println(F(" mV"));
  }
   
  // SIM Number
  fona.println("AT+CCID");
  while(fona.available()) {
    Serial.write(fona.read());
  }
  #endif
  
  if (! fona.unlockSIM(PINCODE)) {
    #ifdef DEBUG
    Serial.println(F("La SIM no s'ha pogut desbloquejar correctament :("));
    #endif
    configError();
  }
  #ifdef DEBUG 
  else {
    Serial.println(F("El PIN és correcte!"));
  }
  #endif

  // Llegim temperatura i enviem missatge inicialització
  
}

void configError() {
  unsigned long prev_millis_blink = 0;
  const long blink_interval = 1000;
  int blink_state = LOW;
  while(true) {
    unsigned long cTime = millis();
    if(cTime - prev_millis_blink >= blink_interval) {
      prev_millis_blink = cTime;
      if(blink_state == LOW) {
        blink_state = HIGH;
      } else {
        blink_state = LOW;
      }
      digitalWrite(STATUS_LED, blink_state);
    }
    checkBtnResetOnly();
  }
}

void checkBtnResetOnly() {
  unsigned long cTime = millis();
  int buttonState = digitalRead(CTRL_BUTTON);

  if(buttonState != lastBtnState) {
    if(buttonState == HIGH) {
      #ifdef DEBUG
      Serial.println("Comencem a clicar");
      #endif
      // Comencem a pulsar
      prevMillisRst = cTime;
    }
    lastBtnState = buttonState;
  } else {
    if(buttonState == HIGH && (cTime - prevMillisRst) >= rstBtnInterval) {
      #ifdef DEBUG
      Serial.println("4s");
      #endif
      // Comença reset
      lastBtnState = LOW;
      // Reinicialitzem fona
      resetFona();
      resetFunc(); // Posem PC a adreça 0x0 i tornem a iniciar el programa
      
    }
  }
}

void resetFona() {
  if(digitalRead(FONA_PS)) {
    // Fem el cicle sencer de KEY
    digitalWrite(FONA_KEY, HIGH);
    delay(200);
    digitalWrite(FONA_KEY, LOW);
    delay(2500);
    digitalWrite(FONA_KEY, HIGH);
    delay(500);
    digitalWrite(FONA_KEY, LOW);
  } else {
    digitalWrite(FONA_KEY, HIGH);
    delay(200);
    digitalWrite(FONA_KEY, LOW);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
}

void escoltarMissatge(char *info) {

  return;
}

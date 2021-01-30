

#include <Arduino.h>
#include "Adafruit_FONA.h"
#include "OneWire.h"
#include <stdlib.h>
#include <SoftwareSerial.h>

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

#define ALLOWED_TLFS {"689582190", "609120101"}
#define DEBUG_TLF "689582190"

char buffer_resposta[160]; // Resposta
char fonaNotificationBuffer[64]; //Notificacions
char smsBuffer[250]; // Lectura sms

// For reset button check
unsigned long prevMillisRst = 0;
const long rstBtnInterval = 4000;
char lastBtnState = LOW;

// CONTROL D'ESTAT
bool estat = false;

// Mirar-se aquest metode tambe: https://www.codeproject.com/articles/1012319/arduino-software-reset
void(* resetFunc) (void) = 0x0000;  //declare reset function at address 0


SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
OneWire onewire(TEMP_READ);

uint8_t readline(char *buff, uint8_t maxbuf, uint16_t timeout = 0);

int putCalefaccioOn(void);
int putCalefaccioOff(void);

void resetFona(void);
bool mesuraTemp(float *lectura);
void checkBtnResetOnly();

void configError(void);

void setup() {
  #ifdef DEBUG
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F("Prova serial debugging..."));
  #endif

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  pinMode(CTRL_BUTTON, INPUT); // Possibilitat de posar Hardware Interrupt #1
  pinMode(RELAY_OUT, OUTPUT);

  pinMode(FONA_PS, INPUT);
  pinMode(FONA_KEY, OUTPUT);

  pinMode(TEMP_READ, INPUT_PULLUP);  // M'he descuidat d'afegir la resistència pull-up al circuit per el DS18B20, provarem això

  digitalWrite(FONA_KEY, LOW);
  
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    #ifdef DEBUG
    Serial.println(F("No s'ha pogut connectar..."));
    #endif
    configError();
  }
  
  #ifdef DEBUG
  Serial.println(F("Connectat a FONA correctament!"));
  //Verbose errors
  fona.println(F("AT+CMEE=2"));

  uint16_t vbat;
  if (! fona.getBattVoltage(&vbat)) {
    Serial.println(F("Failed to read Batt"));
  } else {
    Serial.print(F("VBat = ")); Serial.print(vbat); Serial.println(F(" mV"));
  }
   
  // SIM Number
  fona.println(F("AT+CCID"));
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

  // Esperem 3 segons, temps de resposta per avtivar després de desbloquejar el pin de la SIM
  delay(3500);
  // I buidem la resposta del desbloqueig ("Call ready... etc")
  fona.flush();
  
  // Llegim temperatura i enviem missatge inicialització
  float tempInicial;
  if(!mesuraTemp(&tempInicial)) {
    configError();
  }

  #ifdef DEBUG
  int8_t smsnum = fona.getNumSMS();
  if (smsnum < 0) {
    Serial.println(F("Could not read # SMS"));
  } else {
    Serial.print(smsnum);
    Serial.println(F(" SMS's on SIM card!"));
  }

  int8_t smsn = 1;
  uint16_t smslen;
  for( ; smsn <= smsnum; smsn++) {
    Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);
    if (!fona.readSMS(smsn, smsBuffer, 250, &smslen)) {  // pass in buffer and max len!
      Serial.println(F("Failed!"));
      break;
    }
    // if the length is zero, its a special case where the index number is higher
    // so increase the max we'll look at!
    if (smslen == 0) {
      Serial.println(F("[empty slot]"));
      smsnum++;
      continue;
    }

    Serial.print(F("***** SMS #")); Serial.print(smsn);
    Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
    Serial.println(smsBuffer);
    Serial.println(F("*****"));
  }

    
  // SIM808 is 1 indexed, no entenc xdd
  for(int i = smsnum; i>0; --i) {
    fona.deleteSMS(i);
  }

  char tmp[8];
  dtostrf(tempInicial, 6, 2, tmp);
  int msg_size = sprintf(buffer_resposta, "Espot-nik iniciat correctament! Temperatura actual:%s", tmp);
  #ifdef DEBUG
    Serial.println(buffer_resposta);
  #endif

  
  if(!fona.sendSMS(DEBUG_TLF, buffer_resposta)) {
    #ifdef DEBUG
    Serial.println(F("No s'ha pogut enviar el missatge d'inicialització"));
    #endif
    configError();
  }
  
  
  #endif
  fonaSerial->print("AT+CNMI=2,1\r\n");
  delay(100);
  putCalefaccioOff();

  fona.flush();
}

bool mesuraTemp(float *lectura) {
  uint8_t data[12];
  int i;

  *lectura = -100.0;

  // enviar comando de iniciar la conversion de temperatura
  // primero generamos pulso de reset
  onewire.reset();
  // enviar el comando skip ROM que selecciona todos los dispositivos en el bus
  onewire.skip();
  // enviar el comando de comienzo de conversion A/D
  onewire.write(0x44);

  // esperar el termino de conversion AD en el sensor
  delay(1000);

  // prestamos atención al pulso de presencia al generar el pulso de reset
  if (!onewire.reset())
    return false;
    
  // enviar el comando skip ROM que selecciona todos los dispositivos en el bus
  onewire.skip();
  // enviar comando de lectura de scratchpad
  onewire.write(0xBE);

  // comenzar lectura de datos
  for (i = 0; i < 9; i++)
    data[i] = onewire.read();

  // alinear los datos recibidos
  int16_t temp = (((int16_t)data[1]) << 11) | (((int16_t)data[0]) << 3);

  // convertir a graus centigrads
  *lectura = (float)temp * 0.0078125;

  return true;
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
      Serial.println(F("Comencem a clicar"));
      #endif
      // Comencem a pulsar
      prevMillisRst = cTime;
    }
    lastBtnState = buttonState;
  } else {
    if(buttonState == HIGH && (cTime - prevMillisRst) >= rstBtnInterval) {
      #ifdef DEBUG
      Serial.println(F("4s"));
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
  #ifdef DEBUG
    Serial.println(F("FONA resettejat"));
  #endif
}

void loop() {
  // LOOP PRINCIPAL

  // 1. Comrpovem estat del boto de control: canvi d'estat o reinici del dispositiu
  unsigned long cTime = millis();
  int buttonState = digitalRead(CTRL_BUTTON);

  if(buttonState != lastBtnState) {
    if(buttonState == HIGH) {
      #ifdef DEBUG
      Serial.println(F("Comencem a clicar"));
      #endif
      // Comencem a pulsar
      prevMillisRst = cTime;
    } else {
      if(estat) {
        putCalefaccioOff();
      } else {
        putCalefaccioOn();
      }
    }
    lastBtnState = buttonState;
  } else {
    if(buttonState == HIGH && (cTime - prevMillisRst) >= rstBtnInterval) {
      #ifdef DEBUG
      Serial.println(F("4s"));
      #endif
      // Comença reset
      lastBtnState = LOW;
      // Reinicialitzem fona
      resetFona();
      resetFunc(); // Posem PC a adreça 0x0 i tornem a iniciar el programa
      
    }
  }

  // 2. Comprovem rebuda de missatges
  char* bufPtr = fonaNotificationBuffer;

  if(fona.available()) {
    int slot = 0;            //this will be the slot number of the SMS
    int charCount = 0;
    //Read the notification into fonaInBuffer
    do  {
      *bufPtr = fona.read();
      
      #ifdef DEBUG
      Serial.write(*bufPtr);
      #endif

      delay(1);
    } while ((*bufPtr++ != '\n') && (fona.available()) && (++charCount < (sizeof(fonaNotificationBuffer)-1)));
    
    //Add a terminal NULL to the notification string
    *bufPtr = 0;

    if (1 == sscanf(fonaNotificationBuffer, "+CMTI: " FONA_PREF_SMS_STORAGE ",%d", &slot)) {
      
      char numRemitent[32];  //we'll store the SMS sender number in here
      
      // Obtenim el numero del remitent
      if (! fona.getSMSSender(slot, numRemitent, 31)) {
        #ifdef DEBUG
        Serial.println(F("Didn't find SMS message in slot!"));
        #endif

        return;

      }
      #ifdef DEBUG
      Serial.print(F("DE: ")); Serial.println(numRemitent);
      #endif

      // Obtenim el text del missatge
      uint16_t smslen;
      fona.readSMS(slot, smsBuffer, 250, &smslen);

      // 3. Tractament del missatge i resposta comanda
      char * comanda = strtok(smsBuffer, " ");
      float lecturaTmp;

      if(strcmp(comanda, "engegar") == 0 || strcmp(comanda, "Engegar") == 0 || strcmp(comanda, "ENGEGAR") == 0) {
        putCalefaccioOn();
        mesuraTemp(&lecturaTmp);
        char tmp[8];
        dtostrf(lecturaTmp, 6, 2, tmp);
        sprintf(buffer_resposta, "Calefacció encesa! Temperatura actual:%sºC", tmp);
        if(!fona.sendSMS(numRemitent, buffer_resposta)) {
          #ifdef DEBUG
          Serial.println(F("No s'ha pogut enviar reposta"));
          #endif
          resetFona();
          resetFunc();
        }
      } else if(strcmp(comanda, "apagar") == 0 || strcmp(comanda, "Apagar") == 0 || strcmp(comanda, "APAGAR") == 0) {
        putCalefaccioOff();
        mesuraTemp(&lecturaTmp);
        char tmp[8];
        dtostrf(lecturaTmp, 6, 2, tmp);
        sprintf(buffer_resposta, "Calefacció apagada! Temperatura actual:%sºC", tmp);
        if(!fona.sendSMS(numRemitent, buffer_resposta)) {
          #ifdef DEBUG
          Serial.println(F("No s'ha pogut enviar reposta"));
          #endif
          resetFona();
          resetFunc();
        }
      } else if(strcmp(comanda, "estat") == 0 || strcmp(comanda, "Estat") == 0 || strcmp(comanda, "ESTAT") == 0) {
        mesuraTemp(&lecturaTmp);
        char tmp[8];
        dtostrf(lecturaTmp, 6, 2, tmp);

        if(estat) {
          sprintf(buffer_resposta, "Ara mateix, la calefacció està encesa i la temperatura actual és%s ºC", tmp);
        } else {
          sprintf(buffer_resposta, "Ara mateix, la calefacció està apagada i la temperatura actual és%s ºC", tmp);
        }
        
        if(!fona.sendSMS(numRemitent, buffer_resposta)) {
          #ifdef DEBUG
          Serial.println(F("No s'ha pogut enviar reposta"));
          #endif
          resetFona();
          resetFunc();
        }
      } else {
        if(!fona.sendSMS(numRemitent, "No t'entenc. Pots enviar-me aixo: engegar, apagar, estat")) {
          #ifdef DEBUG
          Serial.println(F("No s'ha pogut enviar reposta"));
          #endif
          resetFona();
          resetFunc();
        }
      }
      
      
      // 4. delete the original msg after it is processed
      //   otherwise, we will fill up all the slots
      //   and then we won't be able to receive SMS anymore
      if (fona.deleteSMS(slot)) {
        Serial.println(F("OK!"));
      } else {
        Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
        fona.print(F("AT+CMGD=?\r\n"));
        resetFona();
        resetFunc();
      }
    }
    #ifdef DEBUG 
    else {
      // No és una notificacio de Missatge, sudem
      Serial.println(F("Notificacio random rebuda"));
    }
    #endif
  }
}

int putCalefaccioOn() {
  digitalWrite(RELAY_OUT, LOW);
  digitalWrite(STATUS_LED, HIGH);
  estat = true;
  #ifdef DEBUG
    Serial.println(F("Calefaccio encesa"));
  #endif
  return 0;
}

int putCalefaccioOff() {
  digitalWrite(RELAY_OUT, HIGH);
  digitalWrite(STATUS_LED, LOW);
  estat = false;
  #ifdef DEBUG
    Serial.println(F("Calefaccio aturada"));
  #endif
  return 0;
}

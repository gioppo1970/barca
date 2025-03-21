#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

TinyGPSPlus gps;
SoftwareSerial SerialGPS(4, 5); 

int IN1 = D3;
int IN2 = D4;
int IN3 = D5;
int IN4 = D6;
int ENA = D7;
int ENB = D8;

int MAXVELA = 255; // 255
int MINVELA = 215; // 200

int MAXVELB = 255; // 255
int MINVELB = 215; // 200

int msStep = 6000;

// int indirizzo = 0;
  
float distanza = 999999;
float epsilon_distanza = 10; // Delta distanza in metri

// Coordinate di destinazione - MOLO
//float lat_fin = 44.08180100509762;
//float lon_fin = 12.576436065676408;

// Coordinate di destinazione - SUNSET
float lat_fin = 44.07733075852632;
float lon_fin = 12.56973054304142;

// Tempo ultimo aggiornamento dati
unsigned long ultimoaggiornamento = 0;

// Intervallo di aggiornamento
const unsigned int intervallo_aggiornamento = 3000;

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600);

/*
  while (!Serial);
  Serial.print("Leggo la EEPROM");
  double d;
  for (int indloc = 0; indloc < 1024; indloc += sizeof(double))
  {
    EEPROM.get(indloc, d);
    Serial.print(d,8);
    indloc += sizeof(double);
    EEPROM.get(indloc, d);
    Serial.print(",");
    Serial.println(d,8);
  }
*/
    
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT); 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void loop() {
  // Leggo i dati dal modulo GPS

  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      // Controllo se i dati GPS sono validi
      if (gps.location.isValid()) {
/*
  while (1 > 0) {
    if (1 > 0) {
      // Controllo se i dati GPS sono validi
      if (1 > 0) {
*/        
        // Aggiorno ogni intervallo_refresh (in ms)
        if (millis() - ultimoaggiornamento >= intervallo_aggiornamento)
        {
          double lat_att = gps.location.lat();
          double lon_att = gps.location.lng();
          
          ultimoaggiornamento = millis();
/*      
          if (indirizzo > 1024) {
            indirizzo = 0;
          }
          EEPROM.put(indirizzo, lat_att);
          indirizzo += sizeof(double);
          EEPROM.put(indirizzo, lon_att);
          indirizzo += sizeof(double);
*/
          Serial.println("---------");
          Serial.print("Coordinate att:");
          Serial.print(lat_att,8);
          Serial.print(",");
          Serial.println(lon_att,8);
          Serial.print("Coordinate fin:");
          Serial.print(lat_fin,8);
          Serial.print(",");
          Serial.println(lon_fin,8);
          
          // Calcolo la distanza dalla destinazione
          double distanza = gps.distanceBetween(lat_att, lon_att, lat_fin, lon_fin);
          Serial.print("Distanza:");
          Serial.println(distanza);
        
          // Verifico se siamo arrivati a destinazione
          if (distanza <= epsilon_distanza) {
            // Fermo i motori
            StopMotori();
          } else {
            // Calcolo la direzione verso la destinazione
            int angolo = CalcoloAngolo(lat_att, lon_att, lat_fin, lon_fin);
            Serial.print("Angolo:");
            Serial.println(angolo);
            // Implementare il controllo dei motori basato sull'angolo calcolato
            ControlloMotori(angolo);
          }
        }
      }
    }
  }
}

/*
// Funzione che calcola l'angolo di direzione verso la destinazione
double CalcoloAngolo(double lat1, double lon1, double lat2, double lon2) {
  double dLon = (lon2 - lon1) * DEG_TO_RAD;
  lat1 = lat1 * DEG_TO_RAD;
  lat2 = lat2 * DEG_TO_RAD;
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double brng = atan2(y, x) * RAD_TO_DEG;
  return (brng + 360.0) > 360.0 ? brng - 360.0 : brng;
}
*/

// Funzione che calcola l'angolo di direzione verso la destinazione
double CalcoloAngolo(double lat1, double lon1, double lat2, double lon2) {
  double destinazione = TinyGPSPlus::courseTo(lat1, lon1, lat2, lon2);
  const char *direzionedestinazione = TinyGPSPlus::cardinal(destinazione);
  int angolo = (int)(360 + destinazione - gps.course.deg()) % 360;
  Serial.print("Angolo GPS:");
  Serial.println(gps.course.deg());
  return angolo;
}

// Funzione che ferma i motori
void StopMotori() {
  analogWrite(ENB, 0);
  analogWrite(ENA, 0);
}

// Attesa ms
static void smartdelay(unsigned long ms)
{
  unsigned long time_wait = millis() + ms;
  while(millis() < time_wait){
    yield();
  }
}

// Funzione per controllare i motori basata sull'angolo calcolato
void ControlloMotori(int angolo) {
  // Logica di base per controllare i motori
  if (angolo >= 345 || angolo < 15) {
    // Vai dritto
    analogWrite(ENA, MAXVELA);
    analogWrite(ENB, MAXVELB);
    Serial.println("Prosegui dritto.");
    smartdelay(msStep*2
    );
    
  } else if (angolo >= 315 && angolo < 345) {
    // Gira a sinistra
    analogWrite(ENA, MINVELA);
    analogWrite(ENB, MAXVELB);
    Serial.println("Vira leggermente a sinistra.");
    smartdelay(msStep/4);
    
    analogWrite(ENA, MAXVELA);
    analogWrite(ENB, MAXVELB);
    Serial.println("Prosegui dritto.");
    smartdelay(msStep);
    
  } else if (angolo >= 15 && angolo < 45) {
    // Gira a destra
    analogWrite(ENA, MAXVELA);
    analogWrite(ENB, MINVELB);
    Serial.println("Vira leggermente a destra.");
    smartdelay(msStep/4);
    
    analogWrite(ENA, MAXVELA);
    analogWrite(ENB, MAXVELB);
    Serial.println("Prosegui dritto.");
    smartdelay(msStep);

    
  } else if (angolo >= 255 && angolo < 315) {
    // Gira a sinistra
    analogWrite(ENA, MINVELA);
    analogWrite(ENB, MAXVELB);
    Serial.println("Gira a sinistra.");
    smartdelay(msStep/2);
    
    analogWrite(ENA, MAXVELA);
    analogWrite(ENB, MAXVELB);
    Serial.println("Prosegui dritto.");
    smartdelay(msStep);
    
  } else if (angolo >= 45 && angolo < 105) {
    // Gira a destra
    analogWrite(ENA, MAXVELA);
    analogWrite(ENB, MINVELB);
    Serial.println("Gira a destra.");
    smartdelay(msStep/2);
    
    analogWrite(ENA, MAXVELA);
    analogWrite(ENB, MAXVELB);
    Serial.println("Prosegui dritto.");
    smartdelay(msStep);
    
  } else {
    // Gira a sinistra (scelta)
    analogWrite(ENA, MINVELA);
    analogWrite(ENB, MAXVELB);
    Serial.println("Gira completamente.");
    smartdelay(msStep);
    
    analogWrite(ENA, MAXVELA);
    analogWrite(ENB, MAXVELB);
    Serial.println("Prosegui dritto.");
    smartdelay(msStep);
    
  }
}

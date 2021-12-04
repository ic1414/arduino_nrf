#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(10, 9); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};


#define bot1 4
#define bot2 5
#define pinX A1
#define pinY A0
#define pinZ A3
#define pinT A2
// i have sapace, so i do want i want :)
int cx;
int cy;
int cz;
int ct;
int cxyzt[4];

float Kp  = 2.2;
float Ki  = 0.0; // /1000
float Kd  = 170.0;

float Kpz  = 5;
float Kiz  = 0.0; // /1000
float Kdz  = 0.0;

struct nrf {
  byte pwm[4];
  float rpid[3];
  float rpidz[3];
  byte i_clear;
};

struct drone_data {
  float voltages[3];
  byte motorOut[4];
  float cAngles[3];
};

struct nrf radioo;
struct drone_data ack;

void setup() {
  pinMode(bot1, INPUT_PULLUP);
  pinMode(bot2, INPUT_PULLUP);
  radioo.rpid[0] = Kp;
  radioo.rpid[1] = Ki;
  radioo.rpid[2] = Kd;
  radioo.rpidz[0] = Kpz;
  radioo.rpidz[1] = Kiz;
  radioo.rpidz[2] = Kdz;
  radioo.i_clear = 0;

  Serial.begin(115200);
  radio.begin();
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openWritingPipe(addresses[0]);
  radio.openWritingPipe(addresses[1]);
  radio.stopListening();
  radio.setPALevel(RF24_PA_MIN);

  cx = 127 - map(analogRead(A1), 0, 1023, 0, 255);
  cy = 127 - map(analogRead(A0), 0, 1023, 0, 255);
  cz = 127 - map(analogRead(A3), 0, 1023, 0, 255);
  ct = 127 - map(analogRead(A2), 0, 1023, 0, 255);
  cxyzt[0] = cx;
  cxyzt[1] = cy;
  cxyzt[2] = cz;
  cxyzt[3] = ct;
}
void loop() {

  if (Serial.available() > 0) {
    // read the incoming string:
    String spid  = Serial.readString();
    Serial.println(spid);
    if (spid[0] == 'p') {
      int slen = spid.length();
      Serial.println(spid);
      spid = spid.substring(1, slen - 1);
      radioo.rpid[0] = spid.toFloat();
    }

    if (spid[0] == 'i') {
      int slen = spid.length();
      spid = spid.substring(1, slen - 1);
      radioo.rpid[1] = spid.toFloat();
      Serial.println(spid.toFloat());
    }

    if (spid[0] == 'd') {
      int slen = spid.length();
      Serial.println(spid);
      spid = spid.substring(1, slen - 1);
      radioo.rpid[2] = spid.toFloat();
    }

    if (spid[0] == 'a') {
      int slen = spid.length();
      Serial.println(spid);
      spid = spid.substring(1, slen - 1);
      radioo.rpidz[0] = spid.toFloat();
    }

    if (spid[0] == 'b') {
      int slen = spid.length();
      Serial.println(spid);
      spid = spid.substring(1, slen - 1);
      radioo.rpidz[1] = spid.toFloat();
    }

    if (spid[0] == 'c') {
      int slen = spid.length();
      Serial.println(spid);
      spid = spid.substring(1, slen - 1);
      radioo.rpidz[2] = spid.toFloat();
    }
  }

  byte x = map(analogRead(pinX), 0, 1023, 0, 255);
  byte y = map(analogRead(pinY), 0, 1023, 0, 255);
  byte z = map(analogRead(pinZ), 0, 1023, 0, 255);
  byte t = map(analogRead(pinT), 0, 1023, 0, 255);
  int xyzt[4] = {x, y, z, t};
  for (int i = 0; i < 4; i++) {
    xyzt[i] = xyzt[i] + cxyzt[i];
    if (xyzt[i] > 255) {
      xyzt[i] = 255;
    } else {
      if (xyzt[i] < 0 or xyzt[i] == cxyzt[i]) {
        xyzt[i] = 0;
      }
    }
  }
  x = xyzt[0];
  y = xyzt[1];
  z = xyzt[2];
  xyzt[3] = xyzt[3] - 127;
  xyzt[3] = constrain(xyzt[3], 0, 127);
  t = xyzt[3] * 2;
  //t = map(t, 0, 180, 180, 0);
  byte xyzt_byte[4] = {x, y, z, t};


  for (int i = 0; i < 4; i++) {
    radioo.pwm[i] = xyzt_byte[i];
  }


  if (!(digitalRead(bot2))) {
    radioo.i_clear = 1;
  } else {
    radioo.i_clear = 0;
  }

  bool report = radio.write(&radioo, sizeof(radioo));
  if (report) {
    if (radio.available()) {
      radio.read(&ack, sizeof(ack));
    }
  }

  /*
    Serial.print(radioo.pwm[0]);
    Serial.print("   ");
    Serial.print(radioo.pwm[1]);
    Serial.print("   ");
    Serial.print(radioo.pwm[2]);
    Serial.print("   ");
    Serial.print(radioo.pwm[3]);
    Serial.println("   ");
  */



  Serial.print("p:");
  Serial.print(radioo.rpid[0], 3);
  Serial.print("  i:");
  Serial.print(radioo.rpid[1], 3);
  Serial.print("  d:");
  Serial.print(radioo.rpid[2], 3);

  Serial.print("     pz:");
  Serial.print(radioo.rpidz[0], 3);
  Serial.print("  iz:");
  Serial.print(radioo.rpidz[1], 3);
  Serial.print("  dz:");
  Serial.print(radioo.rpidz[2], 3);

  Serial.print("      x:");
  Serial.print(ack.cAngles[0], 3);
  Serial.print("   y:");
  Serial.print(ack.cAngles[1], 3);
  Serial.print("   z:");
  Serial.print(ack.cAngles[2], 3);

  Serial.print("     v1:");
  Serial.print(ack.voltages[0], 2);
  Serial.print("   v2:");
  Serial.print(ack.voltages[1], 2);
  Serial.print("   v3:");
  Serial.print(ack.voltages[2], 2);

  Serial.print("     B:" + String(ack.motorOut[1]) + "  A:" + String(ack.motorOut[0]));
  Serial.println("    D:" + String(ack.motorOut[3]) + "  C:" + String(ack.motorOut[2]));


  delay(10);
}

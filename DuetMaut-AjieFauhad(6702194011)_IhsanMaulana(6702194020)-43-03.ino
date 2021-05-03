// PRAKTIKUM 6 Sistem Kendali PID Kasus P-D dan EEPROM
// KELOMPOK 17 DUET MAUT
// ANGGOTA : AJIE FAUHAD FADHLULLAH (6702194011)
// ANGGOTA : IHSAN MAULANA (6702194020)
#include <EEPROM.h>

int ENA =9;
int ENB =10;

//Encoderin Motor 1
//Pin 2 adalah external interrupt Arduino
#define encoderPinA_1 2
#define encoderPinB_1 4

//Encoder PIN motor 2
#define encoderPinA_2 3
#define encoderPinB_2 11

//PIN Kontrol
#define motor_kiri1 7
#define motor_kiri2 8
#define motor_kanan1 5
#define motor_kanan2 6

//Deklarasi Variabel
//Untuk menampilkan serial port Serial
int encoderMotor1 = 0;
int encoderMotor2 = 0;


int sensor[6];

//Konstanta PID
int baca_sensor[6];

//Initial Speed of Motor
int kecepatanSetPoint = 150;;

//Konstanta PID
float Kp = 1 ;
float Ki = 0 ; 
float Kd = 1 ;

float error = 0,P = 0, I = 0 , D = 0 , PID_value = 0;
float lastError = 0;

//Setting
int state = 0;
int setting = 0;
int peka[6] = {500,500,500,500,500,500};
int kp[6] = {20,20,20,20,20,20};
int kd[6] = {5,5,5,5,5,5};
int alamat=0;
byte value;

/* Digunakan untuk Hardware Arduino Uno
int sensorMax[6] = {1023, 1023, 1023, 1023, 1023, 1023};
int sensorMin[6] = {0, 0, 0, 0, 0, 0};
*/

int sensorMax[6] = {687, 687, 687, 687, 687, 687};
int sensorMin[6] = {33, 33, 33, 33, 33, 33};
int sensorMid[6];
// Deklarasi Pin Sensor Photodiode
int sensor1 = A0;
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int sensor5 = A4;
int sensor6 = A5;

int i;

//Fungsi external interupt
void doEncoderMotor1(){
	digitalRead(encoderPinA_1)?encoderMotor1--:encoderMotor1++;
  
}

void setup(){
	Serial.begin(9600);
    pinMode(encoderPinA_1, INPUT_PULLUP);
  	pinMode(encoderPinB_1, INPUT_PULLUP);
  	pinMode(encoderPinA_2, INPUT_PULLUP);
  	pinMode(encoderPinB_2, INPUT_PULLUP);
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  
  pinMode(motor_kiri1, OUTPUT);
  pinMode(motor_kiri1, OUTPUT);
  pinMode(motor_kiri2, OUTPUT);
  pinMode(motor_kanan1, OUTPUT);
  pinMode(motor_kanan2, OUTPUT);
  pinMode(13, INPUT);
  pinMode(12, INPUT);
  
  //Membaca Pin dari encoderPinA == Sinyal yang memberikan notifikasi pada background
  attachInterrupt(digitalPinToInterrupt(encoderPinA_1), doEncoderMotor1, RISING);
  
  //Membaca Nilai EEFROM
  int nilaisensor = analogRead(sensor1)/4; //Dibagi 4 karena data sensor full bit, EEPROM hanya bernilai 8bit agar menyesuaikan variabel 8bit
  if(alamat <= 512){
  	EEPROM.write(alamat, nilaisensor);
    Serial.print(alamat);
    Serial.print("\t");
    Serial.print(nilaisensor);
    alamat = alamat+1;
  }
  value = EEPROM.read(alamat);
  Serial.print(alamat);
  Serial.print("\t");
  Serial.print(value, DEC);
  Serial.println();
  alamat = alamat+1;
  delay(100);
}

void kalibrasi(){
	sensor[0] = analogRead(sensor1);
  	sensor[1] = analogRead(sensor2);
  	sensor[2] = analogRead(sensor3);
  	sensor[3] = analogRead(sensor4);
  	sensor[4] = analogRead(sensor5);
  	sensor[5] = analogRead(sensor6);
  
  for (i = 5; i >= 0; i--){
    if(sensor[i] > sensorMin[i]){
    	sensorMin[i] = sensor[i];
    }  
     if(sensor[i] < sensorMax[i]){
    	sensorMax[i] = sensor[i];
    }
    sensorMid[i] = (sensorMin[i] + sensorMax[i]/2);
  }
  Serial.println("-----------------------------");
  Serial.println("| Auto Calibration");
   Serial.println("| AjieFauhad_6702194011");
  Serial.println("| Kelas D3TK43-03");
  for(int x=0 ; x<=5 ; x++){
    Serial.print("| Sensor ");
    Serial.print(x);
    Serial.print(": ");
    Serial.print(sensor[x]);
    Serial.println(" ");
  }
}

void read_sensor_values(){
  
  
  /* Membaca EEFROM kalibrasi
  for(int y=0; y<=5 ;y++){
   baca_sensor[y] = sensor[y]; 
  }
  */
  
    baca_sensor[0] = analogRead(sensor1); 
    baca_sensor[1] = analogRead(sensor2); 
    baca_sensor[2] = analogRead(sensor3);
    baca_sensor[3] = analogRead(sensor4); 
    baca_sensor[4] = analogRead(sensor5);
    baca_sensor[5] = analogRead(sensor6); 
 
  // 1 = Gelap 0 = Terang
  // Case 1 0 0 0 0 0
if (baca_sensor[0] < 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34)
    {  
    error = -4;
  Serial.print("\n");
    }
  
  // Case 1 1 0 0 0 0
else if (baca_sensor[0] < 34 && baca_sensor[1] < 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  

    error = -3;
  Serial.print("\n");
   }
  
  // Case 0 1 0 0 0 0
else if (baca_sensor[0] > 34 && baca_sensor[1] < 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  

    error = -2;
  Serial.print("\n");
   }
  
  // Case 0 1 1 0 0 0
else if (baca_sensor[0] > 34 && baca_sensor[1] < 34 && 
    baca_sensor[2] < 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  

   error = -1;
  Serial.print("\n");
   }
  
  // Case 0 0 0 1 0 0
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] < 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  

    error = 0;
  Serial.print("\n");
   }
  
  // Case 0 0 1 1 0 0
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] < 34 && baca_sensor[3] < 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] > 34){  
      
    error = 0;
  Serial.print("\n");
   }
  
  
  // Case 0 0 0 1 1 0
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] < 34 && 
    baca_sensor[4] < 34 && baca_sensor[5] > 34){  

    error = 1;
  Serial.print("\n");
   }
  
  // Case 0 0 0 0 1 0
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] < 34 && baca_sensor[5] > 34){  

    error = 2;
  Serial.print("\n");
   }
  
  // Case 0 0 0 0 1 1
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] < 34 && baca_sensor[5] < 34){  

    error = 3;
  Serial.print("\n");
   }
  
  // Case 0 0 0 0 0 1
else if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34 && baca_sensor[5] < 34){  

    error = 4;
  Serial.print("\n");
   }     

}

void hitungPID(){
  P = error;
  I = error + lastError;
  D = error - lastError;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  lastError = error;
}

// Mengatur Kecepatan Motor
void motor_control()
{
  int kecepatanMotorKiri = kecepatanSetPoint - PID_value;
  int kecepatanMotorKanan = kecepatanSetPoint + PID_value;

  // Kecepetan Motor agar tidak melebihhi batas pwm
  kecepatanMotorKiri = constrain(kecepatanMotorKiri, 0, 255);
  kecepatanMotorKanan = constrain(kecepatanMotorKanan, 0, 255);

  
    Serial.println(kecepatanMotorKanan);
  digitalWrite(ENA, HIGH);
  analogWrite(motor_kiri1, kecepatanMotorKiri);
  analogWrite(motor_kiri2, 0);

  digitalWrite(ENB, HIGH);
  analogWrite(motor_kanan1, kecepatanMotorKanan);
  analogWrite(motor_kanan2, 0);
  
}
               
void loop(){
  if(digitalRead(12) == 0){
  goto bawah;
  }
  
  if(digitalRead(13) == 0){
    while(true){
      atas:
  kalibrasi();
  if(digitalRead(12) == 0){
    while(true){
      bawah:
  read_sensor_values();
  hitungPID();
   motor_control();
      if(digitalRead(13) == 0){
      goto atas;
      }
    }
  }
    }
  }
 
  
}
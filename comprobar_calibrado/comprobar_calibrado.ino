//Librerías: 
#include <Wire.h> //Comunicación I2C (giroscopio)
#include <math.h> //Cálculo del ángulo sobre la vertical. 
#include <Ticker.h> //Rutinas de interrupción periódicas. 
#include <MPU6050.h> //Conexión con el periférico. 
#include <I2Cdev.h> //Facilita conexión I2C

Ticker timer; //Rutina de interrupción periódica. 
const double T = 0.06; //Periodo de muestreo 60ms

//Acelerómetro: 
MPU6050 mpu;
const int MPU = 0X68; //Dirección I2C
// Variables para las lecturas del acelerómetro y giroscopio (raw)
int16_t ax, ay, az, gx, gy, gz;
double AcX, AcY, AcZ, tmp99, GyX, GyY, GyZ; //Variables.
//Parámetros de calibración --> Uso del fichero MPU6050_calibration 
const int AcXcal = -5482;
const int AcYcal = -1169;
const int AcZcal = 1676;
const int GyXcal = -54;
const int GyYcal = -36;
const int GyZcal = 19;

//Mediciones del ángulo (Varias opciones):
double ym1 = 0.0; //No filtrada y solo se usa el acelerómetro. 
double ym2 = 0.0; //Filtrada y solo se usa el acelerómetro. 
double ym3 = 0.0; //Filtro complementario.
double ym4 = 0.0; 
//Valor en grados --> Comprobar experimentalmente su utilidad. 
double ym1_grad = 0.0; 
double ym2_grad = 0.0; 
double ym3_grad = 0.0; 
double ym3_grad_ant = 0.0; 
double ym4_grad = 0.0; 

//Funciones: 
//Objetivo: cargar las variables del IMU
void read_MPU6050(){
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //Escalar los valores a RAW en un rango. 
  //Acelerometro en rango de - 2g a + 2g
  AcX = ax * (9.81 / 16384.0);  
  AcY = ay * (9.81 / 16384.0);  
  AcZ = az * (9.81 / 16384.0); 
  //Giroscopio en rango de 0 º/s a 250 º/s
  GyY = gy / 131.0; 
}

void limitador_pendiente() {
  //Evitar picos en la medición que produzcan gran sobreosciliación. 
  //Pensada para trabajar con grados.
  if (ym3_grad > (ym3_grad_ant + 1000*T)) { ym3_grad = ym3_grad_ant + 1000*T;}
  else if (ym3_grad < (ym3_grad_ant -1000*T)) {ym3_grad = ym3_grad_ant - 1000*T;}
}

void get_angle(int Ax, int Ay, int Az, int Gy) {
  //Parámetros filtro complementario: 
  //Ajustar valores mediante visaulización en SerialPlot.
  //Requisito --> A + B = 1
  double A = 0.95; 
  double B = 0.05; 
 
  //Parámetros --> Filtro paso bajo: 
  double a = 0.2; //Ajustar con la evolución de ym2 en SerialPlot
  double Gy_rad = GyY * (M_PI / 180.0); //Pasamos a radianes/segundo
  ym1 = atan(Ax / sqrt((Ay*Ay) + (Az*Az))); //U --> radianes
  ym2 = a*ym2 + (1-a)*ym1; //U --> radianes
  ym3 = A * (ym1 + (Gy_rad)*T) + B *ym1; 
  ym4 = a*ym4 + (1-a)*ym3; 
  
  //Conversión de radianes a grados: 
  ym1_grad = ym1 * (180.0 / M_PI);
  ym2_grad = ym2 * (180.0 / M_PI);
  ym3_grad = ym3 * (180.0 / M_PI);
  ym4_grad = ym4 * (180.0 / M_PI);

  //Evitar picos en la medición:
  limitador_pendiente();
  ym3_grad_ant = ym3_grad;
}

//Objetivo: Mostrar la evolución de las variables de interés. 
void ploter_debug() {
  Serial.print("Y1rad:");
  Serial.print(ym1);
  Serial.print(",Y1grados:");
  Serial.print(ym1_grad);
  Serial.print(",Y2rad:");
  Serial.print(ym2);
  Serial.print(",Y2grados:");
  Serial.print(ym2_grad);
  Serial.print(",Y3rad:");
  Serial.print(ym3);
  Serial.print(",Y3grados:");
  Serial.print(ym3_grad);
  Serial.print(",Y4grados:");
  Serial.println(ym4_grad);
}

void interrupcion_periodica() {
  read_MPU6050();
  get_angle(AcX, AcY, AcZ, GyY); 
  //Visualización de variables (ajuste y calibración):
  ploter_debug(); 
}

void setup(){
  //Comunicación I2C (sensor).
  Wire.begin();
  Serial.begin(9600); //Comunicación serie (baudios)
  mpu.initialize();
  //Offsets (calibración)
  mpu.setXAccelOffset(AcXcal);
  mpu.setYAccelOffset(AcYcal);
  mpu.setZAccelOffset(AcZcal);
  mpu.setXGyroOffset(GyXcal);
  mpu.setYGyroOffset(GyYcal);
  mpu.setZGyroOffset(GyZcal); 
  //Rutina de interrupción periódica.
  timer.attach(T, interrupcion_periodica);
}

void loop() {

}

//Librerías: 
#include <Wire.h> //Comunicación I2C (giroscopio)
#include <math.h> //Cálculo del ángulo sobre la vertical. 
#include <Ticker.h> //Rutinas de interrupción periódicas. 
#include <MPU6050.h> //Conexión con el periférico. 
#include <I2Cdev.h> //Facilita conexión I2C

Ticker timer; //Rutina de interrupción periódica. 
const double T = 0.033; //Muestreo a 30Hz.

//Motores: 
int motor11 = 1; //Motor1
int motor12 = 2; 
int motor21 = 3; //Motor2 
int motor22 = 4; 
int enable1 = 8; //Jumper 1
int enable2 = 9; //Jumper 2
const int freq = 30; //Frecuncia de PWM.
const int canal1 = 0; 
const int canal2 = 1; 
const int resol = 8; //8 bits de resolución (0 - 255)

//Acelerómetro: 
MPU6050 mpu;
const int MPU = 0X68; //Dirección I2C
// Variables para las lecturas del acelerómetro y giroscopio (raw)
int16_t ax, ay, az, gx, gy, gz;
double AcX, AcY, AcZ, tmp99, GyX, GyY, GyZ; //Variables.
//Parámetros de calibración (Offsets) 
const int AcXcal = -5482;
const int AcYcal = -1169;
const int AcZcal = 1676;
const int GyXcal = -54;
const int GyYcal = -36;
const int GyZcal = 19;

//Mediciones del ángulo (Varias opciones):
double ym1 = 0.0; //No filtrada y solo se usa el acelerómetro. 
double ym2 = 0.0; //Filtro paso bajo del acelerómetro. 
double ym3 = 0.0; //Filtro complementario. (Giroscopio + acelerómetro)
double ym4 = 0.0; 
//Valor en grados --> Mejor interpretación.  
double ym1_grad = 0.0; 
double ym2_grad = 0.0; 
double ym3_grad = 0.0; 
double ym3_grad_ant = 0.0; 
double ym4_grad = 0.0; 

//Parámetros para el control PD. 
const double r = 1.0; //Referencia --> U = grados
const double kp = 2.0; // Acción proporcional (P)
const double kd = 0.2; // Acción derivada (D)
double e = 0.0; //Error en la medición actual.
double eo = 0.0; //Error en la medición anterior.  
double ed = 0.0; //Error para la derivada. 
int u = 0; //Acción de control --> PWM.

//PWM MANUAL:
// Variable global de PWM (-255 a 255)
volatile int pwmValue = 15;

// Frecuencia en Hz
unsigned long pwmFrequency = 30;

//Funciones: 
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
  //Requisito --> A + B = 1
  double A = 0.95; //Peso del giroscopio.
  double B = 0.05; //Peso del acelerómetro.
 
  //Parámetros --> Filtro paso bajo: 
  double a = 0.9; //a >> mayor filtrado, pero señal más lenta. 

  double Gy_rad = GyY * (M_PI / 180.0); //Pasamos a radianes/segundo.
  //Cálculo del pitch (U --> radianes)
  //Solo acelerómetro --> Estable en el tiempo, pero mucho ruido.
  ym1 = atan(Ax / sqrt((Ay*Ay) + (Az*Az))); 
  ym2 = a*ym2 + (1-a)*ym1; 
  //Acelerómetro + giroscopio --> Estable en el tiempo y poco ruido.
  ym3 = A * (ym3 + (Gy_rad)*T) + B *ym1; 
  //Señal ym3 filtrada para la acción derivada.
  //Motivo: La acción derivada amplifica el ruido de medida.
  ym4 = a*ym4 + (1-a)*ym3; 
  
  //Conversión de radianes a grados: 
  ym1_grad = ym1 * (180.0 / M_PI);
  ym2_grad = ym2 * (180.0 / M_PI);
  ym3_grad = ym3 * (180.0 / M_PI);
  ym4_grad = ym4 * (180.0 / M_PI);
  
  limitador_pendiente();
  ym3_grad_ant = ym3_grad;
}

void PwmTask(void *pvParameters) {
  // Configuración de pines
  pinMode(motor11, OUTPUT);
  pinMode(motor12, OUTPUT);
  pinMode(motor21, OUTPUT);
  pinMode(motor22, OUTPUT);

  while (true) {
    int pwm = constrain(pwmValue, -255, 255);
    int absPwm = abs(pwm);

    unsigned long period = 1000000UL / pwmFrequency;
    unsigned long onTime = (absPwm * period) / 255;
    unsigned long offTime = period - onTime;

    // Dirección
    if (pwm > 0) {
      // Adelante
      digitalWrite(motor11, HIGH);
      digitalWrite(motor12, LOW);
      digitalWrite(motor21, HIGH);
      digitalWrite(motor22, LOW);
    } else if (pwm < 0) {
      // Atrás
      digitalWrite(motor11, LOW);
      digitalWrite(motor12, HIGH);
      digitalWrite(motor21, LOW);
      digitalWrite(motor22, HIGH);
    } else {
      // Detenido
      digitalWrite(motor11, LOW);
      digitalWrite(motor12, LOW);
      digitalWrite(motor21, LOW);
      digitalWrite(motor22, LOW);
    }

    // Señal PWM
    if (pwm != 0) {
      delayMicroseconds(onTime);
      digitalWrite(motor11, LOW);
      digitalWrite(motor12, LOW);
      digitalWrite(motor21, LOW);
      digitalWrite(motor22, LOW);
      delayMicroseconds(offTime);
    } else {
      delayMicroseconds(period);
    }
  }
}

//Objetivo: Todas las acciones de control producen movimiento en las ruedas.
void banda_muerta() {
  if (u < 0){
    u = u - 12;
  }
  else if (u > 0){
    u = u + 12; 
  }
}

//Objetivo: Visualización en el Serial Plotter 
void ploter_debug() {
  Serial.print("Y1grados:"); //Solo acelerómetro
  Serial.print(ym1_grad);
  Serial.print(",Y3grados:"); //Complementario
  Serial.print(ym3_grad);
  Serial.print(",Y4grados:"); //Complementario + Filtro
  Serial.print(ym4_grad);
  Serial.print(",U_PWM:");
  Serial.println(u);
}

void bucle_control(){
  //Obtención de la medición. 
  read_MPU6050();
  get_angle(AcX, AcY, AcZ, GyY); 
  
  //Obtención del error:
  e = r - ym3_grad; //U --> Grados
  ed = r - ym4_grad;
  //Acción derivada: 
  double D = kd / T*(ed - eo);
  u = kp*e + D; //Acción de control. 

  //Correción de no linealidades: 
  banda_muerta();
  
  //Saturación en la acción de control: 
  if (u > 255) { u = 255;}
  else if (u < -255) { u = -255;}
  
  //Actualizar el error anterior: 
  eo = ed; 

  //Atacar los motores:
  pwmValue = u;

  //Visualización de variables (ajuste y calibración):
  ploter_debug(); 
}

void setup() {
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
  timer.attach(T, bucle_control); //Core 0

  //Pines motores
  pinMode(motor11, OUTPUT);
  pinMode(motor12, OUTPUT);
  pinMode(motor21, OUTPUT);
  pinMode(motor22, OUTPUT);
  //PWM en el Core 1 --> Un solo core para el bucle de control.
  xTaskCreatePinnedToCore(
    PwmTask,
    "PWM_Control",
    2048,
    NULL,
    1,
    NULL,
    1  // Core 1
  );
}

void loop() {}

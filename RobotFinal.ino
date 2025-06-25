/*********************************************************************
 *  Carrito con encoders + HX711 + MPU6050
 *  Giro controlado por IMU (giroscopio Z) recibido por Bluetooth
 *********************************************************************/

#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include "HX711.h"

// ---------------- IMU -----------------
// Objeto para el acelerómetro/giroscopio MPU6050
MPU6050 imu;

// ---------------- Bluetooth -----------
// Comunicación serial por software con el módulo Bluetooth (HM-10)
SoftwareSerial mySerial(10, 11); // RX (pin 10), TX (pin 11)

// ---------------- Motores --------------
// Pines de control de los drivers de motor (IN1/IN2 + ENA para motor izquierdo,
//  IN3/IN4 + ENB para motor derecho)
const int IN1 = 6,  IN2 = 7,  ENA = 43; // Motor izquierdo
const int IN3 = 8,  IN4 = 9,  ENB = 45; // Motor derecho

// ---------------- Encoders -------------
// Pines de interrupción para los encoders de rueda
const int encoderIzqPin = 2;
const int encoderDerPin = 3;
// Contadores de pulsos (volatile porque cambian en ISR)
volatile long pulsosIzq = 0;
volatile long pulsosDer = 0;

// PWM: velocidad máxima para avanzar y girar
const int pwmIzqAvanzar = 255, pwmDerAvanzar = 223;
const int pwmIzqGiro    = 230, pwmDerGiro    = 230;

// ---------------- Físico ---------------
// Dimensiones físicas y parámetros de los encoders
const float diametroRueda = 6.5;             // cm
const float circunferencia = 3.1416 * diametroRueda;
const int   pulsosPorVueltaAvanceIzq = 74;
const int   pulsosPorVueltaAvanceDer = 118;
// Conversión de pulsos a cm recorridos
const float cmPorPulsoAvanceIzq = circunferencia / pulsosPorVueltaAvanceIzq;
const float cmPorPulsoAvanceDer = circunferencia / pulsosPorVueltaAvanceDer;
// Factor de corrección empírico para el avance
const float FACTOR_CORRECCION_AVANCE = 1.04;

// ---------------- HX711 ----------------
// Pines de la celda de carga (HX711)
const int HX711_DOUT = A1;
const int HX711_SCK  = A0;
HX711 scale;
// Umbral (en gramos) para detectar si hay planta
const float UMBRAL_PLANTA = 2.0;

// ---------------- Coordenadas ----------
// Variables para los puntos A (global), B (local) y C (local),
// y sus transformaciones a coordenadas globales (x1,y1) y (x2,y2)
float x0, y0;           // Punto A mundial
float xb_local, yb_local; // Punto B local
float xc_local, yc_local; // Punto C local
float x1, y1, x2, y2;     // Puntos B y C en global
float anguloInicial;      // Orientación inicial del robot

// ---------------- Estado global --------
// Estado de posición y orientación del robot
float x = 0, y = 0;       // Posición actual en global
float anguloActual = 0;   // Heading actual (grados)
// Para temporizar reportes de posición en avanzarConSeguimiento()
unsigned long ultimoReporte = 0;

void setup() {
  // Configurar pines de motor como salida
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Configurar pines de encoder con pull-up interno
  pinMode(encoderIzqPin, INPUT_PULLUP);
  pinMode(encoderDerPin, INPUT_PULLUP);
  // Asociar interrupciones para contar pulsos de encoder
  attachInterrupt(digitalPinToInterrupt(encoderIzqPin), contarPulsosIzq, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderDerPin), contarPulsosDer, RISING);

  // Inicializar Bluetooth y esperar un instante
  mySerial.begin(115200);
  delay(1500);

  // Inicializar I2C y MPU6050
  Wire.begin();
  imu.initialize();
  if (!imu.testConnection()) {
    mySerial.println("❌ ERROR: MPU6050 no encontrado");
    while (1);
  }
  mySerial.println("✅ MPU6050 conectado");

  // Inicializar la celda de carga
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(2280.f); // factor de calibración
  scale.tare();            // tarar a cero
}

void loop() {
  // 1) Leer del Bluetooth los 3 puntos + ángulo
  solicitarCoordenadas();
  // 2) Ejecutar la secuencia de movimiento
  ejecutarRecorrido();
}

// -------------------------------------------------
// Solicita secuencialmente A(global), B(local), C(local)
// y ángulo inicial desde el serial Bluetooth
// -------------------------------------------------
void solicitarCoordenadas() {
  mySerial.println("Introduce A (global), B (local), C (local) y ángulo inicial:");
  esperarEntrada(&x0, &y0, "Coordenadas A (global): ");
  esperarEntrada(&xb_local, &yb_local, "Coordenadas B (local): ");
  esperarEntrada(&xc_local, &yc_local, "Coordenadas C (local): ");
  esperarEntrada(&anguloInicial, "Ángulo inicial (grados): ");

  // Convertir B y C de local a global
  transformarALocalAGlobal(xb_local, yb_local, x1, y1);
  transformarALocalAGlobal(xc_local, yc_local, x2, y2);

  // Inicializar estado interno de pose
  x = x0;  
  y = y0;
  anguloActual = anguloInicial;
}

// -------------------------------------------------
// Controla todo el recorrido: A→B→(C)→retorno a A
// -------------------------------------------------
void ejecutarRecorrido() {
  // Ir de A a B
  moverAHasta(x1, y1);
  mySerial.println("En B: esperando 5 s para cargar planta...");
  delay(5000);

  // Leer peso en B
  float peso = scale.get_units(10);
  mySerial.print("Peso medido (g): "); mySerial.println(peso, 1);

  if (peso >= UMBRAL_PLANTA) {
    // Si hay planta, ir a C
    mySerial.println("¡Planta detectada! Continuando a C...");
    moverAHasta(x2, y2);

    // En C, esperar a que despejen la planta
    mySerial.println("En C: esperando retiro de planta...");
    do {
      float pesoC = scale.get_units(10);
      mySerial.print("Peso en C (g): "); mySerial.println(pesoC, 1);
      delay(1000);
    } while (scale.get_units(10) >= UMBRAL_PLANTA);

    mySerial.println("Planta retirada. Iniciando retorno...");
    // Volver a B y luego a A
    moverAHasta(x1, y1);
    moverAHasta(x0, y0);
  } else {
    // Si no hay planta en B directamente regresa a A
    mySerial.println("No hay planta. Regresando a A.");
    moverAHasta(x0, y0);
  }

  // Corrección final del ángulo si es necesario
  if (abs(anguloActual - anguloInicial) > 1.0) {
    float dif = anguloInicial - anguloActual;
    if (dif > 180)  dif -= 360;
    if (dif < -180) dif += 360;
    mySerial.print("Corrigiendo ángulo final: "); mySerial.println(dif);
    girarConIMU(abs(dif), dif < 0);
    anguloActual = anguloInicial;
  }

  mySerial.println("✅ Ruta completa. Reiniciando...");
  delay(1500);
}

// -------------------------------------------------
// Gira hacia (xd, yd) y avanza la distancia necesaria
// -------------------------------------------------
void moverAHasta(float xd, float yd) {
  // 1) Calcular heading objetivo
  float angObjetivo = atan2(yd - y, xd - x) * 180 / 3.1416;
  if (angObjetivo < 0) angObjetivo += 360;

  // 2) Ajustar giro mínimo
  float dif = angObjetivo - anguloActual;
  if (dif > 180)  dif -= 360;
  if (dif < -180) dif += 360;

  // 3) Girar usando el giroscopio
  mySerial.print("Girando "); mySerial.print(dif); mySerial.println("° …");
  girarConIMU(abs(dif), dif < 0);
  anguloActual = angObjetivo;

  // 4) Calcular distancia lineal y avanzar
  float distancia = sqrt(pow(xd - x, 2) + pow(yd - y, 2))
                    * FACTOR_CORRECCION_AVANCE;
  avanzarConSeguimiento(distancia);
}

// -------------------------------------------------
// Avanza controlando con encoders hasta distCM (cm)
// -------------------------------------------------
void avanzarConSeguimiento(float distCM) {
  pulsosIzq = 0;
  pulsosDer = 0;
  setMotoresAvanzar();
  float avancePrev = 0;

  while (true) {
    // Leer avance real de cada rueda
    float avanceIzq = abs(pulsosIzq) * cmPorPulsoAvanceIzq;
    float avanceDer = abs(pulsosDer) * cmPorPulsoAvanceDer;
    float avance = (avanceIzq + avanceDer) / 2.0;

    // Si llegó, frena y sale
    if (avance >= distCM) {
      parar();
      break;
    }

    // Cada 250 ms, actualizar pose y publicarlo
    if (millis() - ultimoReporte >= 250) {
      ultimoReporte = millis();
      float delta = avance - avancePrev;
      avancePrev = avance;
      x += delta * cos(anguloActual * 3.1416 / 180.0);
      y += delta * sin(anguloActual * 3.1416 / 180.0);
      mySerial.print("Posición: (");
      mySerial.print(x, 2); mySerial.print(", ");
      mySerial.print(y, 2); mySerial.print(")  θ=");
      mySerial.print(anguloActual, 1); mySerial.println("°");
    }
  }
}

// -------------------------------------------------
// Gira usando los datos del giroscopio Z del MPU6050
// -------------------------------------------------
void girarConIMU(float angulo, bool horario) {
  setMotoresGirar(horario);
  unsigned long tPrev = millis();
  float angAcum = 0.0;
  const float margen = 2.0; // tolerancia en grados

  while (angAcum < angulo - margen) {
    int16_t gx, gy, gz;
    imu.getRotation(&gx, &gy, &gz);
    float velZ = gz / 131.0;        // convertir a °/s
    if (horario) velZ *= -1;        // invertir si gira en sentido horario
    unsigned long tNow = millis();
    float dt = (tNow - tPrev) / 1000.0;
    tPrev = tNow;
    angAcum += abs(velZ) * dt;
  }
  parar();
  delay(200);
}

// -------------------------------------------------
// Helpers de control de motor
// -------------------------------------------------
void setMotoresAvanzar() {
  analogWrite(ENA, pwmIzqAvanzar);
  analogWrite(ENB, pwmDerAvanzar);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void setMotoresGirar(bool horario) {
  analogWrite(ENA, pwmIzqGiro);
  analogWrite(ENB, pwmDerGiro);
  if (horario) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  }
}

void parar() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// -------------------------------------------------
// Transforma coordenadas locales (campo) a globales
// usando una rototranslación fija
// -------------------------------------------------
void transformarALocalAGlobal(float x_local, float y_local,
                              float& x_g, float& y_g) {
  const float ang = 30.0 * 3.1416 / 180.0; // 45° en rad
  const float ox  = 150.0, oy = 60.0;       // offset global
  x_g = ox + (x_local * cos(ang) - y_local * sin(ang));
  y_g = oy + (x_local * sin(ang) + y_local * cos(ang));
}

// -------------------------------------------------
// ISRs de encoder: cuentan pulsos
// -------------------------------------------------
void contarPulsosIzq() { pulsosIzq++; }
void contarPulsosDer() { pulsosDer++; }

// -------------------------------------------------
// Lectura segura de dos floats por serial
// -------------------------------------------------
void esperarEntrada(float* v1, float* v2, const char* msg) {
  mySerial.println(msg);
  while (mySerial.available() == 0);
  *v1 = mySerial.parseFloat();
  while (mySerial.available() == 0);
  *v2 = mySerial.parseFloat();
  while (mySerial.available()) mySerial.read();
}

// -------------------------------------------------
// Lectura segura de un solo float
// -------------------------------------------------
void esperarEntrada(float* v, const char* msg) {
  mySerial.println(msg);
  while (mySerial.available() == 0);
  *v = mySerial.parseFloat();
  while (mySerial.available()) mySerial.read();
}

#include <Servo.h>

Servo servo;    // Servo en pin 5
Servo servo2;   // Servo en pin 6

const int pinSensor = 8;

const float frecuenciaMinima = 3000.0;
const float frecuenciaMaxima = 20000.0;        // descarta lecturas absurdas (ruido)
const unsigned long tiempoArranque = 1000;     // ignora detección al inicio
const unsigned long intervaloLectura = 200;    // ms de ventana para contar pulsos
const unsigned long periodoMinimo = 40;        // µs, filtro de ruido en el ISR

const int ventanasNecesarias = 4;              // ventanas seguidas en rango para disparar (~1 s)
const unsigned long tiempoEnfriamiento = 2500; // ms sin detección tras mover servos
const unsigned long tiempoRearme = 1000;       // ms sin frecuencia para volver a armar

const int msPorGrado = 3;
const int margenMs = 30;

volatile unsigned long contadorPulsos = 0;
volatile unsigned long ultimoPulso = 0;

unsigned long ultimaLectura = 0;
unsigned long finEnfriamiento = 0;
unsigned long inicioSinFrecuencia = 0;
int ventanasEnRango = 0;
bool disparado = false;     // true = ya se activó, espera a que se pierda la frecuencia

float frecuencia = 0;
int anguloActual = 90;

// Solo al detectar frecuencia: primero el servo del pin 6 y luego el del pin 5
void moverServos() {
  int angulo;
  do {
    angulo = random(4, 13) * 30;   // múltiplo de 10: 40 a 120
  } while (angulo == anguloActual);

  int recorrido = abs(angulo - anguloActual);
  anguloActual = angulo;

  Serial.print("Servo pin 6 -> ");
  Serial.println(angulo);

  servo2.write(angulo);                          // 1) servo del pin 6
  delay(recorrido * msPorGrado + margenMs);
  servo.write(180);                              // 2) servo del pin 5, se queda en 180

  // borra los pulsos contados durante el movimiento (ruido)
  noInterrupts();
  contadorPulsos = 0;
  interrupts();

  finEnfriamiento = millis() + tiempoEnfriamiento;
  ventanasEnRango = 0;
}

void setup() {
  pinMode(pinSensor, INPUT_PULLUP);

  servo.attach(5);
  servo.write(150);

  servo2.attach(6);
  servo2.write(90);

  randomSeed(analogRead(A0));

  Serial.begin(115200);
  Serial.println("ARRANQUE");   // si aparece al forzar un servo, el Arduino se reinició

  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
}

void loop() {
  if (millis() - ultimaLectura >= intervaloLectura) {
    unsigned long dtMs = millis() - ultimaLectura;
    ultimaLectura = millis();

    unsigned long pulsos;
    noInterrupts();
    pulsos = contadorPulsos;
    contadorPulsos = 0;
    interrupts();

    if (millis() > tiempoArranque && millis() > finEnfriamiento) {
      frecuencia = pulsos * 1000.0 / dtMs;
    } else {
      frecuencia = 0;
    }

    Serial.print("Frecuencia: ");
    Serial.print(frecuencia);
    Serial.print(" Hz | Ventanas: ");
    Serial.println(ventanasEnRango);

    bool enRango = (frecuencia >= frecuenciaMinima && frecuencia <= frecuenciaMaxima);

    if (enRango) {
      inicioSinFrecuencia = 0;
      if (!disparado) {
        ventanasEnRango++;
        if (ventanasEnRango >= ventanasNecesarias) {
          disparado = true;
          moverServos();          // solo al detectar frecuencia sostenida
          Serial.println("2");
        }
      }
    } else {
      ventanasEnRango = 0;
      // re-arma solo después de 1 s sin frecuencia
      if (disparado) {
        if (inicioSinFrecuencia == 0) inicioSinFrecuencia = millis();
        if (millis() - inicioSinFrecuencia >= tiempoRearme) {
          disparado = false;
          inicioSinFrecuencia = 0;
        }
      }
    }
  }

  // Comandos por serial
  if (Serial.available()) {
    char dato = Serial.read();

    if (dato == '1') {
      servo.write(150);         // servo del pin 5 a 145
      disparado = false;        // re-arma
    }
    else if (dato == '2') {
      servo.write(180);         // solo el servo del pin 5, el 6 no se mueve
    }
  }

  delay(10);
}

ISR(PCINT0_vect) {
  static bool estadoAnterior = false;

  bool estadoActual = PINB & B00000001;

  if (estadoActual && !estadoAnterior) {
    unsigned long tiempoActual = micros();

    if (tiempoActual - ultimoPulso >= periodoMinimo) {
      contadorPulsos++;
      ultimoPulso = tiempoActual;
    }
  }

  estadoAnterior = estadoActual;
}
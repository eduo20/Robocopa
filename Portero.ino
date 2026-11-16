#include <Servo.h>

Servo portero;

const int pinServo  = 9;
const int pinSensor = 8;   // Señal del circuito con micrófono electret

// Posiciones del servo (calibradas)
const int pulsoCentro = 1500;  // 90°
const int pulsoMin    = 600;   // 0°
const int pulsoMax    = 2400;  // 180°

// Detección de frecuencia
const float frecuenciaMinima = 3000.0;
const float frecuenciaMaxima = 20000.0;
const float toleranciaFrecuencia = 0.5;         // variación máxima entre ventanas (50%)
const unsigned long intervaloLectura = 50;      // ms por ventana
const unsigned long periodoMinimo = 40;         // µs, filtro de ruido en el ISR

const unsigned long tiempoSostenido = 100;      // ms que debe sostenerse el sonido
const unsigned long tiempoSilencioArmar = 1000; // ms de silencio necesarios para armar
const unsigned long tiempoEnfriamiento = 1000;  // ms sin escuchar tras mover/soltar el servo
const unsigned long tiempoPosicionar = 700;     // ms que el servo recibe pulsos para llegar a 90°

const int ventanasNecesarias = tiempoSostenido / intervaloLectura;
const int ventanasSilencioNecesarias = tiempoSilencioArmar / intervaloLectura;

volatile unsigned long contadorPulsos = 0;
volatile unsigned long ultimoPulso = 0;

unsigned long ultimaLectura = 0;
unsigned long finEnfriamiento = 0;
float frecuenciaAnterior = 0;
int ventanasEstables = 0;
int ventanasSilencio = 0;
bool armado = false;
bool disparado = false;

// Lleva el servo a 90° y luego lo suelta (sin pulsos = sin ruido)
void centrarYSoltar() {
  portero.writeMicroseconds(pulsoCentro);
  portero.attach(pinServo);
  delay(tiempoPosicionar);
  portero.detach();
  finEnfriamiento = millis() + tiempoEnfriamiento;
}

void lanzarPortero() {
  int destino;
  if (random(2) == 0) {
    destino = pulsoMin;    // 0°
    Serial.println("Portero -> 0");
  } else {
    destino = pulsoMax;    // 180°
    Serial.println("Portero -> 180");
  }
  portero.writeMicroseconds(destino);
  portero.attach(pinServo);   // se queda conectado, sosteniendo la posición
}

void setup() {
  pinMode(pinSensor, INPUT_PULLUP);
  randomSeed(analogRead(A0));  // A0 sin conectar

  Serial.begin(115200);
  Serial.println("LISTO");

  centrarYSoltar();            // inicia en 90° y deja de mandar pulsos

  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);

  noInterrupts();
  contadorPulsos = 0;
  interrupts();
}

void loop() {
  if (!disparado && millis() - ultimaLectura >= intervaloLectura) {
    unsigned long dtMs = millis() - ultimaLectura;
    ultimaLectura = millis();

    unsigned long pulsos;
    noInterrupts();
    pulsos = contadorPulsos;
    contadorPulsos = 0;
    interrupts();

    float frecuencia = 0;
    if (millis() > finEnfriamiento) {
      frecuencia = pulsos * 1000.0 / dtMs;
    }

    bool enRango = (frecuencia >= frecuenciaMinima && frecuencia <= frecuenciaMaxima);

    Serial.print("Frecuencia: ");
    Serial.print(frecuencia, 0);
    Serial.print(" Hz | ");
    Serial.print(armado ? "ARMADO" : "esperando silencio");
    Serial.print(" | estables: ");
    Serial.println(ventanasEstables);

    if (!armado) {
      if (enRango) {
        ventanasSilencio = 0;
      } else {
        ventanasSilencio++;
        if (ventanasSilencio >= ventanasSilencioNecesarias) {
          armado = true;
          frecuenciaAnterior = 0;
          ventanasEstables = 0;
          Serial.println(">>> ARMADO");
        }
      }
    } else {
      bool estable = enRango &&
                     frecuenciaAnterior > 0 &&
                     abs(frecuencia - frecuenciaAnterior) <= frecuenciaAnterior * toleranciaFrecuencia;

      frecuenciaAnterior = enRango ? frecuencia : 0;

      if (estable) {
        ventanasEstables++;
        if (ventanasEstables >= ventanasNecesarias) {
          disparado = true;
          lanzarPortero();
        }
      } else {
        ventanasEstables = 0;
      }
    }
  }

  // Comando por serial: '1' devuelve el portero a 90° y rearma
  if (Serial.available()) {
    char dato = Serial.read();
    if (dato == '1') {
      centrarYSoltar();
      noInterrupts();
      contadorPulsos = 0;
      interrupts();
      ventanasEstables = 0;
      ventanasSilencio = 0;
      frecuenciaAnterior = 0;
      armado = false;
      disparado = false;
      Serial.println("Portero -> 90 (esperando silencio para armar)");
    }
  }
}

ISR(PCINT0_vect) {
  static bool estadoAnterior = false;

  bool estadoActual = PINB & B00000001;   // pin 8

  if (estadoActual && !estadoAnterior) {
    unsigned long tiempoActual = micros();
    if (tiempoActual - ultimoPulso >= periodoMinimo) {
      contadorPulsos++;
      ultimoPulso = tiempoActual;
    }
  }

  estadoAnterior = estadoActual;
}
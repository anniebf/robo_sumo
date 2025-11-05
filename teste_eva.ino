#include <AFMotor.h>
#include <Ultrasonic.h>

// Pins
#define TRIGGER A0
#define ECHO    A1
#define SENSOR_LINHA   A5  // único sensor infravermelho

// Velocidades
#define VEL_BUSCA   180
#define VEL_ATAQUE  255
#define VEL_MANOBRA 180
#define VEL_GIRO    200

// Tempos
#define TEMPO_RECUO   600
#define TEMPO_AVANCO  600
#define TEMPO_GIRO    300
#define DEBOUNCE_TEMPO 50
#define TEMPO_PAUSA_ULTRA 500

// Objetos
AF_DCMotor motor1(1); // esquerda
AF_DCMotor motor2(2); // direita
Ultrasonic ultrasonic(TRIGGER, ECHO);

// Variáveis
unsigned long ultimoTempo = 0;
unsigned long pausaUltrassomAte = 0;
unsigned long ultimoDebounceLinha = 0;
int ultimoSensorLinha = LOW;

enum Estado {
  PROCURANDO,
  ATACANDO,
  RECUANDO,
  AVANCANDO,
  PARADO
};
Estado estadoAtual = PROCURANDO;

void setup() {
  Serial.begin(9600);
  Serial.println("=== Robo Sumo - 2 motores / 1 sensor ===");

  motor1.setSpeed(VEL_BUSCA);
  motor2.setSpeed(VEL_BUSCA);

  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(SENSOR_LINHA, INPUT_PULLUP);

  Serial.println("Inicializado!");
}

void loop() {
  int sensorLinha = lerSensorLinhaDigital(SENSOR_LINHA, ultimoSensorLinha, ultimoDebounceLinha);
  float dist;

  // Pausa ultrassom ao detectar borda
  if (sensorLinha == HIGH) {
    pausaUltrassomAte = millis() + TEMPO_PAUSA_ULTRA;
    dist = 400;
  } else if (millis() < pausaUltrassomAte) {
    dist = 400;
  } else {
    dist = ultrasonic.read();
    if (dist <= 0 || dist > 400) {
      dist = readUltrasonicCM(TRIGGER, ECHO);
    }
    if (dist <= 0 || dist > 400) dist = 400;
  }

  Serial.print("Dist: "); Serial.print(dist);
  Serial.print(" cm | Linha: "); Serial.print(sensorLinha);
  Serial.print(" | Estado: "); Serial.println(estadoAtual);

  // Prioridade: sensor de linha
  if (sensorLinha == HIGH) {
    Serial.println("!!! BORDA DETECTADA !!!");
    mudarEstado(RECUANDO);
    executarEstado(dist, sensorLinha);
    return;
  }

  // Detecção doponente
  if (dist <= 40 && dist > 0) {
    mudarEstado(ATACANDO);
  } else if (estadoAtual != RECUANDO && estadoAtual != AVANCANDO) {
    mudarEstado(PROCURANDO);
  }

  executarEstado(dist, sensorLinha);
}

// ------------------- FUNÇÕES -------------------
int lerSensorLinhaDigital(int pino, int &ultimoEstado, unsigned long &ultimoDebounce) {
  int leituraAtual = digitalRead(pino);
  if (leituraAtual != ultimoEstado) {
    if (millis() - ultimoDebounce > DEBOUNCE_TEMPO) {
      ultimoEstado = leituraAtual;
      ultimoDebounce = millis();
      Serial.print("Sensor "); Serial.print(pino);
      Serial.print(" mudou para "); Serial.println(ultimoEstado);
    }
  }
  return ultimoEstado;
}

void mudarEstado(Estado novo) {
  if (estadoAtual != novo) {
    estadoAtual = novo;
    ultimoTempo = millis();
    Serial.print("Mudando estado para: "); Serial.println(estadoAtual);
  }
}

void executarEstado(float dist, int sensorLinha) {
  switch (estadoAtual) {
    case PROCURANDO: procura(); break;
    case ATACANDO: ataque(dist, sensorLinha); break;
    case RECUANDO: recuar(sensorLinha); break;
    case AVANCANDO: avancar(sensorLinha); break;
    case PARADO: parada(); break;
  }
}

void frente(int vel = VEL_BUSCA) {
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
}

void tras(int vel = VEL_MANOBRA) {
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
}

void esquerda(int vel = VEL_GIRO) {
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
}

void direita(int vel = VEL_GIRO) {
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
}

void parada() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}

void procura() {
  // Alterna busca esquerda/direita
  if ((millis() / 2000) % 2 == 0) {
    esquerda(VEL_GIRO);
  } else {
    direita(VEL_GIRO);
  }
}

void ataque(float dist, int sensorLinha) {
  if (dist <= 40 && sensorLinha == LOW) {
    Serial.println(">>> ATAQUE!");
    frente(VEL_ATAQUE);
  } else {
    mudarEstado(PROCURANDO);
  }
}

void recuar(int sensorLinha) {
  tras(VEL_MANOBRA);
  if (millis() - ultimoTempo > TEMPO_RECUO || sensorLinha == LOW) {
    direita(VEL_GIRO);
    delay(TEMPO_GIRO);
    mudarEstado(PROCURANDO);
  }
}

void avancar(int sensorLinha) {
  frente(VEL_MANOBRA);
  if (millis() - ultimoTempo > TEMPO_AVANCO || sensorLinha == LOW) {
    mudarEstado(PROCURANDO);
  }
}

// Manual ultrassonic (fallback)
long readUltrasonicCM(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 400;
  long cm = duration / 58;
  return cm;
}

#include <AFMotor.h>
#include <Ultrasonic.h>

#define TRIGGER A0
#define ECHO A1
#define SENSOR_FRENTE A4
#define SENSOR_TRAS A5

AF_DCMotor motor1(4);
AF_DCMotor motor2(1);
AF_DCMotor motor3(3);
AF_DCMotor motor4(2);

Ultrasonic ultrasonic(TRIGGER, ECHO);

const int DIST_ATAQUE = 30; // cm
const int PRETO = 1;
const int BRANCO = 0;
const int THRESHOLD = 512; // Ajuste conforme necessidade

void setup() {
  Serial.begin(9600);
  modoProcura();
  pinMode(SENSOR_FRENTE, INPUT);
  pinMode(SENSOR_TRAS, INPUT);
  delay(5000); // Espera 5 segundos antes de iniciar movimentação
}

bool detectarBorda(int pino) {
  int valor = analogRead(pino);
  return (valor < THRESHOLD) ? BRANCO : PRETO;
}

float lerDistancia() {
  // Para a maioria das bibliotecas Ultrasonic, read() retorna a distância em centímetros
  float distancia = ultrasonic.read(); // ou ultrasonic.readCM();
  if (distancia <= 0 || distancia > 400)
    return -1;
  return distancia;
}

void detectaAlvo() {
  float alvo = lerDistancia();
  if (alvo == -1) return;
  int sensorFrente = detectarBorda(SENSOR_FRENTE);
  int sensorTras = detectarBorda(SENSOR_TRAS);
  bool foraDoRange = (alvo > DIST_ATAQUE);
  bool noRange = (alvo <= DIST_ATAQUE && alvo > 5);
  bool naArena = (sensorFrente == BRANCO && sensorTras == BRANCO);
  if (foraDoRange && naArena) {
    modoProcura();
    procura();
  }
  if (noRange && naArena) {
    modoAtaque();
    while (detectarBorda(SENSOR_FRENTE) == BRANCO) {
      frente();
      delay(50);
      float distanciaAtual = lerDistancia();
      if (distanciaAtual == -1 || distanciaAtual > DIST_ATAQUE) break;
    }
    parada();
    delay(200);
    tras();
    delay(600);
    parada();
  }
}

void detectaBordaArena() {
  if (detectarBorda(SENSOR_FRENTE) == PRETO) {
    modoProcura();
    parada();
    delay(100);
    tras();
    delay(600);
    parada();
    esquerda();
    delay(400);
    parada();
  }
  if (detectarBorda(SENSOR_TRAS) == PRETO) {
    modoProcura();
    parada();
    delay(100);
    frente();
    delay(600);
    parada();
    esquerda();
    delay(400);
    parada();
  }
}

void loop() {
  detectaBordaArena();
  detectaAlvo();
  delay(50);
}

void frente() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void tras() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void parada() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void esquerda() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void direita() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void procura() {
  frente();
  delay(500);
  esquerda();
  delay(250);
  parada();
  delay(50);
  static int contador = 0;
  if (contador++ % 3 == 0) {
    direita();
    delay(250);
  }
}

void modoAtaque() {
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
}

void modoProcura() {
  motor1.setSpeed(180);
  motor2.setSpeed(180);
  motor3.setSpeed(180);
  motor4.setSpeed(180);
}

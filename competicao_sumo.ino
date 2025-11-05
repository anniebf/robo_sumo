#include <AFMotor.h>
#include <Ultrasonic.h>

// ------------------ PINOS ------------------
#define TRIGGER A0  
#define ECHO    A1  
#define SENSOR_FRENTE A4
#define SENSOR_TRAS   A5

// ------------------ VELOCIDADES ------------------
#define VEL_BUSCA   180
#define VEL_ATAQUE  255
#define VEL_MANOBRA 180
#define VEL_GIRO    200  
#define VEL_DESVIO  220  // velocidade para desvio lateral

// ------------------ TEMPOS ------------------
#define TEMPO_GIRO    300
#define TEMPO_RECUO   600
#define TEMPO_AVANCO  600
#define TEMPO_DESVIO  400  // tempo de desvio lateral
#define TEMPO_POSICAO_INICIAL 500  // tempo para posicionar rodas em 45°
#define DEBOUNCE_TEMPO 50  
#define TEMPO_PAUSA_ULTRA 500

// ------------------ OBJETOS ------------------
AF_DCMotor motor4(4); // traseira esquerda
AF_DCMotor motor2(2); // traseira direita
AF_DCMotor motor3(3); // dianteira direita
AF_DCMotor motor1(1); // dianteira esquerda
Ultrasonic ultrasonic(TRIGGER, ECHO);

// ------------------ VARIÁVEIS ------------------
unsigned long ultimoTempo = 0;
unsigned long pausaUltrassomAte = 0;
unsigned long ultimoDebounceFrente = 0;
unsigned long ultimoDebounceTras = 0;

int ultimoSensorFrente = LOW;
int ultimoSensorTras = LOW;

bool posicaoInicialFeita = false;  // controla se já posicionou as rodas
bool sentidoDesvio = true;  // alterna direção do desvio (true=direita, false=esquerda)
int contadorAtaquesFrontais = 0;  // conta quantas vezes atacou de frente

enum Estado { 
  POSICIONANDO,   // novo estado para posicionar rodas
  PROCURANDO, 
  ATACANDO, 
  ATACANDO_LATERAL,  // novo estado para ataque de lado
  DESVIANDO,  // novo estado para desvio
  RECUANDO, 
  AVANCANDO, 
  PARADO 
};
Estado estadoAtual = POSICIONANDO;

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(9600);
  Serial.println("=== Robô Sumô v7.0 - Modo Ataque Angular ===");

  motor1.setSpeed(VEL_BUSCA);
  motor2.setSpeed(VEL_BUSCA);
  motor3.setSpeed(VEL_BUSCA);
  motor4.setSpeed(VEL_BUSCA);

  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(SENSOR_FRENTE, INPUT_PULLUP);  // corrigido para INPUT_PULLUP
  pinMode(SENSOR_TRAS, INPUT_PULLUP);

  Serial.println("Inicializado!");
  Serial.println("Posicionando rodas dianteiras em 45 graus...");
  delay(1000);
}

// ------------------ LOOP PRINCIPAL ------------------
void loop() {
  int sFrente = lerSensorLinhaDigital(SENSOR_FRENTE, ultimoSensorFrente, ultimoDebounceFrente);
  int sTras   = lerSensorLinhaDigital(SENSOR_TRAS, ultimoSensorTras, ultimoDebounceTras);

  float dist;

  // Pausa ultrassom ao detectar borda
  if (sFrente == HIGH || sTras == HIGH) {
    pausaUltrassomAte = millis() + TEMPO_PAUSA_ULTRA;
    dist = 400;
  } 
  else if (millis() < pausaUltrassomAte) {
    dist = 400;
  } 
  else {
    dist = ultrasonic.read();
    if (dist <= 0 || dist > 400) {
      dist = readUltrasonicCM(TRIGGER, ECHO);
    }
    if (dist <= 0 || dist > 400) dist = 400;
  }

  Serial.print("Dist: "); Serial.print(dist);
  Serial.print(" cm | Frente: "); Serial.print(sFrente);
  Serial.print(" | Tras: "); Serial.print(sTras);
  Serial.print(" | Estado: "); Serial.println(estadoAtual);

  // Prioridade: sensores de linha
  if (sFrente == HIGH) {
    Serial.println("!!! BORDA FRONTAL DETECTADA !!!");
    mudarEstado(RECUANDO);
    executarEstado(dist, sFrente, sTras);
    return;
  }

  if (sTras == HIGH) {
    Serial.println("!!! BORDA TRASEIRA DETECTADA !!!");
    mudarEstado(AVANCANDO);
    executarEstado(dist, sFrente, sTras);
    return;
  }

  // Detecção do oponente
  if (dist <= 40 && dist > 0) {
    // Decide se ataca de frente ou desvia
    if (estadoAtual != DESVIANDO && estadoAtual != ATACANDO_LATERAL) {
      // A cada 2 ataques frontais, faz um desvio
      if (contadorAtaquesFrontais >= 2) {
        mudarEstado(DESVIANDO);
        contadorAtaquesFrontais = 0;
        sentidoDesvio = !sentidoDesvio;  // alterna lado do desvio
      } else {
        mudarEstado(ATACANDO);
      }
    }
  } 
  else if (estadoAtual != RECUANDO && estadoAtual != AVANCANDO && 
           estadoAtual != DESVIANDO && estadoAtual != POSICIONANDO &&
           estadoAtual != ATACANDO_LATERAL) {
    mudarEstado(PROCURANDO);
  }

  executarEstado(dist, sFrente, sTras);
}

// ------------------ LEITURA SENSOR LINHA DIGITAL COM DEBOUNCE ------------------
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

// ------------------ CONTROLE DE ESTADOS ------------------
void mudarEstado(Estado novo) {
  if (estadoAtual != novo) {
    estadoAtual = novo;
    ultimoTempo = millis();
    Serial.print("Mudando estado para: "); Serial.println(estadoAtual);
  }
}

void executarEstado(float dist, int sf, int st) {
  switch (estadoAtual) {
    case POSICIONANDO: posicionarRodas(); break;
    case PROCURANDO: procura(); break;
    case ATACANDO: ataque(dist, sf); break;
    case ATACANDO_LATERAL: ataqueLateral(dist, sf); break;
    case DESVIANDO: desviar(dist, sf); break;
    case RECUANDO: recuar(st); break;
    case AVANCANDO: avancar(sf); break;
    case PARADO: parada(); break;
  }
}

// ------------------ MOVIMENTOS BÁSICOS ------------------
void frente(int vel = VEL_BUSCA) {
  setVelocidade(vel);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void tras(int vel = VEL_MANOBRA) {
  setVelocidade(vel);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void esquerda(int vel = VEL_GIRO) {
  setVelocidade(vel);
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

void direita(int vel = VEL_GIRO) {
  setVelocidade(vel);
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

// ------------------ NOVOS MOVIMENTOS ANGULARES ------------------

// Move em diagonal para direita (ataque angular)
void diagonalDireita(int vel = VEL_ATAQUE) {
  motor1.setSpeed(vel);
  motor2.setSpeed(vel * 0.6);  // motor traseiro direito mais lento
  motor3.setSpeed(vel * 0.6);  // motor dianteiro direito mais lento
  motor4.setSpeed(vel);
  
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

// Move em diagonal para esquerda (ataque angular)
void diagonalEsquerda(int vel = VEL_ATAQUE) {
  motor1.setSpeed(vel * 0.6);  // motor esquerdo mais lento
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel * 0.6);  // motor traseiro esquerdo mais lento
  
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

// Desvio rápido para lateral
void desvioRapidoDireita(int vel = VEL_DESVIO) {
  motor1.setSpeed(vel);
  motor2.setSpeed(vel * 0.3);
  motor3.setSpeed(vel * 0.3);
  motor4.setSpeed(vel);
  
  motor1.run(BACKWARD);
  motor2.run(FORWARD);  // lado direito vai para trás
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

void desvioRapidoEsquerda(int vel = VEL_DESVIO) {
  motor1.setSpeed(vel * 0.3);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel * 0.3);
  
  motor1.run(FORWARD);  // lado esquerdo vai para trás
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

void parada() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void setVelocidade(int vel) {
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel);
}

// ------------------ AÇÕES ESTRATÉGICAS ------------------

// Posiciona rodas dianteiras em 45 graus ao iniciar
void posicionarRodas() {
  if (!posicaoInicialFeita) {
    Serial.println(">>> Posicionando rodas dianteiras em 45 graus");
    
    // Gira levemente para posicionar rodas M1 e M3 em ângulo
    motor1.setSpeed(150);
    motor3.setSpeed(150);
    motor2.setSpeed(0);
    motor4.setSpeed(0);
    
    motor1.run(BACKWARD);
    motor3.run(FORWARD);
    
    delay(TEMPO_POSICAO_INICIAL);
    
    parada();
    posicaoInicialFeita = true;
    
    Serial.println(">>> Posicionamento concluído! Iniciando busca...");
    delay(200);
    mudarEstado(PROCURANDO);
  }
}

// Busca com movimento angular
void procura() {
  // Gira com movimento angular para cobrir mais área
  if ((millis() / 1500) % 3 == 0) {
    diagonalDireita(VEL_BUSCA);
  } else if ((millis() / 1500) % 3 == 1) {
    diagonalEsquerda(VEL_BUSCA);
  } else {
    direita(VEL_GIRO);
  }
}

// Ataque frontal com incremento de contador
void ataque(float dist, int sensorFrente) {
  if (dist <= 40 && sensorFrente == LOW) {
    Serial.println(">>> ATAQUE FRONTAL!");
    frente(VEL_ATAQUE);
    contadorAtaquesFrontais++;
  } else {
    mudarEstado(PROCURANDO);
  }
}

// Desvio lateral seguido de ataque de lado
void desviar(float dist, int sensorFrente) {
  if (millis() - ultimoTempo < TEMPO_DESVIO) {
    // Fase 1: Desvio rápido
    Serial.println(">>> DESVIANDO!");
    if (sentidoDesvio) {
      desvioRapidoDireita(VEL_DESVIO);
    } else {
      desvioRapidoEsquerda(VEL_DESVIO);
    }
  } else {
    // Fase 2: Ataque lateral
    mudarEstado(ATACANDO_LATERAL);
  }
}

// Ataque de lado (mais efetivo para empurrar)
void ataqueLateral(float dist, int sensorFrente) {
  if (dist <= 50 && sensorFrente == LOW) {
    Serial.println(">>> ATAQUE LATERAL!");
    if (sentidoDesvio) {
      diagonalDireita(VEL_ATAQUE);
    } else {
      diagonalEsquerda(VEL_ATAQUE);
    }
    
    // Após 1 segundo de ataque lateral, volta a procurar
    if (millis() - ultimoTempo > 1000) {
      mudarEstado(PROCURANDO);
    }
  } else {
    mudarEstado(PROCURANDO);
  }
}

void recuar(int sensorTras) {
  tras(VEL_MANOBRA);
  if (millis() - ultimoTempo > TEMPO_RECUO || sensorTras == LOW) {
    // Após recuar, gira para não cair novamente
    direita(VEL_GIRO);
    delay(300);
    mudarEstado(PROCURANDO);
  }
}

void avancar(int sensorFrente) {
  frente(VEL_MANOBRA);
  if (millis() - ultimoTempo > TEMPO_AVANCO || sensorFrente == LOW) {
    mudarEstado(PROCURANDO);
  }
}

// ------------------ LEITURA ULTRASSOM MANUAL (fallback) ------------------
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

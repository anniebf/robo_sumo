#include <AFMotor.h>
#include <Ultrasonic.h>

// ------------------ PINOS ------------------
#define TRIGGER A0  
#define ECHO    A1  
#define SENSOR_FRENTE A5  // Único sensor infravermelho

// ------------------ VELOCIDADES ------------------
#define VEL_BUSCA   180
#define VEL_ATAQUE  255
#define VEL_MANOBRA 180
#define VEL_GIRO    150  // Aumentado para compensar apenas 2 motores

// ------------------ TEMPOS ------------------
#define TEMPO_GIRO    300
#define TEMPO_GIRO_180  1800  // Ajustado para 2 motores
#define TEMPO_RECUO   700
#define TEMPO_AVANCO  700
#define DEBOUNCE_TEMPO 50  
#define TEMPO_PAUSA_ULTRA 200

// ------------------ OBJETOS ------------------
AF_DCMotor motorEsquerdo(1);  // M1 - Motor esquerdo (INVERTIDO)
AF_DCMotor motorDireito(2);   // M2 - Motor direito
Ultrasonic ultrasonic(TRIGGER, ECHO);

// ------------------ VARIÁVEIS ------------------
unsigned long ultimoTempo = 0;
unsigned long pausaUltrassomAte = 0;
unsigned long ultimoDebounceFrente = 0;

int ultimoSensorFrente = LOW;

bool sentidoGiro = true;  // alterna direção de busca (true=direita, false=esquerda)
int contadorAtaquesFrontais = 0;

enum Estado { 
  PROCURANDO, 
  ATACANDO, 
  RECUANDO, 
  AVANCANDO, 
  PARADO 
};
Estado estadoAtual = PROCURANDO;

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(9600);
  Serial.println("=== Robô Sumô 2 Motores - Arena Branca ===");

  motorEsquerdo.setSpeed(VEL_BUSCA);
  motorDireito.setSpeed(VEL_BUSCA);

  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(SENSOR_FRENTE, INPUT_PULLUP);

  Serial.println("Inicializado!");
  delay(1000);
}

// ------------------ LOOP PRINCIPAL ------------------
void loop() {
  int sFrente = lerSensorLinhaDigital(SENSOR_FRENTE, ultimoSensorFrente, ultimoDebounceFrente);

  float dist;

  // Pausa ultrassom ao detectar borda
  if (sFrente == HIGH) {
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
  Serial.print(" | Estado: "); Serial.println(estadoAtual);

  // PRIORIDADE MÁXIMA: sensor de linha (borda preta)
  if (sFrente == HIGH) {
    Serial.println("!!! BORDA DETECTADA !!!");
    mudarEstado(RECUANDO);
    executarEstado(dist, sFrente);
    return;
  }

  // Detecção do oponente
  if (dist <= 60 && dist > 0) {
    if (estadoAtual != ATACANDO) {
      mudarEstado(ATACANDO);
    }
  } 
  else if (estadoAtual != RECUANDO && estadoAtual != AVANCANDO) {
    mudarEstado(PROCURANDO);
  }

  executarEstado(dist, sFrente);
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

void executarEstado(float dist, int sf) {
  switch (estadoAtual) {
    case PROCURANDO: procura(); break;
    case ATACANDO: ataque(dist, sf); break;
    case RECUANDO: recuar(sf); break;
    case AVANCANDO: avancar(sf); break;
    case PARADO: parada(); break;
  }
}

// ------------------ MOVIMENTOS BÁSICOS ------------------
// ATENÇÃO: M1 está invertido (FORWARD vai para trás, BACKWARD vai para frente)

void frente(int vel = VEL_BUSCA) {
  motorEsquerdo.setSpeed(vel);
  motorDireito.setSpeed(vel);
  motorEsquerdo.run(BACKWARD);  // INVERTIDO: usa BACKWARD para ir frente
  motorDireito.run(FORWARD);
}

void tras(int vel = VEL_MANOBRA) {
  motorEsquerdo.setSpeed(vel);
  motorDireito.setSpeed(vel);
  motorEsquerdo.run(FORWARD);   // INVERTIDO: usa FORWARD para ir trás
  motorDireito.run(BACKWARD);
}

void esquerda(int vel = VEL_GIRO) {
  motorEsquerdo.setSpeed(vel);
  motorDireito.setSpeed(vel);
  motorEsquerdo.run(FORWARD);   // INVERTIDO: gira no próprio eixo
  motorDireito.run(FORWARD);
}

void direita(int vel = VEL_GIRO) {
  motorEsquerdo.setSpeed(vel);
  motorDireito.setSpeed(vel);
  motorEsquerdo.run(BACKWARD);  // INVERTIDO: gira no próprio eixo
  motorDireito.run(BACKWARD);
}

void parada() {
  motorEsquerdo.run(RELEASE);
  motorDireito.run(RELEASE);
}

// ------------------ AÇÕES ESTRATÉGICAS ------------------

void procura() {
  // Gira alternando direções para cobrir área
  unsigned long tempoDecorrido = millis() - ultimoTempo;
  
  if (tempoDecorrido < 2000) {
    // Gira em uma direção por 2 segundos
    if (sentidoGiro) {
      direita(VEL_GIRO);
    } else {
      esquerda(VEL_GIRO);
    }
  } else if (tempoDecorrido < 2500) {
    // Pequeno avanço
    frente(VEL_BUSCA);
  } else {
    // Reinicia ciclo alternando direção
    sentidoGiro = !sentidoGiro;
    ultimoTempo = millis();
  }
}

void ataque(float dist, int sensorFrente) {
  if (dist <= 60 && dist > 0 && sensorFrente == LOW) {
    Serial.println(">>> ATAQUE!");
    frente(VEL_ATAQUE);
  } else {
    mudarEstado(PROCURANDO);
  }
}

void recuar(int sensorFrente) {
  if (millis() - ultimoTempo < TEMPO_RECUO) {
    // Fase 1: Recuar da borda
    Serial.println(">>> RECUANDO DA BORDA!");
    tras(VEL_MANOBRA);
  } 
  else if (millis() - ultimoTempo < (TEMPO_RECUO + TEMPO_GIRO_180)) {
    // Fase 2: Girar 180 graus
    Serial.println(">>> GIRANDO 180 GRAUS!");
    direita(VEL_GIRO);
    
    // Verifica se detectou borda durante o giro
    if (sensorFrente == HIGH) {
      Serial.println(">>> BORDA DURANTE GIRO!");
      mudarEstado(RECUANDO);
      ultimoTempo = millis();  // Reinicia tempo
      return;
    }
  } 
  else {
    // Fase 3: Avançar de volta para arena
    Serial.println(">>> RETORNANDO PARA ARENA!");
    frente(VEL_MANOBRA);
    delay(400);  // avança por 400ms
    mudarEstado(PROCURANDO);
  }
}

void avancar(int sensorFrente) {
  if (millis() - ultimoTempo < TEMPO_AVANCO) {
    Serial.println(">>> AVANCANDO!");
    frente(VEL_MANOBRA);
  } 
  else {
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

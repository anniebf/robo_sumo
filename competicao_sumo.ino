#include <AFMotor.h>
#include <Ultrasonic.h>

// ------------------ PINOS ------------------
#define TRIGGER A0
#define ECHO A1
#define SENSOR_FRENTE A4
#define SENSOR_TRAS A5

// ------------------ OBJETOS ------------------
AF_DCMotor motor1(4);  // Motor traseiro esquerdo
AF_DCMotor motor2(1);  // Motor dianteiro esquerdo
AF_DCMotor motor3(3);  // Motor dianteiro direito
AF_DCMotor motor4(2);  // Motor traseiro direito

Ultrasonic ultrasonic(TRIGGER, ECHO);

// ------------------ CONSTANTES ------------------
const int DIST_ATAQUE = 30;  // 30CM
const int PRETO = 1;
const int BRANCO = 0;
const int THRESHOLD = 241;
const bool CALIBRAR_INICIO = false;
const bool DEBUG = true;

const int searchSpeed = 200;
const int atackSpeed = 255;

// ------------------ VARIÁVEIS DE ESTADO ------------------
unsigned long tempoUltimaAcao = 0;
enum Estado { NORMAL, EVITANDO_BORDA_FRENTE, EVITANDO_BORDA_TRAS };
Estado estadoAtual = NORMAL;
int etapaEvitacao = 0;

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(9600);

  // eva v1.0
  Serial.println("EVA V1.0");

  modoProcura();

  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(SENSOR_FRENTE, INPUT);
  pinMode(SENSOR_TRAS, INPUT);

  if (CALIBRAR_INICIO) {
    calibrar();
  }

  delay(6000);  // Delay de segurança
  Serial.println("🚀 INICIADO!");
}

// ------------------ FUNÇÕES AUXILIARES ------------------
// Detecta cor da superfície (TCRT5000)
bool detectarBorda(int pino) {
  int valor = analogRead(pino);
  return (valor < THRESHOLD) ? BRANCO : PRETO;
}

// Detecta distância (Ultrassônico)
float lerDistancia() {
  float distancia = ultrasonic.read();
  // Validação: retorna -1 se leitura inválida
  if (distancia <= 0 || distancia > 400)
    return -1;

  return distancia;
}

// ------------------ DETECÇÃO ------------------
void detectaAlvo() {
  float alvo = lerDistancia();

  if (alvo == -1) {
    // Sem leitura válida - continua procurando
    modoProcura();
    frente();
    return;
  }

  // Lê sensores uma vez
  int sensorFrente = detectarBorda(SENSOR_FRENTE);
  int sensorTras = detectarBorda(SENSOR_TRAS);

  bool foraDoRange = (alvo > DIST_ATAQUE);
  bool noRange = (alvo <= DIST_ATAQUE && alvo > 5);
  bool naArena = (sensorFrente == BRANCO && sensorTras == BRANCO);

  // Procurando oponente - MOVIMENTA SEMPRE
  if (foraDoRange && naArena) {
    modoProcura();
    
    // Movimento contínuo de busca
    static unsigned long ultimaMudanca = 0;
    static int padrao = 0;
    
    if (millis() - ultimaMudanca > 2000) {
      ultimaMudanca = millis();
      padrao = (padrao + 1) % 2;
    }
    
    if (padrao == 0) {
      frente();
    } else {
      esquerda();
    }
  }

  // Oponente detectado - ATAQUE!
  if (noRange && naArena) {
    modoAtaque();
    Serial.println("🎯 ALVO DETECTADO - ATACANDO!");
    frente();  // Ataca direto
  }
}

// ------------------ MÁQUINA DE ESTADOS PARA EVITAR BORDA ------------------
void executaEvitacaoFrente(unsigned long agora) {
  modoProcura();
  
  switch(etapaEvitacao) {
    case 0: // Parada inicial
      parada();
      if (agora - tempoUltimaAcao >= 100) {
        etapaEvitacao = 1;
        tempoUltimaAcao = agora;
      }
      break;
      
    case 1: // Ré
      tras();
      if (agora - tempoUltimaAcao >= 500) {
        etapaEvitacao = 2;
        tempoUltimaAcao = agora;
      }
      break;
      
    case 2: // Parada antes de girar
      parada();
      if (agora - tempoUltimaAcao >= 50) {
        etapaEvitacao = 3;
        tempoUltimaAcao = agora;
      }
      break;
      
    case 3: // Gira esquerda
      esquerda();
      if (agora - tempoUltimaAcao >= 400) {
        etapaEvitacao = 4;
        tempoUltimaAcao = agora;
      }
      break;
      
    case 4: // Para e volta ao normal
      parada();
      estadoAtual = NORMAL;
      Serial.println("✓ Evasão frontal completa");
      break;
  }
}

void executaEvitacaoTras(unsigned long agora) {
  modoProcura();
  
  switch(etapaEvitacao) {
    case 0: // Parada inicial
      parada();
      if (agora - tempoUltimaAcao >= 100) {
        etapaEvitacao = 1;
        tempoUltimaAcao = agora;
      }
      break;
      
    case 1: // Frente
      frente();
      if (agora - tempoUltimaAcao >= 500) {
        etapaEvitacao = 2;
        tempoUltimaAcao = agora;
      }
      break;
      
    case 2: // Parada antes de girar
      parada();
      if (agora - tempoUltimaAcao >= 50) {
        etapaEvitacao = 3;
        tempoUltimaAcao = agora;
      }
      break;
      
    case 3: // Gira esquerda
      esquerda();
      if (agora - tempoUltimaAcao >= 400) {
        etapaEvitacao = 4;
        tempoUltimaAcao = agora;
      }
      break;
      
    case 4: // Para e volta ao normal
      parada();
      estadoAtual = NORMAL;
      Serial.println("✓ Evasão traseira completa");
      break;
  }
}

// ------------------ LOOP PRINCIPAL (NÃO-BLOQUEANTE) ------------------
void loop() {
  unsigned long agora = millis();
  
  // Sempre lê os sensores
  float alvo = lerDistancia();
  int sensorFrente = detectarBorda(SENSOR_FRENTE);
  int sensorTras = detectarBorda(SENSOR_TRAS);
  
  if (DEBUG) {
    Serial.print("Distância: ");
    Serial.print(alvo);
    Serial.print(" | Estado: ");
    Serial.println(estadoAtual);
  }
  
  // Máquina de estados para evitar borda
  switch(estadoAtual) {
    case NORMAL:
      // Detecta bordas (PRIORIDADE)
      if (sensorFrente == PRETO) {
        estadoAtual = EVITANDO_BORDA_FRENTE;
        etapaEvitacao = 0;
        tempoUltimaAcao = agora;
        Serial.println("⚠️ BORDA FRONTAL DETECTADA!");
      }
      else if (sensorTras == PRETO) {
        estadoAtual = EVITANDO_BORDA_TRAS;
        etapaEvitacao = 0;
        tempoUltimaAcao = agora;
        Serial.println("⚠️ BORDA TRASEIRA DETECTADA!");
      }
      else {
        // Comportamento normal - busca e ataque
        detectaAlvo();
      }
      break;
      
    case EVITANDO_BORDA_FRENTE:
      executaEvitacaoFrente(agora);
      break;
      
    case EVITANDO_BORDA_TRAS:
      executaEvitacaoTras(agora);
      break;
  }
  
  delay(50);
}

// ------------------ MOVIMENTOS ------------------
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

// Função de procura removida - agora é gerenciada no detectaAlvo()

// Ataque (máxima velocidade)
void modoAtaque() {
  motor1.setSpeed(atackSpeed);
  motor2.setSpeed(atackSpeed);
  motor3.setSpeed(atackSpeed);
  motor4.setSpeed(atackSpeed);
}

// Procura (velocidade média)
void modoProcura() { 
  motor1.setSpeed(searchSpeed);
  motor2.setSpeed(searchSpeed);
  motor3.setSpeed(searchSpeed);
  motor4.setSpeed(searchSpeed);
}

// ------------------ CALIBRAÇÃO ------------------
void calibrar() {
  Serial.println("╔════════════════════════════════════╗");
  Serial.println("║   CALIBRAÇÃO DOS SENSORES TCRT    ║");
  Serial.println("╚════════════════════════════════════╝");
  Serial.println();

  // Teste 1: BRANCO
  Serial.println(">>> PASSO 1: Coloque o robô no BRANCO");
  Serial.println("Aguardando 8 segundos...");
  delay(8000);

  int branco_frente = analogRead(SENSOR_FRENTE);
  int branco_tras = analogRead(SENSOR_TRAS);

  Serial.println("✓ Leitura no BRANCO:");
  Serial.print("  Sensor Frente: ");
  Serial.println(branco_frente);
  Serial.print("  Sensor Tras:   ");
  Serial.println(branco_tras);
  Serial.println();

  delay(2000);

  // Teste 2: PRETO
  Serial.println(">>> PASSO 2: Coloque o robô no PRETO");
  Serial.println("Aguardando 8 segundos...");
  delay(8000);

  int preto_frente = analogRead(SENSOR_FRENTE);
  int preto_tras = analogRead(SENSOR_TRAS);

  Serial.println("✓ Leitura no PRETO:");
  Serial.print("  Sensor Frente: ");
  Serial.println(preto_frente);
  Serial.print("  Sensor Tras:   ");
  Serial.println(preto_tras);
  Serial.println();

  // Cálculo do THRESHOLD ideal
  int threshold_frente = (branco_frente + preto_frente) / 2;
  int threshold_tras = (branco_tras + preto_tras) / 2;
  int threshold_medio = (threshold_frente + threshold_tras) / 2;

  Serial.println("╔════════════════════════════════════╗");
  Serial.println("║          RESULTADOS               ║");
  Serial.println("╚════════════════════════════════════╝");
  Serial.print("THRESHOLD Sensor Frente: ");
  Serial.println(threshold_frente);
  Serial.print("THRESHOLD Sensor Tras:   ");
  Serial.println(threshold_tras);
  Serial.println();
  Serial.print(">>> THRESHOLD RECOMENDADO: ");
  Serial.println(threshold_medio);
  Serial.println("");
  Serial.println("Copie este valor e cole na linha:");
  Serial.print("const int THRESHOLD = ");
  Serial.print(threshold_medio);
  Serial.println(";");
  Serial.println("════════════════════════════════════");

  // Loop infinito para não sair da calibração
  while (true) {
    delay(1000);
  }
}

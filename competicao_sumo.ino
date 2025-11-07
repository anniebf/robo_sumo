#include <AFMotor.h>
#include <Ultrasonic.h>

// ------------------ PINOS ------------------
#define TRIGGER A0  
#define ECHO    A1  
#define SENSOR_FRENTE A4  // TCRT5000 frontal (analógico)
#define SENSOR_TRAS   A5  // TCRT5000 traseiro (analógico)

// ------------------ CALIBRAÇÃO TCRT5000 ------------------
#define THRESHOLD_BRANCO 700  // Valor ajustável: acima disso = branco
#define THRESHOLD_PRETO  400  // Valor ajustável: abaixo disso = preto
#define NUM_LEITURAS 5        // Número de leituras para média

// ------------------ VELOCIDADES ------------------
#define VEL_BUSCA   180
#define VEL_ATAQUE  255
#define VEL_MANOBRA 180
#define VEL_GIRO    100  
#define VEL_DESVIO  220

// ------------------ TEMPOS ------------------
#define TEMPO_GIRO    100
#define TEMPO_GIRO_180  1600
#define TEMPO_RECUO   600    // AUMENTADO: recua mais da borda
#define TEMPO_AVANCO  600    // AUMENTADO: avança mais da borda
#define TEMPO_DESVIO  100
#define TEMPO_POSICAO_INICIAL 100
#define DEBOUNCE_TEMPO 50  
#define TEMPO_PAUSA_ULTRA 1000

// ------------------ DISTÂNCIAS ULTRASSÔNICO ------------------
#define DIST_ATAQUE_MIN 10   // Distância mínima para atacar (cm)
#define DIST_ATAQUE_MAX 15   // Distância máxima para atacar (cm)

// ------------------ OBJETOS ------------------
AF_DCMotor motor4(4); // traseira esquerda
AF_DCMotor motor2(2); // traseira direita
AF_DCMotor motor3(3); // dianteira direita
AF_DCMotor motor1(1); // dianteira esquerda
Ultrasonic ultrasonic(TRIGGER, ECHO);

// ------------------ VARIÁVEIS ------------------
unsigned long ultimoTempo = 0;
unsigned long pausaUltrassomAte = 0;

// Calibração dos sensores TCRT
int thresholdFrente = (THRESHOLD_BRANCO + THRESHOLD_PRETO) / 2;
int thresholdTras = (THRESHOLD_BRANCO + THRESHOLD_PRETO) / 2;

bool posicaoInicialFeita = false;
bool sentidoDesvio = true;
int contadorAtaquesFrontais = 0;

enum Estado { 
  CALIBRANDO,     // novo estado para calibração
  POSICIONANDO,
  PROCURANDO, 
  ATACANDO, 
  ATACANDO_LATERAL,
  DESVIANDO,
  RECUANDO, 
  AVANCANDO, 
  PARADO 
};
Estado estadoAtual = CALIBRANDO;

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(9600);
  Serial.println("=== Robô Sumô v8.0 - TCRT Analógico ===");

  motor1.setSpeed(VEL_BUSCA);
  motor2.setSpeed(VEL_BUSCA);
  motor3.setSpeed(VEL_BUSCA);
  motor4.setSpeed(VEL_BUSCA);

  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(SENSOR_FRENTE, INPUT);  // Analógico
  pinMode(SENSOR_TRAS, INPUT);    // Analógico

  Serial.println("Inicializado!");
  delay(1000);
  
  // Calibração automática dos sensores
  calibrarSensores();
  
  Serial.println("Posicionando rodas dianteiras...");
  delay(500);
}

// ------------------ CALIBRAÇÃO DOS SENSORES TCRT ------------------
void calibrarSensores() {
  Serial.println(">>> CALIBRANDO SENSORES TCRT5000...");
  Serial.println(">>> Posicione o robô sobre a parte BRANCA da arena");
  delay(3000);
  
  // Calibrar sobre branco
  int somaFrente = 0;
  int somaTras = 0;
  
  for (int i = 0; i < 20; i++) {
    somaFrente += analogRead(SENSOR_FRENTE);
    somaTras += analogRead(SENSOR_TRAS);
    delay(50);
  }
  
  int valorBrancoFrente = somaFrente / 20;
  int valorBrancoTras = somaTras / 20;
  
  Serial.print("Valor BRANCO - Frente: "); Serial.println(valorBrancoFrente);
  Serial.print("Valor BRANCO - Trás: "); Serial.println(valorBrancoTras);
  
  // Calibrar sobre preto (opcional, mas recomendado)
  Serial.println(">>> Posicione o robô sobre a linha PRETA");
  delay(3000);
  
  somaFrente = 0;
  somaTras = 0;
  
  for (int i = 0; i < 20; i++) {
    somaFrente += analogRead(SENSOR_FRENTE);
    somaTras += analogRead(SENSOR_TRAS);
    delay(50);
  }
  
  int valorPretoFrente = somaFrente / 20;
  int valorPretoTras = somaTras / 20;
  
  Serial.print("Valor PRETO - Frente: "); Serial.println(valorPretoFrente);
  Serial.print("Valor PRETO - Trás: "); Serial.println(valorPretoTras);
  
  // Calcular threshold (ponto médio entre branco e preto)
  thresholdFrente = (valorBrancoFrente + valorPretoFrente) / 2;
  thresholdTras = (valorBrancoTras + valorPretoTras) / 2;
  
  Serial.print("THRESHOLD Frente: "); Serial.println(thresholdFrente);
  Serial.print("THRESHOLD Trás: "); Serial.println(thresholdTras);
  
  Serial.println(">>> CALIBRAÇÃO CONCLUÍDA!");
  delay(1000);
  
  estadoAtual = POSICIONANDO;
}

// ------------------ LOOP PRINCIPAL ------------------
void loop() {
  // Leitura analógica dos sensores TCRT com média
  int sFrente = lerSensorLinhaAnalogico(SENSOR_FRENTE, thresholdFrente);
  int sTras   = lerSensorLinhaAnalogico(SENSOR_TRAS, thresholdTras);

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

  // PRIORIDADE MÁXIMA: sensores de linha (SEMPRE verifica primeiro!)
  // Interrompe QUALQUER ação se detectar borda
  if (sFrente == HIGH && estadoAtual != RECUANDO) {
    Serial.println("!!! BORDA FRONTAL DETECTADA !!!");
    parada();  // PARA IMEDIATAMENTE!
    delay(100); // Pausa maior para garantir parada completa
    mudarEstado(RECUANDO);
    executarEstado(dist, sFrente, sTras);
    return;
  }

  if (sTras == HIGH && estadoAtual != AVANCANDO) {
    Serial.println("!!! BORDA TRASEIRA DETECTADA !!!");
    parada();  // PARA IMEDIATAMENTE!
    delay(100); // Pausa maior para garantir parada completa
    mudarEstado(AVANCANDO);
    executarEstado(dist, sFrente, sTras);
    return;
  }

  // Debug apenas quando não está em manobra de borda
  if (estadoAtual != RECUANDO && estadoAtual != AVANCANDO) {
    Serial.print("Dist: "); Serial.print(dist);
    Serial.print(" cm | Frente: "); Serial.print(sFrente);
    Serial.print(" | Tras: "); Serial.print(sTras);
    Serial.print(" | Estado: "); Serial.println(estadoAtual);
  }

  // Detecção do oponente com nova distância (10-15 cm)
  if (dist >= DIST_ATAQUE_MIN && dist <= DIST_ATAQUE_MAX) {
    if (estadoAtual != DESVIANDO && estadoAtual != ATACANDO_LATERAL && 
        estadoAtual != RECUANDO && estadoAtual != AVANCANDO) {
      // A cada 2 ataques frontais, faz um desvio
      if (contadorAtaquesFrontais >= 2) {
        mudarEstado(DESVIANDO);
        contadorAtaquesFrontais = 0;
        sentidoDesvio = !sentidoDesvio;
      } else {
        mudarEstado(ATACANDO);
      }
    }
  } 
  else if (estadoAtual != RECUANDO && estadoAtual != AVANCANDO && 
           estadoAtual != DESVIANDO && estadoAtual != POSICIONANDO &&
           estadoAtual != ATACANDO_LATERAL && estadoAtual != CALIBRANDO) {
    mudarEstado(PROCURANDO);
  }

  executarEstado(dist, sFrente, sTras);
}

// ------------------ LEITURA SENSOR TCRT ANALÓGICO ------------------
int lerSensorLinhaAnalogico(int pino, int threshold) {
  // Faz múltiplas leituras rápidas para detecção imediata
  long soma = 0;
  int deteccaoImediata = 0;  // contador de detecções de preto
  
  for (int i = 0; i < NUM_LEITURAS; i++) {
    int leitura = analogRead(pino);
    soma += leitura;
    
    // Se já detectou preto em 3 das 5 leituras, retorna imediatamente
    if (leitura < threshold) {
      deteccaoImediata++;
      if (deteccaoImediata >= 3) {
        Serial.print("!!! PRETO DETECTADO IMEDIATAMENTE no pino A");
        Serial.print(pino); Serial.print(" (valor: ");
        Serial.print(leitura); Serial.println(")");
        return HIGH;  // Detecção imediata!
      }
    }
    
    delayMicroseconds(100);
  }
  
  int valorMedio = soma / NUM_LEITURAS;
  
  // Debug: mostrar valor lido periodicamente
  static unsigned long ultimoDebug = 0;
  if (millis() - ultimoDebug > 1000) {
    Serial.print("Sensor A"); Serial.print(pino);
    Serial.print(": "); Serial.print(valorMedio);
    Serial.print(" (threshold: "); Serial.print(threshold);
    Serial.print(") | Detecções: "); Serial.print(deteccaoImediata);
    Serial.println("/5");
    ultimoDebug = millis();
  }
  
  // Decisão final baseada na média
  if (valorMedio < threshold) {
    Serial.print(">>> BORDA! Sensor A"); Serial.print(pino);
    Serial.print(" média: "); Serial.println(valorMedio);
    return HIGH;  // Detectou linha preta (borda)
  } else {
    return LOW;   // Está sobre branco (arena)
  }
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
    case CALIBRANDO: break;  // Já foi feito no setup
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
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void tras(int vel = VEL_MANOBRA) {
  setVelocidade(vel);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void esquerda(int vel = VEL_GIRO) {
  setVelocidade(vel);
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

void direita(int vel = VEL_GIRO) {
  setVelocidade(vel);
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

// ------------------ MOVIMENTOS ANGULARES ------------------
void diagonalDireita(int vel = VEL_ATAQUE) {
  motor1.setSpeed(vel);
  motor2.setSpeed(vel * 0.6);
  motor3.setSpeed(vel * 0.6);
  motor4.setSpeed(vel);
  
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void diagonalEsquerda(int vel = VEL_ATAQUE) {
  motor1.setSpeed(vel * 0.6);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel * 0.6);
  
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void desvioRapidoDireita(int vel = VEL_DESVIO) {
  motor1.setSpeed(vel);
  motor2.setSpeed(vel * 0.3);
  motor3.setSpeed(vel * 0.3);
  motor4.setSpeed(vel);
  
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

void desvioRapidoEsquerda(int vel = VEL_DESVIO) {
  motor1.setSpeed(vel * 0.3);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel * 0.3);
  
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
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
void posicionarRodas() {
  if (!posicaoInicialFeita) {
    Serial.println(">>> Posicionando rodas dianteiras em 45 graus");
    
    motor1.setSpeed(150);
    motor3.setSpeed(150);
    motor2.setSpeed(0);
    motor4.setSpeed(0);
    
    motor1.run(FORWARD);
    motor3.run(BACKWARD);
    
    delay(TEMPO_POSICAO_INICIAL);
    
    parada();
    posicaoInicialFeita = true;
    
    Serial.println(">>> Posicionamento concluído! Iniciando busca...");
    delay(200);
    mudarEstado(PROCURANDO);
  }
}

void procura() {
  if ((millis() / 1500) % 3 == 0) {
    diagonalDireita(VEL_BUSCA);
  } else if ((millis() / 1500) % 3 == 1) {
    diagonalEsquerda(VEL_BUSCA);
  } else {
    direita(VEL_GIRO);
  }
}

void ataque(float dist, int sensorFrente) {
  if (dist >= DIST_ATAQUE_MIN && dist <= DIST_ATAQUE_MAX && sensorFrente == LOW) {
    Serial.println(">>> ATAQUE FRONTAL!");
    frente(VEL_ATAQUE);
    contadorAtaquesFrontais++;
  } else {
    mudarEstado(PROCURANDO);
  }
}

void desviar(float dist, int sensorFrente) {
  if (millis() - ultimoTempo < TEMPO_DESVIO) {
    Serial.println(">>> DESVIANDO!");
    if (sentidoDesvio) {
      desvioRapidoDireita(VEL_DESVIO);
    } else {
      desvioRapidoEsquerda(VEL_DESVIO);
    }
  } else {
    mudarEstado(ATACANDO_LATERAL);
  }
}

void ataqueLateral(float dist, int sensorFrente) {
  if (dist >= DIST_ATAQUE_MIN && dist <= DIST_ATAQUE_MAX * 1.5 && sensorFrente == LOW) {
    Serial.println(">>> ATAQUE LATERAL!");
    if (sentidoDesvio) {
      diagonalDireita(VEL_ATAQUE);
    } else {
      diagonalEsquerda(VEL_ATAQUE);
    }
    
    if (millis() - ultimoTempo > 1000) {
      mudarEstado(PROCURANDO);
    }
  } else {
    mudarEstado(PROCURANDO);
  }
}

// CORRIGIDO: Recuar da borda frontal - MAIS AGRESSIVO
void recuar(int sensorTras) {
  if (millis() - ultimoTempo < TEMPO_RECUO) {
    // Fase 1: Recuar RÁPIDO e FORTE da borda
    Serial.println(">>> RECUANDO DA BORDA FRONTAL!");
    tras(VEL_ATAQUE);  // Recua na velocidade máxima!
  } 
  else if (millis() - ultimoTempo < (TEMPO_RECUO + TEMPO_GIRO_180)) {
    // Fase 2: Girar 180 graus
    Serial.println(">>> GIRANDO 180 GRAUS!");
    direita(VEL_MANOBRA);  // Gira mais rápido
    
    // Verifica se detectou borda traseira durante o giro
    if (sensorTras == HIGH) {
      Serial.println(">>> BORDA TRASEIRA DURANTE GIRO!");
      parada();
      delay(50);
      mudarEstado(AVANCANDO);
      return;
    }
  } 
  else {
    // Fase 3: Avançar de volta para arena
    Serial.println(">>> RETORNANDO PARA ARENA!");
    frente(VEL_ATAQUE);  // Avança rápido para centro
    delay(500);  // AUMENTADO: avança por 500ms
    mudarEstado(PROCURANDO);
  }
}

// CORRIGIDO: Avançar da borda traseira - MAIS AGRESSIVO
void avancar(int sensorFrente) {
  if (millis() - ultimoTempo < TEMPO_AVANCO) {
    // Fase 1: Avançar RÁPIDO e FORTE para sair da borda traseira
    Serial.println(">>> AVANCANDO DA BORDA TRASEIRA!");
    frente(VEL_ATAQUE);  // Avança na velocidade máxima!
  } 
  else if (millis() - ultimoTempo < (TEMPO_AVANCO + TEMPO_GIRO_180)) {
    // Fase 2: Girar 180 graus
    Serial.println(">>> GIRANDO 180 GRAUS!");
    direita(VEL_MANOBRA);  // Gira mais rápido
    
    // Verifica se detectou borda frontal durante o giro
    if (sensorFrente == HIGH) {
      Serial.println(">>> BORDA FRONTAL DURANTE GIRO!");
      parada();
      delay(50);
      mudarEstado(RECUANDO);
      return;
    }
  } 
  else {
    // Fase 3: Continuar para centro da arena
    Serial.println(">>> RETORNANDO PARA ARENA!");
    frente(VEL_ATAQUE);  // Avança rápido para centro
    delay(500);  // AUMENTADO: avança por 500ms
    mudarEstado(PROCURANDO);
  }
}

// ------------------ LEITURA ULTRASSOM MANUAL ------------------
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

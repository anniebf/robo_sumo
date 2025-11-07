// Pinos dos sensores TCRT5000
const int sensorTras = A5;   // Sensor de trás (no preto)
const int sensorFrente = A4; // Sensor da frente (ajustando potenciômetro)

void setup() {
  Serial.begin(9600);
  
  pinMode(sensorTras, INPUT);
  pinMode(sensorFrente, INPUT);
  
  Serial.println("=== LEITURA CONTINUA SENSORES TCRT5000 ===");
  Serial.println("Sensor Tras (A0) | Sensor Frente (A1)");
  Serial.println("==========================================");
}

void loop() {
  // Lê os sensores
  int leituraTras = analogRead(sensorTras);
  int leituraFrente = analogRead(sensorFrente);
  
  // Exibe de forma clara
  Serial.print("Tras: ");
  Serial.print(leituraTras);
  Serial.print("\t|\tFrente: ");
  Serial.println(leituraFrente);
  
  delay(200); // Atualiza 5x por segundo
}

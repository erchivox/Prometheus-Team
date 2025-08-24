#define IN1 27
#define IN2 26
#define ENA 14
#define SIGNAL_PIN_ENA 15
#define SIGNAL_PIN_ENA_BACK 19

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT); 
    
    // CONFIGURAR PULL-UP CORRECTAMENTE
    pinMode(SIGNAL_PIN_ENA, INPUT_PULLUP);      // Esto activa pull-up interna
    pinMode(SIGNAL_PIN_ENA_BACK, INPUT_PULLUP); // Esto activa pull-up interna
    
    Serial.begin(9600);
    
    // Motor inicialmente detenido
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
}

void loop() {
    Serial.println("vuelta");
    
    int signalBACK = digitalRead(SIGNAL_PIN_ENA_BACK);
    int signal = digitalRead(SIGNAL_PIN_ENA);
    
    // CON PULL-UP: 
    // - Señal activa = LOW (cuando el maestro conecta a GND)
    // - Sin señal = HIGH (por la resistencia pull-up)
    
    if (signal == LOW) {  // Cambiar a LOW porque usas pull-up
        Serial.println("pa delante");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        //analogWrite(ENA, 255);
        analogWrite(ENA, 170);
    } 
    else if (signalBACK == LOW) {  // Cambiar a LOW porque usas pull-up
        Serial.println("retroceso");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        //analogWrite(ENA, 255);
        analogWrite(ENA, 150);
    } 
    else {
        Serial.println("parao :C");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 0); 
    }
    
    delay(10);
}
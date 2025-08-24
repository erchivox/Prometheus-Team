#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <ColorConverterLib.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
MPU6050 mpu;
Servo direccion;
#define I2C_SDA 21
#define I2C_SCL 22
const int MOTOR_RETROCESO_PIN = 19;  // Pin para motor de retroceso
const int MOTOR_PIN = 15;
const int SERVO_PIN = 13;
// Declaraciones de pines (usar pines analógicos como digitales)
#define LED_PIN 12      // Pin analógico A0 usado como digital para LED
#define BUZZER_PIN 14   // Pin analógico A1 usado como digital para buzzer


// Pin para sensor ultrasónico frontal (emergencia)
const int TRIG_FRONTAL = 4;
const int ECHO_FRONTAL = 36;

const int INTERRUPTOR_PIN = 27;  // Pin digital para el interruptor (usa pin con interrupt)

// Variables de estado del sistema
bool sistemaIniciado = false;
bool interruptorAnterior = false;
unsigned long tiempoDebounce = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// ===== CONFIGURACIÓN DE SERVO =====
const int SERVO_RIGHT = 0;
const int SERVO_CENTER = 90;
const int SERVO_LEFT  = 180;

const int SERVO_OrientadoObjeto_LEFT = 130;
const int SERVO_OrientadoObjeto_RIGH = 50;

// Pines para sensores ultrasónicos
const int TRIG_DERECHO = 16;
const int ECHO_DERECHO = 39;
const int TRIG_IZQUIERDO = 17;
const int ECHO_IZQUIERDO = 34;

const int SERVO_CENTRADO = 90;

int contadorEntradaEstadoFinal = 0;

// Contador de líneas
int contadorLineas = 0;
int limiteLineas = 12;

int16_t ax, ay, az, gx, gy, gz;

float anguloZ = 0.0;        // Ángulo Yaw (giro alrededor del eje Z)
float pitch = 0.0;          // Ángulo Pitch (inclinación adelante/atrás)
float roll = 0.0;           // Ángulo Roll (inclinación lateral)
float anguloObjetivoGlobal = 0.0; // angulo recto para el giro

// Factores de peso para el filtro complementario
// K_ACC: Peso del acelerómetro (0.0 a 1.0). Un valor pequeño es común.
// K_GYRO: Peso del giroscopio (1.0 - K_ACC).
const float K_ACC = 0.02; // Puedes ajustar este valor. Más alto = más influencia del acelerómetro, más bajo = más influencia del giroscopio.
const float K_GYRO = 1.0 - K_ACC;

// Ángulos de giro modificables para cada sentido
float giroObjetivoAzul = 90.0;    // Giro a la izquierda para línea azul
float giroObjetivoNaranja = 90.0; // Giro a la derecha para línea naranja (modificable)


float toleranciaGiro = 5.0;
float offsetGz = 0.0;
float offsetGx = 0.0; // Offset del giroscopio en X (Roll)
float offsetGy = 0.0; // Offset del giroscopio en Y (Pitch)

float umbralGiro = 0.5;//1.0 antes
float umbralReposo = 0.5;

//mantener este timeout o aumentarlo para que termine el giro correctamente.
unsigned long lastTime, startTime, timeoutGiro = 1500;

unsigned long lastYawPrint = 0; // Para el intervalo de impresión del Yaw
const long YAW_INTERVAL = 300; // Intervalo de impresión en ms

// Variables para control de centrado
bool enGiro = false;
unsigned long tiempoFinGiro = 0;
const unsigned long tiempoEsperaPosgiro = 0;
const int anguloCorreccionMax = 50;// normal 50 16/07/2025
const unsigned long intervaloCentrado = 100;
unsigned long ultimoTiempoCentrado = 0;


// SISTEMA HÍBRIDO MPU6050 + QMC5883L

// Dirección I2C del QMC5883L
#define QMC5883L_ADDRESS 0x0D

// Registros del QMC5883L
#define QMC5883L_DATA_OUTPUT_X_LSB 0x00
#define QMC5883L_DATA_OUTPUT_X_MSB 0x01
#define QMC5883L_DATA_OUTPUT_Y_LSB 0x02
#define QMC5883L_DATA_OUTPUT_Y_MSB 0x03
#define QMC5883L_DATA_OUTPUT_Z_LSB 0x04
#define QMC5883L_DATA_OUTPUT_Z_MSB 0x05
#define QMC5883L_STATUS 0x06
#define QMC5883L_CONTROL_1 0x09
#define QMC5883L_CONTROL_2 0x0A
#define QMC5883L_SET_RESET_PERIOD 0x0B

// Variables globales necesarias 
float declinacionMagnetica = -9.0; // Para Venezuela - ajustar según tu ubicación
float offsetAngular = 0.0; // Offset adicional si necesitas ajustar el norte de referencia
/*
// Valores de calibración
float xOffset = -242.5;    // Tu valor obtenido
float yOffset = 92.5;      // Tu valor obtenido  
float xScale = 1.008;      // Tu valor obtenido
float yScale = 0.992;      // Tu valor obtenido
*/
float xOffset = -427.5;//0.0;    
float yOffset = -199.0;//0.0;    
float xScale = 1.021;//1.0;     
float yScale = 0.980;//1.0;
// ========== VARIABLES GLOBALES ADICIONALES ==========
float anguloMagnetometro = 0;
float factorCorreccion = 0.0;
unsigned long ultimaCorreccionMag = 0;
const unsigned long INTERVALO_MAGNETOMETRO = 500; // ms
const float UMBRAL_DIFERENCIA_MAXIMA = 5.0; // grados
const float FACTOR_FUSION_COMPLEMENTARIO = 0.98; // 0.98 = 98% gyro, 2% mag

// Buffer para promedio móvil del magnetómetro
const int BUFFER_SIZE_MAGNETOMETRO = 5;
float bufferMagnetometro[BUFFER_SIZE_MAGNETOMETRO];
int indiceBufer = 0;
bool bufferLleno = false;


// Variables para pausar centrado durante detección
bool pausarCentrado = false;
unsigned long tiempoInicioDeteccion = 0;
const unsigned long tiempoPausaCentrado = 100;

// Variables para detección de líneas
bool lineaNaranjaDetectada = false;
unsigned long tiempoUltimaDeteccion = 0;
const unsigned long tiempoEsperaDeteccion = 500;
// Variables para direccionalidad
enum SentidoGiro {
  INDEFINIDO,
  IZQUIERDA,  // Detectó azul primero
  DERECHA     // Detectó naranja primero
};

SentidoGiro sentidoVehiculo = INDEFINIDO;
bool primeraDeteccion = true;

// ==================== VARIABLES PID PARA GIROS ====================

// Variables para controlador PID de giros
struct PIDController {
  float kp;           // Ganancia proporcional
  float ki;           // Ganancia integral
  float kd;           // Ganancia derivativa
  float error_anterior;
  float integral;
  float integral_max; // Límite para evitar windup
  unsigned long tiempo_anterior;
};

// Configuración del PID para giros
PIDController pidGiro = {
  .kp = 2.5,          // Ganancia proporcional - ajusta respuesta principal
  .ki = 0.8,          // Ganancia integral - elimina error en estado estacionario
  .kd = 0.4,          // Ganancia derivativa - reduce oscilaciones
  .error_anterior = 0,
  .integral = 0,
  .integral_max = 30, // Límite para integral windup
  .tiempo_anterior = 0
};

// Variables adicionales para control PID
float anguloObjetivoActual = 0;
bool giroEnProceso = false;
const float TOLERANCIA_PID = 2.0;      // Tolerancia más estricta con PID

// ==================== VARIABLES PID PARA CENTRADO ====================

// Variables para controlador PID de centrado
struct PIDCentrado {
  float kp;           // Ganancia proporcional
  float ki;           // Ganancia integral  
  float kd;           // Ganancia derivativa
  float error_anterior;
  float integral;
  float integral_max; // Límite para evitar windup
  unsigned long tiempo_anterior;
  bool inicializado;  // Para manejar primera ejecución
};

// Configuración del PID para centrado
PIDCentrado pidCentrado = {
  .kp = 3.8,          // Ganancia proporcional - respuesta principal al error
  .ki = 0.7,          // Ganancia integral - elimina error residual
  .kd = 0.8,          // Ganancia derivativa - reduce oscilaciones
  .error_anterior = 0,
  .integral = 0,
  .integral_max = 20, // Límite para integral windup
  .tiempo_anterior = 0,
  .inicializado = false
};

// Variables adicionales para control PID del centrado
const float TOLERANCIA_CENTRADO_PID = 5.0;    // Tolerancia más estricta
const float ERROR_MAXIMO_CENTRADO = 30.0;     // Error máximo esperado
const float FACTOR_SUAVIZADO = 0.7;           // Para suavizar cambios bruscos

// Buffer para comandos serie
String comandoRecibido = "";
  bool comandoListo = false;


//-------variables de evasion de objetos--------------

// Estados del sistema integrado
enum EstadoSistema {
  ESTADO_NORMAL,      // Navegación normal + detección de líneas + recepción comandos
  SIGUIENDO_OBJETO,   // Siguiendo objeto detectado por app
  ESQUIVANDO,         // Esquivando objeto
  ESTADO_DETENCION_FINAL
};
float distanciaFrontalInicial = 0.0;
const float TOLERANCIA_DISTANCIA_FINAL = 2.0;  // cm de tolerancia aceptable

// Variables para recepción serial optimizada (del código de esquive)
const int BUFFER_SIZE = 32;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

// Variables para parsing ultra rápido
char colorDetectado = 'N';
int distanciaObjeto = 0;
char orientacionObjeto = 'C';

// Variables de estado
EstadoSistema estadoActual = ESTADO_NORMAL;
char colorObjetivo = 'N';
unsigned long tiempoEsquive = 0;
int etapaEsquive = 0;
char lastOrientation = 'X';

// Configuración de distancias para esquive
const int DISTANCIA_ROJO = 40;//35;    // Distancia de esquive para objetos rojos
const int DISTANCIA_VERDE = 45;//40;   // Distancia de esquive para objetos verdes
const int DISTANCIA_EMERGENCIA = 8;
// Tiempos de esquive por color (en milisegundos)
const unsigned long TIEMPO_ESQUIVE_VERDE = 800;   // Tiempo para esquivar verde
const unsigned long TIEMPO_ESQUIVE_ROJO = 800;   // Tiempo para esquivar rojo
const unsigned long TIEMPO_RETORNO_VERDE = 600;   // Tiempo de retorno para verde
const unsigned long TIEMPO_RETORNO_ROJO = 600;    // Tiempo de retorno para rojo

// Variables para almacenar los tiempos actuales
unsigned long tiempoEsquiveActual = 0;
unsigned long tiempoRetornoActual = 0;

// Variables para recordar último color detectado
char ultimoColorDetectado = 'N';           // Último color válido recibido
unsigned long tiempoUltimoColor = 0;       // Timestamp del último color detectado
const unsigned long TIEMPO_MEMORIA_COLOR = 0; // 4 segundos de memoria

// Variables para función de emergencia
bool emergenciaActivada = false;
unsigned long tiempoUltimaEmergencia = 0;
const unsigned long TIEMPO_ESPERA_EMERGENCIA = 2000; // Evitar activaciones múltiples
char colorEmergencia = 'N';

// Variables para interrupción de cruce
volatile bool interrupcionCruce = false;
volatile char comandoInterrupcion = 'N';
bool cruceEnProceso = false;
//------------------- Fin de variables de esquive--------------

// ==================== CONFIGURACIÓN Y VARIABLES SHARP ====================

// Pines para sensores Sharp diagonales
const int SHARP_DIAGONAL_IZQ_PIN = 32;  // Pin analógico para sensor Sharp izquierdo
const int SHARP_DIAGONAL_DER_PIN = 33;  // Pin analógico para sensor Sharp derecho

// Variables para esquive diagonal por sensores Sharp
bool esquiveDiagonalActivo = false;
unsigned long tiempoInicioEsquiveDiagonal = 0;
unsigned long tiempoUltimaDeteccionSharp = 0;
char direccionEsquiveDiagonal = 'N'; // 'I' = izquierda, 'D' = derecha
int etapaEsquiveDiagonal = 0;

// Configuración para sensores Sharp diagonales
const float DISTANCIA_SHARP_EMERGENCIA = 15.0;     // 10cm de detección
const unsigned long TIEMPO_CRUCE_DIAGONAL = 500;  // Tiempo de cruce diagonal
const unsigned long TIEMPO_ESPERA_SHARP = 100;     // Evitar detecciones múltiples

// Sistema de prioridades para gestión de emergencias
enum PrioridadEmergencia {
  PRIORIDAD_NORMAL,
  PRIORIDAD_RETROCESO,  // Máxima prioridad
  PRIORIDAD_ESQUIVE     // Prioridad normal
};
PrioridadEmergencia prioridadActual = PRIORIDAD_NORMAL;

// Variables mejoradas para retroceso de emergencia
const int DISTANCIA_RETROCESO_EMERGENCIA = 13;    // Distancia mínima para activar retroceso (cm)
const unsigned long TIEMPO_RETROCESO = 1500;     // Tiempo de retroceso en ms
const unsigned long TIMEOUT_RETROCESO = 3000;    // Timeout de seguridad para retroceso
bool retrocesoPorEmergencia = false;    
// Variables de control temporal
unsigned long tiempoInicioRetroceso = 0;          
unsigned long tiempoUltimaVerificacionRetroceso = 0; 
const unsigned long INTERVALO_VERIFICACION_RETROCESO = 50; // Verificación cada 100ms
const unsigned long TIEMPO_ESPERA_POST_RETROCESO = 100;     // Tiempo de espera después del retroceso
bool motorRetrocesoConfigurado = false;  // controlar configuración de motores

bool retrocesoOrientadoActivo = false;
unsigned long tiempoInicioRetrocesoOrientado = 0;
const unsigned long TIEMPO_RETROCESO_ORIENTADO = 700; // 1 segundo (modificable)
int direccionRetroceso = SERVO_CENTRADO;

// Variables para control de retroceso post-giro
unsigned long tiempoInicioEvaluacion = 0;

// ============== CONSTANTES DE ÁNGULOS ==============
const float GIRO_MAXIMO = 75.0;      // Giro máximo - obstáculo muy cerca
const float GIRO_ALTO = 70.0;        // Giro alto - obstáculo cerca
const float GIRO_MEDIO_ALTO = 67.0;  // Giro medio-alto
const float GIRO_MEDIO = 63.0;       // Giro medio
const float GIRO_MEDIO_BAJO = 58.0;  // Giro medio-bajo
const float GIRO_BAJO = 53.0;        // Giro bajo - obstáculo lejano
const float GIRO_MINIMO = 48.0;      // Giro mínimo - obstáculo muy lejano

// Constantes de tiempos de retroceso (en milisegundos)
const unsigned long TIEMPO_RETROCESO_MAXIMO = 0;     // Para GIRO_MAXIMO (70°)
const unsigned long TIEMPO_RETROCESO_ALTO = 0;       // Para GIRO_ALTO (55°)
const unsigned long TIEMPO_RETROCESO_MEDIO_ALTO = 0;  // Para GIRO_MEDIO_ALTO (45°)
const unsigned long TIEMPO_RETROCESO_MEDIO = 0;       // Para GIRO_MEDIO (35°)
const unsigned long TIEMPO_RETROCESO_MEDIO_BAJO = 0;  // Para GIRO_MEDIO_BAJO (30°)
const unsigned long TIEMPO_RETROCESO_BAJO = 0;        // Para GIRO_BAJO (25°)
const unsigned long TIEMPO_RETROCESO_MINIMO = 0;      // Para GIRO_MINIMO (20°)
const unsigned long TIEMPO_RETROCESO_DEFECTO = 0;     // Valor por defecto
const unsigned long TIMEOUT_RETROCESO_MAXIMO =1300;  // Timeout de seguridad (2.5 segundos)

unsigned long tiempoRetrocesoActual = 0;  // Variable para tiempo dinámico de retroceso
//
bool sistemaActivo = false;  // Estado del sistema (activo/inactivo)

// Variable para controlar giro antes de estacionamiento
bool giroParaEstacionamiento = false;

// tiempo de finalizacion del codigo para avance final
unsigned long tiempoInicio = millis();
//unsigned long duracionCiclo = 3000; // 5 segundos en milisegundos

 // Añadir esta línea con las otras variables de estado/tiempo
unsigned long tiempoInicioDetencionFinal = 0;

// Variables globales para el control de arranque del motor
unsigned long tiempoInicioSistema = 0;
bool motorArrancado = false;
const unsigned long DELAY_ARRANQUE_MOTOR = 1000; // 1 segundo en milisegundos

// Función auxiliar para generar pulsos cortos (volumen bajo)
void buzzerVolumenBajo(int duracionTotal) {
  int tiempoTranscurrido = 0;
  while (tiempoTranscurrido < duracionTotal) {
    // Toca un tono corto de 2000 Hz por 20ms
    tone(BUZZER_PIN, 1500, 100);
    delay(100); // Pausa de 80ms + 20ms del tono
    tiempoTranscurrido += 100;
  }
}

void esperarInterruptor() {
  ////ln(F("Esperando activacion del interruptor (LOW)..."));
  
  // LED parpadea lentamente mientras espera
  bool ledState = false;
  unsigned long ultimoParpadeo = 0;

  
  while (digitalRead(INTERRUPTOR_PIN) == HIGH) {  // Espera a que pase a LOW
    unsigned long ahora = millis();
    
    // Parpadeo lento del LED (cada 500ms)
    if (ahora - ultimoParpadeo > 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      ultimoParpadeo = ahora;
    }
    
    delay(10);
  }
  
  // Cuando el interruptor pasa a LOW, activar sistema
  digitalWrite(LED_PIN, LOW); // Apagar LED de espera
  delay(50); // Pequeña pausa para debounce
  iniciarSistema();
} 
  // Función mejorada para verificar el interruptor y controlar el sistema
void verificarInterruptor() {
  bool estadoActual = digitalRead(INTERRUPTOR_PIN);
  unsigned long tiempoActual = millis();
  
  // Debounce del interruptor
  if (estadoActual != interruptorAnterior) {
    tiempoDebounce = tiempoActual;
  }
  
  if ((tiempoActual - tiempoDebounce) > DEBOUNCE_DELAY) {
    if (estadoActual != interruptorAnterior) {
      // Cambio de estado confirmado
      if (estadoActual == LOW) {
        // Interruptor en LOW - INICIAR SISTEMA
        if (!sistemaActivo) {
          iniciarSistema();
        }
      } else {
        // Interruptor en HIGH - DETENER SISTEMA
        if (sistemaActivo) {
          detenerSistema();
        }
      }
      interruptorAnterior = estadoActual;
    }
  }
}
// LED indicador de sistema activo - 3 parpadeos rápidos con sonido
void indicadorSistemaActivo() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    //buzzerVolumenBajo(500);  // 100ms con pulsos cortos
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}
// Función para iniciar el sistema completamente
void iniciarSistema() {
  // //ln(F("=== INICIANDO SISTEMA ==="));
  
  // Activar el sistema
  sistemaActivo = true;
  sistemaIniciado = true;
  
  // Reinicializar todas las variables del sistema
  estadoActual = ESTADO_NORMAL;
  // Medir distancia frontal al inicio
  distanciaFrontalInicial = medirDistanciaPromedio(TRIG_FRONTAL, ECHO_FRONTAL);
  Serial.print("Distancia inicial: ");
  Serial.println(distanciaFrontalInicial);

  prioridadActual = PRIORIDAD_NORMAL;
  emergenciaActivada = false;
  retrocesoPorEmergencia = false;
  esquiveDiagonalActivo = false;
  giroEnProceso = false;
  enGiro = false;
  cruceEnProceso = false;
  retrocesoOrientadoActivo = false;
  
  // Reinicializar variables de detección
  colorDetectado = 'N';
  colorObjetivo = 'N';
  ultimoColorDetectado = 'N';
  sentidoVehiculo = INDEFINIDO;
  primeraDeteccion = true;
  lineaNaranjaDetectada = false;
  
  // Reinicializar contadores y timers
  contadorLineas = 0;
  tiempoEsquive = 0;
  tiempoUltimaDeteccion = millis();
  ultimoTiempoCentrado = millis();
  lastTime = millis();
  
  // Reinicializar PIDs
  pidGiro.error_anterior = 0;
  pidGiro.integral = 0;
  pidGiro.tiempo_anterior = 0;
  
  pidCentrado.error_anterior = 0;
  pidCentrado.integral = 0;
  pidCentrado.tiempo_anterior = 0;
  pidCentrado.inicializado = false;
  
 // Centrar servo
  direccion.write(SERVO_CENTRADO);
  
  // Inicializar variables para el arranque retrasado del motor
  tiempoInicioSistema = millis();
  motorArrancado = false;
  
  // Motor permanece apagado hasta que pase el delay
  digitalWrite(MOTOR_PIN, HIGH);  // Motor apagado
  digitalWrite(MOTOR_RETROCESO_PIN, HIGH);  // Retroceso desactivado
  
  indicadorSistemaActivo();
  // //ln(F("Sistema iniciado - Motor arrancará en 1 segundo"));
}

// Función mejorada para detener el sistema completamente
void detenerSistema() {
  ////ln(F("=== DETENIENDO SISTEMA ==="));
  
  // Desactivar el sistema
  sistemaActivo = false;
  sistemaIniciado = false;
  
  // Detener todos los motores inmediatamente
  digitalWrite(MOTOR_PIN, HIGH);  // HIGH para detener según tu configuración
  digitalWrite(MOTOR_RETROCESO_PIN, HIGH);  // HIGH para detener retroceso
  
  // Centrar servo
  direccion.write(SERVO_CENTRADO);
  
  // Resetear sistema de retroceso y emergencias
  resetearRetrocesoEmergencia();
  retrocesoOrientadoActivo = false;
  emergenciaActivada = false;
  retrocesoPorEmergencia = false;
  esquiveDiagonalActivo = false;
  giroEnProceso = false;
  enGiro = false;
  cruceEnProceso = false;
  
  // Resetear estado a normal
  estadoActual = ESTADO_NORMAL;
  prioridadActual = PRIORIDAD_NORMAL;
  
  // Resetear variables de detección
  colorDetectado = 'N';
  colorObjetivo = 'N';
  ultimoColorDetectado = 'N';
  
  // LED parpadea rápidamente indicando sistema detenido, luego se apaga
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  digitalWrite(LED_PIN, LOW); // Asegurar que LED esté apagado
  
  ////ln(F("Sistema detenido completamente"));
}



float calcularPID(PIDController* pid, float setpoint, float medicion) {
  unsigned long tiempo_actual = millis();
  float dt = (tiempo_actual - pid->tiempo_anterior) / 1000.0;
  
  // Evitar divisiones por cero o deltas muy grandes
  if (dt <= 0 || dt > 0.1) {
    dt = 0.02; // 20ms por defecto
  }
  
  // Calcular error
  float error = setpoint - medicion;
  
  // Normalizar error angular (manejar cruce de 0°/360°)
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  
  // Término proporcional
  float proporcional = pid->kp * error;
  
  // Término integral con anti-windup
  pid->integral += error * dt;
  pid->integral = constrain(pid->integral, -pid->integral_max, pid->integral_max);
  float integral = pid->ki * pid->integral;
  
  // Término derivativo
  float derivativo = 0;
  if (pid->tiempo_anterior > 0) {
    derivativo = pid->kd * (error - pid->error_anterior) / dt;
  }
  
  // Salida PID total
  float salida = proporcional + integral + derivativo;
  
  // Guardar valores para próxima iteración
  pid->error_anterior = error;
  pid->tiempo_anterior = tiempo_actual;
  
  return salida;
}

void resetearPID(PIDController* pid) {
  pid->error_anterior = 0;
  pid->integral = 0;
  pid->tiempo_anterior = millis();
}

float calcularPIDCentrado(PIDCentrado* pid, float error) {
  unsigned long tiempo_actual = millis();
  
  // Inicializar en primera ejecución
  if (!pid->inicializado) {
    pid->tiempo_anterior = tiempo_actual;
    pid->error_anterior = error;
    pid->integral = 0;
    pid->inicializado = true;
    return 0; // No aplicar corrección en primera ejecución
  }
  
  float dt = (tiempo_actual - pid->tiempo_anterior) / 1000.0;
  
  // Evitar deltas muy grandes o muy pequeños
  if (dt <= 0 || dt > 0.5) {
    dt = 0.1; // 100ms por defecto
  }
  
  // Término proporcional
  float proporcional = pid->kp * error;
  
  // Término integral con anti-windup
  pid->integral += error * dt;
  pid->integral = constrain(pid->integral, -pid->integral_max, pid->integral_max);
  float integral = pid->ki * pid->integral;
  
  // Término derivativo
  float derivativo = pid->kd * (error - pid->error_anterior) / dt;
  
  // Salida PID total
  float salida = proporcional + integral + derivativo;
  
  // Guardar valores para próxima iteración
  pid->error_anterior = error;
  pid->tiempo_anterior = tiempo_actual;
  
  return salida;
}

void resetearPIDCentrado(PIDCentrado* pid) {
  pid->error_anterior = 0;
  pid->integral = 0;
  pid->tiempo_anterior = millis();
  pid->inicializado = false;
}

void ajustarParametrosPIDCentrado(float kp, float ki, float kd) {
  pidCentrado.kp = kp;
  pidCentrado.ki = ki;
  pidCentrado.kd = kd;
  resetearPIDCentrado(&pidCentrado);
  
 /* //(F("PID Centrado actualizado - Kp:"));
  //(kp);
  //(F(" Ki:"));
  //(ki);
  //(F(" Kd:"));
  //ln(kd);
  */
}


// ==================== FUNCIONES ORIGINALES ====================

// Función para confirmar color - MEJORADA
bool confirmarColor(int hueMin, int hueMax, float satMin, float valueMin, float valueMax, int lecturas, int confirmacionesRequeridas) {
  int detecciones = 0;
  
  for (int i = 0; i < lecturas; i++) {
    uint16_t r, g, b, c;
    double hue, saturation, value;
    
    tcs.getRawData(&r, &g, &b, &c);
    
    if (c > 0) {
      float r_norm = (float)r / c;
      float g_norm = (float)g / c;
      float b_norm = (float)b / c;
      
      uint8_t r_byte = (uint8_t)(r_norm * 255.0);
      uint8_t g_byte = (uint8_t)(g_norm * 255.0);
      uint8_t b_byte = (uint8_t)(b_norm * 255.0);
      
      ColorConverter::RgbToHsv(r_byte, g_byte, b_byte, hue, saturation, value);
      hue *= 360;
      
      if (hueMin >= 180) { // Es azul
        float proporcionAzul = b_norm / (r_norm + g_norm + 0.001);
        if (hue >= hueMin && hue <= hueMax && 
            saturation > satMin && 
            value > valueMin && value < valueMax &&
            proporcionAzul > 0.9) {
          detecciones++;
        }
      } 
      else { // Es naranja
        bool enRangoNormal = (hue >= hueMin && hue <= hueMax);
        bool enRangoAlto = (hueMax > 300 && hue >= 300) || (hueMin < 60 && hue <= 60);
        
        if ((enRangoNormal || enRangoAlto) && 
            saturation > satMin && 
            value > valueMin && value < valueMax) {
          detecciones++;
        }
      }
    }
    delay(5);
  }
  
  return (detecciones >= confirmacionesRequeridas);
}

// Función para medir distancia
float medirDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duracion = pulseIn(echoPin, HIGH, 30000);
  if (duracion == 0) return 999;
  
  return (duracion * 0.034) / 2;
}

// Función de emergencia para detectar obstáculos frontales

bool verificarEmergencia() {
  // Solo verificar si está en modo seguimiento y no hay emergencia reciente
  if (estadoActual != SIGUIENDO_OBJETO || 
      millis() - tiempoUltimaEmergencia < TIEMPO_ESPERA_EMERGENCIA) {
    return false;
  }
  
  // Medir distancia frontal
  float distanciaFrontal = medirDistancia(TRIG_FRONTAL, ECHO_FRONTAL);
  
  // Verificar si hay obstáculo en distancia de emergencia
  if (distanciaFrontal <= DISTANCIA_EMERGENCIA && distanciaFrontal > 0) {
    // Verificar si hay un color objetivo válido ACTUAL o RECORDADO
    bool colorActualValido = (colorObjetivo == 'R' || colorObjetivo == 'G');
   /* bool colorRecordadoValido = (ultimoColorDetectado == 'R' || ultimoColorDetectado == 'G') && 
                                (millis() - tiempoUltimoColor <= TIEMPO_MEMORIA_COLOR);*/
    
   // if (colorActualValido || colorRecordadoValido) { 
    if (colorActualValido) {
      emergenciaActivada = true;
      
      //  Usar color actual si está disponible, sino usar el recordado
      if (colorActualValido) {
        colorEmergencia = colorObjetivo;
      }/* else {
        colorEmergencia = ultimoColorDetectado;
        // Actualizar colorObjetivo para que el esquive funcione correctamente
        colorObjetivo = ultimoColorDetectado;
      }*/
      
      tiempoUltimaEmergencia = millis();
      
      // Debug opcional (mantener comentado en producción)
      /*
      //(F("EMERGENCIA: Obstáculo a "));
      //(distanciaFrontal);
      //(F("cm - Esquivando "));
      //(colorEmergencia);
      //(F(" ("));
      //(colorActualValido ? F("actual") : F("recordado"));
      //ln(F(")"));
      */
      
      return true;
    }
  }
  
  return false;
}

// Función para centrar el vehículo 
void centrarVehiculo() {
  //  No centrar si hay emergencias activas
  if (prioridadActual != PRIORIDAD_NORMAL || 
      esquiveDiagonalActivo || 
      retrocesoPorEmergencia) {
    // Resetear PID si hay emergencia para evitar acumulación
    if (pidCentrado.inicializado) {
      resetearPIDCentrado(&pidCentrado);
    }
    return;
  }
  
  // No centrar si está en giro, recién terminó giro, pausado por detección, o en esquive
  if (enGiro || 
      millis() - tiempoFinGiro < tiempoEsperaPosgiro ||
      (pausarCentrado && millis() - tiempoInicioDeteccion < tiempoPausaCentrado)) {
    
    // Resetear PID si no se puede ejecutar para evitar acumulación
    if (pidCentrado.inicializado) {
      resetearPIDCentrado(&pidCentrado);
    }
    return;
  }
  
  if (millis() - ultimoTiempoCentrado < intervaloCentrado) {
    return;
  }
  
  // Leer sensores con pequeña pausa para estabilidad
  float distanciaDerecha = medirDistancia(TRIG_DERECHO, ECHO_DERECHO);
  delay(5);
  float distanciaIzquierda = medirDistancia(TRIG_IZQUIERDO, ECHO_IZQUIERDO);
  
  // Debug (mantener comentado en producción)
  /*
  Serial.print(F("D:"));
  Serial.print(distanciaDerecha, 1);
  Serial.print(F(" I:"));
  Serial.print(distanciaIzquierda, 1);
  
  */
  
  // Validar lecturas de sensores
  if (distanciaDerecha > 80 || distanciaIzquierda > 80 || 
      distanciaDerecha < 1 || distanciaIzquierda < 1) {
    resetearPIDCentrado(&pidCentrado);
    return;
  }
  
  // Doble check de emergencias antes de mover servo
  if (prioridadActual != PRIORIDAD_NORMAL || 
      esquiveDiagonalActivo || 
      retrocesoPorEmergencia) {
    resetearPIDCentrado(&pidCentrado);
    return;
  }
  
  // Calcular error (positivo = más cerca de la derecha, negativo = más cerca de la izquierda)
  float diferencia = distanciaDerecha - distanciaIzquierda;
  
  // Debug del error
  /*
  //(F(" Error:"));
  //(diferencia, 1);
  */
  
  // Verificar si está dentro de tolerancia
  if (abs(diferencia) <= TOLERANCIA_CENTRADO_PID) {
    // Ya está centrado - mantener servo centrado y resetear integral
    direccion.write(SERVO_CENTRADO);
    pidCentrado.integral = 0; // Resetear integral para evitar acumulación
    pausarCentrado = false;
    
    /*
    //ln(F(" CENTRADO"));
    */
  } else {
    // Aplicar control PID
    float salidaPID = calcularPIDCentrado(&pidCentrado, diferencia);
    
    // Convertir salida PID a ángulo de servo
    // Limitar la salida PID para evitar movimientos extremos
    salidaPID = constrain(salidaPID, -ERROR_MAXIMO_CENTRADO, ERROR_MAXIMO_CENTRADO);
    
    // Calcular ángulo del servo
    // Factor de escalado para convertir PID a ángulo de servo
    float factorEscala = anguloCorreccionMax / ERROR_MAXIMO_CENTRADO;
    int correccionAngulo = salidaPID * factorEscala;
    
   //  Verificar si la suma de ambos sensores es menor a 50cm
    float sumaSensores = distanciaDerecha + distanciaIzquierda;
    if (sumaSensores < 50.0) {
      // Reducir grados de corrección cuando el espacio es muy estrecho
      correccionAngulo = correccionAngulo * 0.5; // Reducir a la mitad
    }
    // Aplicar suavizado para evitar cambios bruscos
    static int anguloAnterior = SERVO_CENTRADO;
    int anguloDeseado = SERVO_CENTRADO - correccionAngulo; // Invertir signo para corrección correcta
    anguloDeseado = constrain(anguloDeseado, SERVO_CENTRADO - anguloCorreccionMax, 
                              SERVO_CENTRADO + anguloCorreccionMax);
    
    // Aplicar suavizado
    int anguloFinal = anguloAnterior * (1 - FACTOR_SUAVIZADO) + anguloDeseado * FACTOR_SUAVIZADO;
    anguloAnterior = anguloFinal;
    
    // VERIFICACIÓN FINAL antes de mover servo
    if (prioridadActual == PRIORIDAD_NORMAL && 
        !esquiveDiagonalActivo && 
        !retrocesoPorEmergencia) {
      // Aplicar comando al servo SOLO si no hay emergencias
      direccion.write(anguloFinal);
    }
    
    // Debug de la corrección PID
    /*
    //(F(" PID:"));
    //(salidaPID, 1);
    //(F(" Servo:"));
    //(anguloFinal);
    //(diferencia > 0 ? F(" <-") : F(" ->"));
    //ln();
    */
  }
  
  ultimoTiempoCentrado = millis();
}
void detenerRobot() {
  digitalWrite(MOTOR_PIN, HIGH);
  direccion.write(SERVO_CENTRADO);
  enGiro = false;
  
  // Bucle infinito con LED y buzzer de pulsos cortos intermitentes
  while(1) {
    digitalWrite(LED_PIN, HIGH);
   // buzzerVolumenBajo(500);  // 500ms con pulsos cortos
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }


}
// ==================== FUNCIÓN realizarGiro MEJORADA CON PID ====================

//medir distancia con promedio
float medirDistanciaPromedio(int trigPin, int echoPin) {
  float suma = 0;
  int medicionesValidas = 0;
  
  for (int i = 0; i < 3; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duracion = pulseIn(echoPin, HIGH, 30000);
    if (duracion != 0) {
      suma += (duracion * 0.034) / 2;
      medicionesValidas++;
    }
    
    if (i < 2) delay(5); // Pequeño delay entre mediciones
  }
  
  if (medicionesValidas > 0) {
    return suma / medicionesValidas;
  } else {
    return 999; // Error en todas las mediciones
  }
}
unsigned long calcularTiempoRetroceso(float anguloUtilizado) {
  // Si está en modo de estacionamiento, no hay tiempo de retroceso
  if (giroParaEstacionamiento) {
    return 0;
  }
  
  // Mapeo más detallado de ángulos a tiempos de retroceso usando variables
  if (anguloUtilizado >= GIRO_MAXIMO) {
    return TIEMPO_RETROCESO_MAXIMO;     // 1200ms para giros máximos
  } else if (anguloUtilizado >= GIRO_ALTO) {
    return TIEMPO_RETROCESO_ALTO;       // 1000ms para giros altos
  } else if (anguloUtilizado >= GIRO_MEDIO_ALTO) {
    return TIEMPO_RETROCESO_MEDIO_ALTO; // 850ms para giros medio-altos
  } else if (anguloUtilizado >= GIRO_MEDIO) {
    return TIEMPO_RETROCESO_MEDIO;      // 700ms para giros medios
  } else if (anguloUtilizado >= GIRO_MEDIO_BAJO) {
    return TIEMPO_RETROCESO_MEDIO_BAJO; // 600ms para giros medio-bajos
  } else if (anguloUtilizado >= GIRO_BAJO) {
    return TIEMPO_RETROCESO_BAJO;       // 500ms para giros bajos
  } else if (anguloUtilizado >= GIRO_MINIMO) {
    return TIEMPO_RETROCESO_MINIMO;     // 400ms para giros mínimos
  } else {
    return TIEMPO_RETROCESO_DEFECTO;    // 700ms valor por defecto
  }
}
//calcular el ángulo dinámico
float calcularAnguloGiroDinamico(bool giroIzquierda) {
  float distanciaSensor;
  
  // Medir distancias justo antes del giro
  float distanciaDerecha = medirDistanciaPromedio(TRIG_DERECHO, ECHO_DERECHO);
  float distanciaIzquierda = medirDistanciaPromedio(TRIG_IZQUIERDO, ECHO_IZQUIERDO);
  
  // Seleccionar sensor según sentido del vehículo
  if (sentidoVehiculo == DERECHA) {
    distanciaSensor = distanciaIzquierda;
  } else if (sentidoVehiculo == IZQUIERDA) {
    distanciaSensor = distanciaDerecha;
  } else {
    // Fallback: usar ángulos fijos si no hay sentido definido
    float anguloFallback = giroIzquierda ? giroObjetivoAzul : giroObjetivoNaranja;
    return giroParaEstacionamiento ? anguloFallback + 5.0 : anguloFallback;
  }
  
  // Calcular ángulo basado en distancia con más subdivisiones
  float anguloCalculado;
  
  if (distanciaSensor < 25.0) {
    // Obstáculo muy cerca - giro máximo
    anguloCalculado = GIRO_MAXIMO;
  } else if (distanciaSensor >= 25.0 && distanciaSensor < 30.0) {
    // Obstáculo cerca - giro alto
    anguloCalculado = GIRO_ALTO;
  } else if (distanciaSensor >= 30.0 && distanciaSensor < 35.0) {
    // Obstáculo medio-cerca - giro medio-alto
    anguloCalculado = GIRO_MEDIO_ALTO;
  } else if (distanciaSensor >= 35.0 && distanciaSensor < 40.0) {
    // Obstáculo medio - giro medio
    anguloCalculado = GIRO_MEDIO;
  } else if (distanciaSensor >= 40.0 && distanciaSensor < 45.0) {
    // Obstáculo medio-lejos - giro medio-bajo
    anguloCalculado = GIRO_MEDIO_BAJO;
  } else if (distanciaSensor >= 45.0 && distanciaSensor < 55.0) {
    // Obstáculo lejos - giro bajo
    anguloCalculado = GIRO_BAJO;
  } else {
    // Obstáculo muy lejos o no detectado - giro mínimo
    anguloCalculado = GIRO_MINIMO;
  }
  
  // Si está en modo de estacionamiento, agregar 5 grados
  if (giroParaEstacionamiento) {
    anguloCalculado += 5.0;
  }
  
  // Debug opcional - descomenta para ver valores
  /*
  Serial.print("Distancia: ");
  Serial.print(distanciaSensor);
  Serial.print(" cm, Ángulo: ");
  Serial.print(anguloCalculado);
  Serial.println("°");
  */
  
  return anguloCalculado;
}
void realizarGiro(bool giroIzquierda) {
  enGiro = true;
  giroEnProceso = true;
  cruceEnProceso = true;
  interrupcionCruce = false;
  startTime = millis();
  
  // Calcular ángulo dinámico basado en sensores
  //float anguloObjetivo = calcularAnguloGiroDinamico(giroIzquierda);
  //unsigned long tiempoRetrocesoCalculado = calcularTiempoRetroceso(anguloObjetivo);
  
  // Resetear y configurar PID
  resetearPID(&pidGiro);
  actualizarGiro();
  float anguloInicial = anguloZ;
  //anguloObjetivoActual = anguloInicial + (giroIzquierda ? anguloObjetivo : -anguloObjetivo);
  anguloObjetivoActual = anguloObjetivoGlobal;
  // Normalizar ángulo objetivo
  while (anguloObjetivoActual >= 360) anguloObjetivoActual -= 360;
  while (anguloObjetivoActual < 0) anguloObjetivoActual += 360;
  
  // Activar motor para el giro
  digitalWrite(MOTOR_PIN, LOW);
  
  // Variables para control del bucle PID
  bool giroCompletado = false;
  int contadorEstable = 0;
  const int CONFIRMACIONES_ESTABLE = 5;
  bool giroInterrumpido = false;
  
  // Bucle principal de control PID
  while (millis() - startTime < timeoutGiro && !giroCompletado && !interrupcionCruce) {
    // Actualizar lectura del giroscopio
    actualizarGiro();
    
    // Calcular salida del controlador PID
    float salidaPID = calcularPID(&pidGiro, anguloObjetivoActual, anguloZ);
    
    // Convertir salida PID a ángulo de servo
    int anguloServo = SERVO_CENTRADO;
    
    if (abs(salidaPID) > 0.5) {
      float factorServo = constrain(abs(salidaPID) / 10.0, 0.2, 1.0);
      
      if (salidaPID > 0) {
        anguloServo = SERVO_CENTRADO + (factorServo * 90);
      } else {
        anguloServo = SERVO_CENTRADO - (factorServo * 90);
      }
      
      anguloServo = constrain(anguloServo, 0, 180);
    }
    
    direccion.write(anguloServo);
    
    // Calcular error actual para verificación de estabilidad
    float errorActual = anguloObjetivoActual - anguloZ;
    if (errorActual > 180) errorActual -= 360;
    if (errorActual < -180) errorActual += 360;
    
    // Verificar si el giro está completo
    if (abs(errorActual) <= TOLERANCIA_PID) {
      contadorEstable++;
      if (contadorEstable >= CONFIRMACIONES_ESTABLE) {
        giroCompletado = true;
      }
    } else {
      contadorEstable = 0;
    }
    
   /* // Verificar obstáculo diagonal cada 100ms
    static unsigned long lastObstacleCheck = 0;
    if (millis() - lastObstacleCheck > 100) {
      if (verificarObstaculoDiagonal()) {
        giroInterrumpido = true;
        break;
      }
      lastObstacleCheck = millis();
    }*/
    delay(20);
  }
  /*
  // Manejar interrupción por obstáculo
  if (giroInterrumpido) {
    enGiro = false;
    giroEnProceso = false;
    cruceEnProceso = false;
    return;
  }*/
  
  // Giro completado normalmente
  direccion.write(SERVO_CENTRADO);
  digitalWrite(MOTOR_PIN, LOW);
  
  // Resetear flags del giro (NO del retroceso)
  enGiro = false;
  giroEnProceso = false;
  cruceEnProceso = false;
  tiempoFinGiro = millis();
  
  unsigned long tiempoRetrocesoCalculado =0;  
  // Validación de tiempo de retroceso
  if (tiempoRetrocesoCalculado <= 0 || tiempoRetrocesoCalculado > TIMEOUT_RETROCESO_MAXIMO) {
    tiempoRetrocesoCalculado = 0;
  }
  
  // ACTIVAR RETROCESO POST-GIRO
  retrocesoOrientadoActivo = true;
  tiempoInicioRetrocesoOrientado = millis();
  direccionRetroceso = giroIzquierda ? SERVO_RIGHT : SERVO_LEFT;
  tiempoRetrocesoActual = tiempoRetrocesoCalculado;
  
  // IMPORTANTE: NO cambiar estadoActual aquí
  // Se cambiará automáticamente a ESTADO_NORMAL cuando termine el retroceso
}

void calcularAnguloObjetivoMantener() {
  if (sentidoVehiculo == IZQUIERDA) {
    // Giro a la izquierda: 1->90°, 2->180°, 3->270°, 4->0°, etc.
    int ciclo = (contadorLineas -1) % 4; // Ajuste para que la primera línea sea ciclo 0
    switch (ciclo) {
      case 0: anguloObjetivoGlobal = 90.0; break;  // Línea 1, 5, 9...
      case 1: anguloObjetivoGlobal = 180.0; break; // Línea 2, 6, 10...
      case 2: anguloObjetivoGlobal = 270.0; break; // Línea 3, 7, 11...
      case 3: anguloObjetivoGlobal = 0.0; break;   // Línea 4, 8, 12...
    }
  } else { // sentidoVehiculo == DERECHA
    // Giro a la derecha: 1->270°, 2->180°, 3->90°, 4->0°, etc.
    int ciclo = (contadorLineas - 1) % 4; // Ajuste para que la primera línea sea ciclo 0
    switch (ciclo) {
      case 0: anguloObjetivoGlobal = 270.0; break; // Línea 1, 5, 9...
      case 1: anguloObjetivoGlobal = 180.0; break; // Línea 2, 6, 10...
      case 2: anguloObjetivoGlobal = 90.0; break;  // Línea 3, 7, 11...
      case 3: anguloObjetivoGlobal = 0.0; break;   // Línea 4, 8, 12...
    }
  }
}
// ============== FUNCIÓN DE RETROCESO POST-GIRO ==============
bool procesarRetrocesoPostGiro() {
  // Si el retroceso no está activo, no hacer nada
  if (!retrocesoOrientadoActivo) {
    return false; // Indica que no hay retroceso activo
  }
  
  unsigned long tiempoTranscurrido = millis() - tiempoInicioRetrocesoOrientado;
  
  // Verificar condiciones de finalización
  bool tiempoCompleto = (tiempoTranscurrido >= tiempoRetrocesoActual);
  bool timeoutSeguridad = (tiempoTranscurrido >= TIMEOUT_RETROCESO_MAXIMO);
 
  bool emergenciaTimeout = (tiempoTranscurrido >= 4000); // 4 segundos absoluto
  
  // Si alguna condición se cumple, finalizar retroceso
  if (tiempoCompleto || timeoutSeguridad || emergenciaTimeout) {
    finalizarRetrocesoPostGiro();
    return false; // Retroceso completado
  }
  
  // Continuar con el retroceso
  direccion.write(direccionRetroceso);
  digitalWrite(MOTOR_RETROCESO_PIN, LOW);  // Activar retroceso
  digitalWrite(MOTOR_PIN, HIGH);           // Asegurar que avance esté desactivado
  
  return true; // Indica que el retroceso sigue activo
}
// ============== FUNCIÓN PARA FINALIZAR RETROCESO ==============
void finalizarRetrocesoPostGiro() {
  // Paso 1: Detener motores inmediatamente
  digitalWrite(MOTOR_RETROCESO_PIN, HIGH); // Desactivar retroceso
  digitalWrite(MOTOR_PIN, HIGH);           // Desactivar avance también
  
  // Paso 2: Centrar dirección
  direccion.write(SERVO_CENTER);
  
  // Paso 3: Pausa para estabilizar sistema
  delay(100);
  
  // Paso 4: Resetear flags de estado completamente
  retrocesoOrientadoActivo = false;
  enGiro = false;
  giroEnProceso = false;
  cruceEnProceso = false;
  
  // Paso 5: Resetear variables de tiempo
  tiempoInicioRetrocesoOrientado = 0;
  tiempoRetrocesoActual = 0;

  if (giroParaEstacionamiento) {
    estadoActual = ESTADO_DETENCION_FINAL;
    tiempoInicioDetencionFinal = millis();
    digitalWrite(MOTOR_RETROCESO_PIN, HIGH);
    digitalWrite(MOTOR_PIN, LOW);
    giroParaEstacionamiento = false;

    // === Ajuste de posición final para igualar distancia inicial ===
    float distanciaActual = medirDistanciaPromedio(TRIG_FRONTAL, ECHO_FRONTAL);
    float diferencia = distanciaFrontalInicial - distanciaActual;

    if (abs(diferencia) > TOLERANCIA_DISTANCIA_FINAL) {
      unsigned long tiempoAjuste = millis();
      const unsigned long tiempoMaximoAjuste = 3000;

      while (millis() - tiempoAjuste < tiempoMaximoAjuste) {
        float actual = medirDistanciaPromedio(TRIG_FRONTAL, ECHO_FRONTAL);
        float dif = distanciaFrontalInicial - actual;

        if (abs(dif) <= TOLERANCIA_DISTANCIA_FINAL) break;

        if (dif > 0) {
          // Está más lejos: avanzar
          digitalWrite(MOTOR_PIN, LOW);
          digitalWrite(MOTOR_RETROCESO_PIN, HIGH);
        } else {
          // Está más cerca: retroceder
          digitalWrite(MOTOR_PIN, HIGH);
          digitalWrite(MOTOR_RETROCESO_PIN, LOW);
        }

        delay(100);  // pequeño movimiento
        digitalWrite(MOTOR_PIN, HIGH);
        digitalWrite(MOTOR_RETROCESO_PIN, HIGH);
        delay(50);
      }
    }

  } else {
    if (contadorLineas < limiteLineas) {
      estadoActual = ESTADO_NORMAL;
    }
  }

  // Paso 6: ESTABLECER ESTADO NORMAL EXPLÍCITAMENTE

  // Paso 7: Resetear PID para estado limpio
  resetearPIDCentrado(&pidCentrado);
  
  // Paso 8: Reactivar avance normal si no es detención final
  if (estadoActual != ESTADO_DETENCION_FINAL) {
    digitalWrite(MOTOR_PIN, LOW);
  }
}


void manejarInterrupcionCruce() {
  // Detener el giro inmediatamente
  direccion.write(SERVO_CENTRADO);
  digitalWrite(MOTOR_PIN, LOW); // Mantener motor activo
  
  // Resetear flags de giro
  enGiro = false;
  giroEnProceso = false;
  cruceEnProceso = false;
  tiempoFinGiro = millis();
  
  // Procesar el comando que causó la interrupción
  colorDetectado = comandoInterrupcion;
  
  // Cambiar al estado apropiado según el comando
  if (comandoInterrupcion == 'R' || comandoInterrupcion == 'G') {
    colorObjetivo = comandoInterrupcion;
    estadoActual = SIGUIENDO_OBJETO;
   // //(F("Iniciando seguimiento de objeto: "));
    ////ln(comandoInterrupcion);
    
    // Orientar servo según la última orientación recibida
    orientarServoSeguimiento();
  }
  
  // Resetear flags de interrupción
  interrupcionCruce = false;
  comandoInterrupcion = 'N';
}

// ========== INICIALIZACIÓN ADICIONAL ==========
void inicializarSistemaHibrido() {
  // Inicializar buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    bufferMagnetometro[i] = 0;
  }
  
  Serial.println("Sistema híbrido MPU6050 + QMC5883L inicializado");
  Serial.println("Estrategias disponibles:");
  Serial.println("1: Corrección periódica simple");
  Serial.println("2: Filtro complementario continuo");
  Serial.println("3: Promedio móvil del magnetómetro");
  Serial.println("4: Corrección adaptativa");
}
// ========== FUNCIÓN PRINCIPAL ==========

void reiniciarQMC5883L() {
  // Software Reset
  Wire.beginTransmission(0x0D);
  Wire.write(0x0A);
  Wire.write(0x80);
  Wire.endTransmission();
  delay(10);
  
  // Set/Reset Period
  Wire.beginTransmission(0x0D);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(5);
  
  // Control 2 (sin interrupciones)
  Wire.beginTransmission(0x0D);
  Wire.write(0x0A);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(5);
  
  // Control 1 (modo continuo)
  Wire.beginTransmission(0x0D);
  Wire.write(0x09);
  Wire.write(0x1D);
  Wire.endTransmission();
  delay(50);
}
float obtenerAngulo() {
  static unsigned long ultimaVerificacion = 0;
  static unsigned long ultimaLecturaValida = 0;
  static int lecturasFallidas = 0;
  static float ultimoAnguloValido = 0;
  static float anguloAnterior = -999; // Para detectar valores repetidos
  static int valoresRepetidos = 0;
  static bool sensorInicializado = false;
  static int ultimoContadorLineas = 0;
  static float anguloReferencia = -999; // Ángulo inicial de referencia (nuestro "norte")
  static bool referenciaEstablecida = false;
  
  int x, y, z;
  unsigned long tiempoActual = millis();

  // INICIALIZACIÓN UNA SOLA VEZ
  if (!sensorInicializado) {
    reiniciarQMC5883L();
    sensorInicializado = true;
    ultimaLecturaValida = tiempoActual;
    //ultimoContadorLineas = contadorLineas;
    return 0;
  }

  // REINICIO PREVENTIVO CADA 4 LÍNEAS DETECTADAS
  /*if (contadorLineas - ultimoContadorLineas >= 4) {
    Serial.println("Reinicio preventivo - 4 líneas detectadas");
    reiniciarQMC5883L();
    ultimoContadorLineas = contadorLineas;
    lecturasFallidas = 0;
    valoresRepetidos = 0;
    anguloAnterior = -999;
    ultimaLecturaValida = tiempoActual;
    // NOTA: NO resetear anguloReferencia - mantener el norte original
  }
    */
 /* // VERIFICACIÓN PERIÓDICA (cada 3 segundos)
  if (tiempoActual - ultimaVerificacion > 3000) {
    if (tiempoActual - ultimaLecturaValida > 1500) { // 1.5 segundos sin lecturas válidas
      Serial.println("Sensor sin respuesta - Reiniciando");
      reiniciarQMC5883L();
      lecturasFallidas = 0;
      valoresRepetidos = 0;
      anguloAnterior = -999;
      ultimaLecturaValida = tiempoActual;
    }
    ultimaVerificacion = tiempoActual;
  }
*/
  // LECTURA RÁPIDA DEL STATUS
  Wire.beginTransmission(0x0D);
  Wire.write(0x06);
  if (Wire.endTransmission() != 0) {
    lecturasFallidas++;
    if (lecturasFallidas > 5) { // Reiniciar después de 5 fallos
      Serial.println("Múltiples fallos I2C - Reiniciando");
      reiniciarQMC5883L();
      lecturasFallidas = 0;
      valoresRepetidos = 0;
      anguloAnterior = -999;
    }
    return ultimoAnguloValido;
  }

  Wire.requestFrom(0x0D, 1);
  if (!Wire.available()) {
    lecturasFallidas++;
    return ultimoAnguloValido;
  }

  uint8_t status = Wire.read();
  
  // Si no hay datos nuevos, retornar valor anterior
  if ((status & 0x01) == 0) {
    return ultimoAnguloValido;
  }

  // LEER DATOS
  Wire.beginTransmission(0x0D);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    lecturasFallidas++;
    return ultimoAnguloValido;
  }

  Wire.requestFrom(0x0D, 6);
  if (Wire.available() != 6) {
    lecturasFallidas++;
    return ultimoAnguloValido;
  }

  x = Wire.read() | (Wire.read() << 8);
  y = Wire.read() | (Wire.read() << 8);
  z = Wire.read() | (Wire.read() << 8);

  if (x > 32767) x -= 65536;
  if (y > 32767) y -= 65536;

  // VERIFICACIÓN DE VALORES ANÓMALOS
  if (abs(x) > 30000 || abs(y) > 30000) {
    lecturasFallidas++;
    if (lecturasFallidas > 3) {
      Serial.println("Valores saturados - Reiniciando");
      reiniciarQMC5883L();
      lecturasFallidas = 0;
      valoresRepetidos = 0;
      anguloAnterior = -999;
    }
    return ultimoAnguloValido;
  }

  // CALCULAR ÁNGULO ABSOLUTO (con respecto al norte magnético real)
  float xCal = (x - xOffset) * xScale;
  float yCal = (y - yOffset) * yScale;

  float anguloAbsoluto = atan2(yCal, xCal) * 180.0 / PI;
  anguloAbsoluto *= -1;
  anguloAbsoluto += declinacionMagnetica;
  anguloAbsoluto -= offsetAngular;

  // Normalizar ángulo absoluto a 0-360
  while (anguloAbsoluto < 0) anguloAbsoluto += 360;
  while (anguloAbsoluto >= 360) anguloAbsoluto -= 360;

  // ESTABLECER REFERENCIA EN LA PRIMERA LECTURA VÁLIDA
  if (!referenciaEstablecida) {
    anguloReferencia = anguloAbsoluto;
    referenciaEstablecida = true;
    Serial.print("Referencia establecida en: ");
    Serial.print(anguloAbsoluto, 1);
    Serial.println("° (ahora será 0°)");
    return 0.0; // Primera lectura siempre es 0°
  }

  // CALCULAR ÁNGULO RELATIVO A LA REFERENCIA INICIAL
  float angulo = anguloAbsoluto - anguloReferencia;
  
  // Normalizar la diferencia a 0-360
  while (angulo < 0) angulo += 360;
  while (angulo >= 360) angulo -= 360;

  // DETECTAR VALORES REPETIDOS (sensor pegado)
  if (anguloAnterior != -999) {
    if (abs(angulo - anguloAnterior) < 0.1) { // Mismo valor (tolerancia 0.1°)
      valoresRepetidos++;
      if (valoresRepetidos >= 5) { // 5 valores idénticos = sensor pegado
        Serial.println("Valores repetidos detectados - Reiniciando");
        reiniciarQMC5883L();
        lecturasFallidas = 0;
        valoresRepetidos = 0;
        anguloAnterior = -999;
        ultimaLecturaValida = tiempoActual;
        return angulo;
      }
    } else {
      valoresRepetidos = 0; // Reset contador si el valor cambió
    }
  }
  anguloAnterior = angulo;

  // ACTUALIZAR CONTROL DE ESTADO
  ultimaLecturaValida = tiempoActual;
  lecturasFallidas = 0;
  ultimoAnguloValido = angulo;
  
  return angulo;
}
// ========== ESTRATEGIA 1: CORRECCIÓN PERIÓDICA SIMPLE ==========
void actualizarGiro() {
  // Ejecutar el código original del MPU6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();
  if (dt > 0.1) dt = 0.1;

  float velocidadAngularZ = (gz / 131.0) - offsetGz;
  
  if (abs(velocidadAngularZ) < umbralReposo) {
    velocidadAngularZ = 0;
  }

  if (abs(velocidadAngularZ) >= umbralGiro) {
    anguloZ += velocidadAngularZ * dt;
    while (anguloZ >= 360) anguloZ -= 360;
    while (anguloZ < 0) anguloZ += 360;
  }

  // CORRECCIÓN PERIÓDICA CON MAGNETÓMETRO
  if (millis() - ultimaCorreccionMag >= INTERVALO_MAGNETOMETRO) {
    anguloMagnetometro = obtenerAngulo();

    //Serial.print("angulo magne");
   // Serial.println(anguloMagnetometro);
    if (anguloMagnetometro >= 0) { // Lectura válida
      float diferencia = anguloMagnetometro - anguloZ;
      
      // Manejar la transición 0°-360°
      if (diferencia > 180) diferencia -= 360;
      if (diferencia < -180) diferencia += 360;
      
      // Solo corregir si la diferencia no es excesiva (filtro de outliers)
      if (abs(diferencia) > UMBRAL_DIFERENCIA_MAXIMA) {
        anguloZ += diferencia * 0.1; // Corrección suave del 10%
        
        // Normalizar
        while (anguloZ >= 360) anguloZ -= 360;
        while (anguloZ < 0) anguloZ += 360;
        
        Serial.print("Corrección aplicada: ");
        Serial.print(diferencia);
        Serial.println("°");
      }
    }
    ultimaCorreccionMag = millis();
  }

  // Continuar con pitch y roll como en el código original
  float accelPitch = atan2(ax, sqrt(ay*ay + az*az)) * 180 / PI;
  float accelRoll = atan2(ay, sqrt(ax*ax + az*az)) * 180 / PI;
  float gyroPitch = pitch + ((gy / 131.0) - offsetGy) * dt;
  float gyroRoll = roll + ((gx / 131.0) - offsetGx) * dt;
  pitch = K_GYRO * gyroPitch + K_ACC * accelPitch;
  roll = K_GYRO * gyroRoll + K_ACC * accelRoll;
}

void calibrarGyroAcel() {
  const int muestras = 500; // Número de muestras para promediar
  float sumaGz = 0.0;
  float sumaGx = 0.0;
  float sumaGy = 0.0;

  Serial.println("Calibrando giroscopio... Mantén el sensor inmóvil.");
  delay(1000); // Dar un segundo para estabilizar el sensor

  for (int i = 0; i < muestras; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Lee los datos crudos del MPU
    sumaGz += gz / 131.0; // Convierte la lectura cruda de Gz a grados por segundo (dps) y la suma
    sumaGx += gx / 131.0; // Convierte la lectura cruda de Gx a dps y la suma
    sumaGy += gy / 131.0; // Convierte la lectura cruda de Gy a dps y la suma
    delay(10); // Pequeño retardo para asegurar lecturas distintas y estabilización
  }

  // Calcula el promedio de las sumas para obtener los offsets
  offsetGz = sumaGz / muestras;
  offsetGx = sumaGx / muestras;
  offsetGy = sumaGy / muestras;

  Serial.print("Offset Gx: "); Serial.println(offsetGx);
  Serial.print("Offset Gy: "); Serial.println(offsetGy);
  Serial.print("Offset Gz: "); Serial.println(offsetGz);
  Serial.println("Calibración completa.");
}

void detectarColor() {
  uint16_t r, g, b, c;
  double hue, saturation, value;
  
  // Ignorar TODA detección durante retroceso post-giro
  if (retrocesoOrientadoActivo) {
    return; // Salir inmediatamente sin procesar nada
  }

  tcs.getRawData(&r, &g, &b, &c);

  if (c > 0) {
    float r_norm = (float)r / c;
    float g_norm = (float)g / c;
    float b_norm = (float)b / c;

    uint8_t r_byte = (uint8_t)(r_norm * 255.0);
    uint8_t g_byte = (uint8_t)(g_norm * 255.0);
    uint8_t b_byte = (uint8_t)(b_norm * 255.0);

    ColorConverter::RgbToHsv(r_byte, g_byte, b_byte, hue, saturation, value);
    hue *= 360;

    if (millis() - tiempoUltimaDeteccion > tiempoEsperaDeteccion) {
      bool esBlanco = (value > 0.85 && saturation < 0.15);
      float proporcionAzul = b_norm / (r_norm + g_norm + 0.001);
      
      bool azulDetectado = false;
      bool naranjaDetectado = false;
        
      // Detección azul
     if (hue >= 200 && hue <= 235 &&
          saturation > 0.70 &&
          value > 0.45 && value < 0.82 &&
          proporcionAzul > 1.66 &&
          !esBlanco) {
          
        if (confirmarColor(200, 235, 0.70, 0.45, 0.82, 5, 1)) {
          
          azulDetectado = true;
          
          pausarCentrado = true;
          tiempoInicioDeteccion = millis();
          
          if (primeraDeteccion) {
            sentidoVehiculo = IZQUIERDA;
            primeraDeteccion = false;
          }
        }
      }
       // Detección naranja/rojo/amarillo
      else if (((hue >= 3 && hue <= 45)) &&
               saturation > 0.40 &&
               value > 0.34 && value < 0.72 &&
               !esBlanco) {
               
        if (confirmarColor(3, 45, 0.40, 0.34, 0.72, 5, 1)) {
          naranjaDetectado = true;
          
          pausarCentrado = true;
          tiempoInicioDeteccion = millis();
          
          if (primeraDeteccion) {
            sentidoVehiculo = DERECHA;
            primeraDeteccion = false;
          }
        }
      }

      // Al detectar línea, verificar si puede interrumpir esquive
      if (azulDetectado || naranjaDetectado) {
        
        // INTERRUMPIR ESQUIVE DIAGONAL si está activo
        if (esquiveDiagonalActivo) {
          resetearEsquiveDiagonal();
          resetearPIDCentrado(&pidCentrado);
          estadoActual = ESTADO_NORMAL;
        }
        // INTERRUMPIR ESQUIVE si está en etapa de retorno
        if (estadoActual == ESQUIVANDO && (etapaEsquive == 3 || etapaEsquive == 4)) {
          estadoActual = ESTADO_NORMAL;
          etapaEsquive = 0;
        }
        
        // Ejecutar giro normal según el sentido del vehículo
        if (azulDetectado && sentidoVehiculo == IZQUIERDA) {
          digitalWrite(LED_PIN, HIGH);
          contadorLineas++;
          //("linea contada   ");
            //ln(contadorLineas);
          // *** MODIFICACIÓN PRINCIPAL ***
          /*if (contadorLineas >= limiteLineas) {
            //ln("limite de lineas alcanzaado");

            resetearPIDCentrado(&pidCentrado);
            
            realizarGiro(true); // Giro a la izquierda
            // REALIZAR EL GIRO PRIMERO antes de buscar estacionamiento
            // Marcar que después del giro debe iniciar búsqueda de estacionamiento
            // Esto se manejará en la función actualizarGiro() cuando termine el giro
            giroParaEstacionamiento = true;
            
          }*/
          if (contadorLineas >= limiteLineas) {
            //ln("Limite de lineas alcanzado, iniciando detencion en 3s...");
            estadoActual = ESTADO_DETENCION_FINAL;
            giroParaEstacionamiento = true;
            resetearPIDCentrado(&pidCentrado);
            calcularAnguloObjetivoMantener(); // calculo de angulo dependiendo de contadorLineas
            realizarGiro(true); // Giro a la izquierda
            tiempoInicioDetencionFinal = millis(); // Inicia el temporizador de 3 segundos
          }else {
            resetearPIDCentrado(&pidCentrado);
            calcularAnguloObjetivoMantener(); // calculo de angulo dependiendo de contadorLineas
            realizarGiro(true); // Giro a la izquierda
            
            // Después del giro, gestionar el estado apropiado
            if (estadoActual == ESQUIVANDO) {
              estadoActual = ESQUIVANDO;
            } else if (estadoActual == SIGUIENDO_OBJETO) {
              estadoActual = SIGUIENDO_OBJETO;
            }
          }
          
          digitalWrite(LED_PIN, LOW);
          tiempoUltimaDeteccion = millis();
        }
        else if (naranjaDetectado && sentidoVehiculo == DERECHA) {
          digitalWrite(LED_PIN, HIGH);
          contadorLineas++;
          //("linea contada   ");
            //ln(contadorLineas);
          // *** MODIFICACIÓN PRINCIPAL ***
          /*if (contadorLineas >= limiteLineas) {
            //ln("limite de lineas alcanzaado");
            // REALIZAR EL GIRO PRIMERO antes de buscar estacionamiento
            // Marcar que después del giro debe iniciar búsqueda de estacionamiento
            // Esto se manejará en la función actualizarGiro() cuando termine el giro
            giroParaEstacionamiento = true;
            resetearPIDCentrado(&pidCentrado);
           
            realizarGiro(false); // Giro a la derecha
            
            
          }*/
           if (contadorLineas >= limiteLineas) {
            //ln("Limite de lineas alcanzado, iniciando detencion en 3s...");
            giroParaEstacionamiento = true;
            resetearPIDCentrado(&pidCentrado);
            calcularAnguloObjetivoMantener(); // calculo de angulo dependiendo de contadorLineas
            realizarGiro(false); // Giro a la derecha
            estadoActual = ESTADO_DETENCION_FINAL;
            tiempoInicioDetencionFinal = millis(); // Inicia el temporizador de 3 segundos
          }else {
            resetearPIDCentrado(&pidCentrado);
            calcularAnguloObjetivoMantener(); // calculo de angulo dependiendo de contadorLineas
            realizarGiro(false); // Giro a la derecha
            
            // Después del giro, gestionar el estado apropiado
            if (estadoActual == ESQUIVANDO) {
              estadoActual = ESQUIVANDO;
            } else if (estadoActual == SIGUIENDO_OBJETO) {
              estadoActual = SIGUIENDO_OBJETO;
            }
          }
          
          digitalWrite(LED_PIN, LOW);
          tiempoUltimaDeteccion = millis();
        }
        else if (naranjaDetectado && sentidoVehiculo == IZQUIERDA) {
          // Línea naranja detectada pero el vehículo va hacia la izquierda
          // Parpadeo de advertencia
          for (int i = 0; i < 2; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
          }
        }
      }
    }
  }
}


//-----------------Funciones de evasion y recepcion de serial---------------
//PASO 3: Integrar funciones de recepción serial optimizada
void processSerialData() {
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      if (bufferIndex > 0) {
        buffer[bufferIndex] = '\0';
        parseCommand();
        bufferIndex = 0;
      }
      return;
    }
    
    if (bufferIndex < BUFFER_SIZE - 1) {
      buffer[bufferIndex++] = inChar;
    } else {
      bufferIndex = 0;
    }
  }
}

void parseCommand() {
  //  Ignorar comandos durante AVANCE FINAL
  /*if (estadoActual == ESTADO_AVANCE_FINAL) {
    // Durante el avance final, ignorar todos los comandos de detección
    // Solo permitir comandos de emergencia críticos si es necesario
    return;
  }*/
  
  // Comando de no detección
  if (buffer[0] == 'N' && bufferIndex == 1) {
    handleNoDetection();
    return;
  }
  
  // Parsing formato: R,25,I o G,30,D
  int firstComma = -1;
  int secondComma = -1;
  
  for (int i = 0; i < bufferIndex; i++) {
    if (buffer[i] == ',') {
      if (firstComma == -1) {
        firstComma = i;
      } else if (secondComma == -1) {
        secondComma = i;
        break;
      }
    }
  }
  
  if (firstComma == -1 || secondComma == -1) return;
  
  // Extraer datos
  colorDetectado = buffer[0];
  
  distanciaObjeto = 0;
  for (int i = firstComma + 1; i < secondComma; i++) {
    if (buffer[i] >= '0' && buffer[i] <= '9') {
      distanciaObjeto = distanciaObjeto * 10 + (buffer[i] - '0');
    }
  }
  
  orientacionObjeto = buffer[secondComma + 1];
  
  // Guardar último color válido detectado
  if (colorDetectado == 'R' || colorDetectado == 'G') {
    ultimoColorDetectado = colorDetectado;
    tiempoUltimoColor = millis();
  }
  
  // Verificar interrupción de cruce
  if (cruceEnProceso && (colorDetectado == 'R' || colorDetectado == 'G')) {
    interrupcionCruce = true;
    comandoInterrupcion = colorDetectado;
    return; // Salir inmediatamente para procesar la interrupción
  }
  
  processCommand();
}
// PASO 4: Máquina de estados principal
void processCommand() {
  switch(estadoActual) {
    case ESTADO_NORMAL:
      // Cambiar a seguimiento si detecta R o G
      if (colorDetectado == 'R' || colorDetectado == 'G') {
        colorObjetivo = colorDetectado;
        estadoActual = SIGUIENDO_OBJETO;
        // Mantener motor activo, servo se orienta en loop principal
      }
      break;
      
    case SIGUIENDO_OBJETO:{
      // Actualizar orientación del servo
      orientarServoSeguimiento();
      
      // NUEVA FUNCIONALIDAD: Verificar emergencia por sensor frontal
      if (verificarEmergencia()) {
        // Activar esquive de emergencia
        estadoActual = ESQUIVANDO;
        etapaEsquive = 0;
        tiempoEsquive = millis();
        colorObjetivo = colorEmergencia; // Usar el color detectado en emergencia
        
        // Debug opcional
        /*
        //ln(F("ESQUIVE DE EMERGENCIA ACTIVADO"));
        */
        break;
      
      }
      
      // Verificar si debe esquivar por comando normal (distancia específica por color)
      int distanciaEsquive = 0;
      if (colorDetectado == 'R') {
        distanciaEsquive = DISTANCIA_ROJO;
      } else if (colorDetectado == 'G') {
        distanciaEsquive = DISTANCIA_VERDE;
      }
      
      if (distanciaObjeto <= distanciaEsquive && distanciaObjeto > 0 && 
          (colorDetectado == 'R' || colorDetectado == 'G')) {
        colorObjetivo = colorDetectado;
        estadoActual = ESQUIVANDO;
        etapaEsquive = 0;
        tiempoEsquive = millis();
      }
      break;
    }
    case ESQUIVANDO:
      // Durante retorno (etapas 3-4), puede interrumpirse
      if ((etapaEsquive == 3 || etapaEsquive == 4) && 
          (colorDetectado == 'R' || colorDetectado == 'G')) {
        colorObjetivo = colorDetectado;
        estadoActual = SIGUIENDO_OBJETO;
        etapaEsquive = 0;
        orientarServoSeguimiento();
      }
      break;
      
   
  }
}
void handleNoDetection() {
  //  No procesar durante AVANCE FINAL
  /*if (estadoActual == ESTADO_AVANCE_FINAL) {
    return; // Ignorar completamente durante avance final
  }*/
  
  // Si estabas siguiendo un objeto y ya no lo ves...
  if (estadoActual == SIGUIENDO_OBJETO) {
    // Centra el servo
    if (lastOrientation != 'C') {
      direccion.write(SERVO_CENTRADO);
      lastOrientation = 'C';
    }
    
    // Solo regresar a estado normal si ha pasado suficiente tiempo
    // desde la última detección válida (para evitar cambios bruscos)
    unsigned long tiempoSinDeteccion = millis() - tiempoUltimoColor;
    
    if (tiempoSinDeteccion > TIEMPO_MEMORIA_COLOR) {
      // Ha pasado mucho tiempo, regresar al estado normal
      estadoActual = ESTADO_NORMAL;
      resetearPIDCentrado(&pidCentrado);
      ultimoColorDetectado = 'N'; // Limpiar memoria
    }
    // Si no ha pasado suficiente tiempo, mantener el estado de seguimiento
    // para permitir que la emergencia funcione con el color recordado
  }
}

//PASO 5: Funciones de orientación del servo
void orientarServoSeguimiento() {
  if (orientacionObjeto != lastOrientation) {
    int servoPosition = SERVO_CENTRADO;
    
    switch(orientacionObjeto) {
      case 'I':
        servoPosition = SERVO_OrientadoObjeto_LEFT; // Izquierda moderada para seguimiento
        break;
      case 'D':
        servoPosition = SERVO_OrientadoObjeto_RIGH; // Derecha moderada para seguimiento
        break;
      case 'C':
        servoPosition = SERVO_CENTRADO;
        break;
    }
    
    direccion.write(servoPosition);
    lastOrientation = orientacionObjeto;
  }
}

void orientarServoEsquive() {
  if (orientacionObjeto != lastOrientation) {
    int servoPosition = SERVO_CENTRADO;
    
    switch(orientacionObjeto) {
      case 'I':
        servoPosition = SERVO_LEFT; // Izquierda extrema para esquive
        break;
      case 'D':
        servoPosition = SERVO_RIGHT; // Derecha extrema para esquive
        break;
      case 'C':
        servoPosition = SERVO_CENTRADO;
        break;
    }
    
    direccion.write(servoPosition);
    lastOrientation = orientacionObjeto;
  }
}
void configurarTiemposEsquive() {
  if (colorObjetivo == 'G') {
    tiempoEsquiveActual = TIEMPO_ESQUIVE_VERDE;
    tiempoRetornoActual = TIEMPO_RETORNO_VERDE;
  } else if (colorObjetivo == 'R') {
    tiempoEsquiveActual = TIEMPO_ESQUIVE_ROJO;
    tiempoRetornoActual = TIEMPO_RETORNO_ROJO;
  } else {
    // Valores por defecto si hay otro color
    tiempoEsquiveActual = 800;
    tiempoRetornoActual = 600;
  }
}
//PASO 6: Función de esquive completa
void procesarEsquive() {
  unsigned long tiempoActual = millis();
  
  switch(etapaEsquive) {
   case 0: // Orientar servo hacia lado de esquive
      if (colorObjetivo == 'G') {
        direccion.write(SERVO_LEFT); // Esquivar izquierda extrema
        lastOrientation = 'I';
      } else if (colorObjetivo == 'R') {
        direccion.write(SERVO_RIGHT); // Esquivar derecha extrema
        lastOrientation = 'D';
      }
          configurarTiemposEsquive();
      
      tiempoEsquive = tiempoActual;
      etapaEsquive = 1;
      break;
        if (colorObjetivo == 'G') {
          direccion.write(SERVO_LEFT); // Esquivar izquierda extrema
          lastOrientation = 'I';
        } else if (colorObjetivo == 'R') {
          direccion.write(SERVO_RIGHT); // Esquivar derecha extrema
          lastOrientation = 'D';
        }
        tiempoEsquive = tiempoActual;
        etapaEsquive = 1;
        break;
      
    case 1: // Esperar orientación y activar motor
      if (tiempoActual - tiempoEsquive > 300) {
        // Motor ya está activo en estado normal
        etapaEsquive = 2;
        tiempoEsquive = tiempoActual;
      }
      break;
      
    case 2: // Avanzar esquivando
      if (tiempoActual - tiempoEsquive > tiempoEsquiveActual) {
        // Girar al lado contrario para retornar
        if (colorObjetivo == 'G') {
          direccion.write(SERVO_RIGHT); // Retornar por derecha
          lastOrientation = 'D';
        } else if (colorObjetivo == 'R') {
          direccion.write(SERVO_LEFT); // Retornar por izquierda
          lastOrientation = 'I';
        }
        etapaEsquive = 3;
        tiempoEsquive = tiempoActual;
      }
      break;
      
    case 3: // Esperar orientación de retorno
      if (tiempoActual - tiempoEsquive > 300) {
        etapaEsquive = 4;
        tiempoEsquive = tiempoActual;
      }
      break;
      
    case 4: // Avanzar de retorno (INTERRUMPIBLE)
      // CAMBIAR ESTA LÍNEA:
      if (tiempoActual - tiempoEsquive > tiempoRetornoActual) {
        etapaEsquive = 5;
        tiempoEsquive = tiempoActual;
      }
      break;
      
    case 5: // Centrar servo y volver a estado normal
      if (tiempoActual - tiempoEsquive > 100) {
        direccion.write(SERVO_CENTRADO);
        lastOrientation = 'C';

        //prueba cambio de lo de abajo
          estadoActual = ESTADO_NORMAL;

        // Si fue una emergencia, volver a seguimiento del mismo color
        /*if (emergenciaActivada) {
          estadoActual = SIGUIENDO_OBJETO;
          // Mantener el mismo colorObjetivo
          emergenciaActivada = false;
        } else {
          estadoActual = ESTADO_NORMAL;
        }*/
        
        etapaEsquive = 0;
      }
      break;
  }
}

// ==================== FUNCIÓN LEER SENSOR SHARP ====================

float leerSensorSharp(int pin) {
  // Leer valor analógico del sensor Sharp
  int valorAnalogico = analogRead(pin);
  
  // Convertir a voltaje (asumiendo 5V de referencia)
  float voltaje = valorAnalogico * (3.3 / 4095);
  
  // Conversión aproximada para sensor Sharp GP2Y0A21YK0F (10-80cm)
  // Fórmula: distancia = 27.728 / (voltaje - 0.1696)
  if (voltaje < 0.4) return 999; // Fuera de rango
  
  float distancia = 27.728 / (voltaje - 0.1696);
  
  // Limitar rango válido, el sensor sharp agarra minimo 10cm
  if (distancia < 10 || distancia > 80) return 999;
  
  return distancia;
}

// ==================== FUNCIÓN VERIFICAR OBSTÁCULO DIAGONAL ====================

bool verificarObstaculoDiagonal() {
    if (estadoActual == ESQUIVANDO) {
    return false; // Desactivar detección en ESQUIVANDO
  }
  // No verificar si ya está en esquive diagonal o retroceso, o muy reciente
  if (esquiveDiagonalActivo || 
      prioridadActual == PRIORIDAD_RETROCESO ||
      millis() - tiempoUltimaDeteccionSharp < TIEMPO_ESPERA_SHARP) {
    return false;
  }
  
  // Leer ambos sensores Sharp
  float distanciaIzq = leerSensorSharp(SHARP_DIAGONAL_IZQ_PIN);
  float distanciaDer = leerSensorSharp(SHARP_DIAGONAL_DER_PIN);

  // Verificar obstáculo en diagonal izquierda
  if (distanciaIzq <= DISTANCIA_SHARP_EMERGENCIA && distanciaIzq > 0) {
    esquiveDiagonalActivo = true;
    direccionEsquiveDiagonal = 'D'; // Esquivar hacia la derecha
    tiempoInicioEsquiveDiagonal = millis();
    tiempoUltimaDeteccionSharp = millis();
    etapaEsquiveDiagonal = 0;
    return true;
  }
  
  // Verificar obstáculo en diagonal derecha
  if (distanciaDer <= DISTANCIA_SHARP_EMERGENCIA && distanciaDer > 0) {
    esquiveDiagonalActivo = true;
    direccionEsquiveDiagonal = 'I'; // Esquivar hacia la izquierda
    tiempoInicioEsquiveDiagonal = millis();
    tiempoUltimaDeteccionSharp = millis();
    etapaEsquiveDiagonal = 0;
    return true;
  }
  
  return false;
}

// ==================== FUNCIÓN PROCESAR ESQUIVE DIAGONAL ====================

void procesarEsquiveDiagonal() {
   if (estadoActual == ESQUIVANDO || !esquiveDiagonalActivo) {
   
    return;
  }
  // Verificar si ha sido interrumpido por una línea
  if (interrupcionCruce) {
    resetearEsquiveDiagonal();
    return;
  }
  if (!esquiveDiagonalActivo) {
    return;
  }
  
  unsigned long tiempoTranscurrido = millis() - tiempoInicioEsquiveDiagonal;
  
  switch(etapaEsquiveDiagonal) {
    case 0: // Orientar servo para cruce diagonal
      if (direccionEsquiveDiagonal == 'D') {
        direccion.write(SERVO_RIGHT); // Cruzar hacia la derecha
      } else if (direccionEsquiveDiagonal == 'I') {
        direccion.write(SERVO_LEFT); // Cruzar hacia la izquierda
      }
      
      // Asegurar que el motor esté activo
      digitalWrite(MOTOR_PIN, LOW);
      
      etapaEsquiveDiagonal = 1;
      tiempoInicioEsquiveDiagonal = millis();
      break;
      
    case 1: // Ejecutar cruce diagonal
      if (tiempoTranscurrido > TIEMPO_CRUCE_DIAGONAL) {
        etapaEsquiveDiagonal = 2;
        tiempoInicioEsquiveDiagonal = millis();
      }
      break;
      
    case 2: // Centrar servo y finalizar
      direccion.write(SERVO_CENTRADO);
      
      // Finalizar esquive diagonal
      esquiveDiagonalActivo = false;
      direccionEsquiveDiagonal = 'N';
      etapaEsquiveDiagonal = 0;
      
      // Resetear PID de centrado para retomar navegación normal
      // (Si tienes sistema PID de centrado)
      // resetearPIDCentrado(&pidCentrado);
      break;
  }
}
void resetearEsquiveDiagonal() {
  //direccion.write(SERVO_CENTRADO);
  esquiveDiagonalActivo = false;
  direccionEsquiveDiagonal = 'N';
  etapaEsquiveDiagonal = 0;
  prioridadActual = PRIORIDAD_NORMAL;
  estadoActual = ESTADO_NORMAL;
  //Reactivar la navegación normal ---
  enGiro = false;
  giroEnProceso = false;
  cruceEnProceso = false;
}

bool verificarRetrocesoEmergencia() {
  // NO verificar si ya está en retroceso o si no ha pasado suficiente tiempo
  if (retrocesoPorEmergencia || 
      millis() - tiempoUltimaVerificacionRetroceso < INTERVALO_VERIFICACION_RETROCESO) {
    return false;
  }
  
  tiempoUltimaVerificacionRetroceso = millis();
  
  // Medir distancia frontal
  float distanciaFrontal = medirDistancia(TRIG_FRONTAL, ECHO_FRONTAL);
  
  // Verificar si hay obstáculo muy cerca y no está ya en retroceso
  if (distanciaFrontal <= DISTANCIA_RETROCESO_EMERGENCIA && 
    distanciaFrontal > 0 && distanciaFrontal < 200) {
    
    // Activar retroceso de emergencia
    retrocesoPorEmergencia = true;
    motorRetrocesoConfigurado = false;  // RESETEAR FLAG AQUÍ
    tiempoInicioRetroceso = millis();
    prioridadActual = PRIORIDAD_RETROCESO;  // MÁXIMA PRIORIDAD
    
    ////ln("ACTIVANDO RETROCESO DE EMERGENCIA");
    
    return true;
  }
  
  return false;
}
void procesarRetrocesoEmergencia() {
  if (!retrocesoPorEmergencia) {
    return;
  }

  unsigned long tiempoTranscurrido = millis() - tiempoInicioRetroceso;
  
  // Verificar timeout de seguridad
  if (tiempoTranscurrido > TIMEOUT_RETROCESO) {
    ////ln("TIMEOUT DE RETROCESO - FINALIZANDO FORZADAMENTE");
    
    // Timeout alcanzado - forzar finalización
    digitalWrite(MOTOR_RETROCESO_PIN, HIGH); // Apagar motor de retroceso
    delay(100);
    digitalWrite(MOTOR_PIN, LOW);            // Reactivar motor principal
    digitalWrite(LED_PIN, LOW);              // Apagar LED
    
    // RESETEAR TODAS LAS VARIABLES
    retrocesoPorEmergencia = false;
    motorRetrocesoConfigurado = false;
    prioridadActual = PRIORIDAD_NORMAL;
    estadoActual = ESTADO_NORMAL;
    
    delay(TIEMPO_ESPERA_POST_RETROCESO);
    return;
  }
  
  if (tiempoTranscurrido < TIEMPO_RETROCESO) {
    // FASE DE RETROCESO ACTIVO
    
    // Configurar motores solo una vez por ciclo
    if (!motorRetrocesoConfigurado) {
     // //ln("Configurando motores para retroceso...");
      
      digitalWrite(MOTOR_PIN, HIGH);           // APAGAR motor principal COMPLETAMENTE
      delay(150);                              // Pausa para asegurar parada
      digitalWrite(MOTOR_RETROCESO_PIN, LOW);  // ACTIVAR motor de retroceso
      direccion.write(SERVO_CENTRADO);         // Centrar servo para retroceso recto
      
      motorRetrocesoConfigurado = true;
     // //ln("Motores configurados - Retrocediendo...");
    }
    
    // Parpadeo rápido del LED para indicar retroceso
    if ((tiempoTranscurrido / 150) % 2 == 0) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
    
  } else {
    // FASE DE FINALIZACIÓN
    ////ln("Finalizando retroceso...");
    
    digitalWrite(MOTOR_RETROCESO_PIN, HIGH); // Apagar motor de retroceso PRIMERO
    delay(200);                              // Pausa para que se detenga completamente
    
    // Verificar distancia SOLO al final
    float distanciaFinal = medirDistancia(TRIG_FRONTAL, ECHO_FRONTAL);
    ////("Distancia final tras retroceso: ");
    ////ln(distanciaFinal);
    
    if (distanciaFinal > DISTANCIA_RETROCESO_EMERGENCIA + 5 || distanciaFinal > 100) {
      // Hay suficiente espacio O sensor fuera de rango (espacio libre)
      digitalWrite(MOTOR_PIN, LOW);            // Reactivar motor principal
      digitalWrite(LED_PIN, LOW);              // Apagar LED
      // Se resetea el búfer para ignorar comandos recibidos durante la emergencia.
      bufferIndex = 0;
      buffer[0] = '\0';
      // RESETEAR TODAS LAS VARIABLES CORRECTAMENTE
      retrocesoPorEmergencia = false;
      motorRetrocesoConfigurado = false;  // IMPORTANTE: Resetear para próxima vez
      prioridadActual = PRIORIDAD_NORMAL;
      estadoActual = ESTADO_NORMAL;
      
      ////ln("Retroceso completado exitosamente - Reanudando marcha normal");
      delay(TIEMPO_ESPERA_POST_RETROCESO);
      
    } else {
      // AÚN hay obstáculo - reiniciar ciclo de retroceso
      ////ln("Obstáculo aún presente - Reiniciando ciclo de retroceso");
      
      // RESETEAR PARA NUEVO CICLO
      motorRetrocesoConfigurado = false;  // PERMITIR NUEVA CONFIGURACIÓN
      tiempoInicioRetroceso = millis();   // NUEVO TIEMPO DE INICIO
      
      // Mantener retrocesoPorEmergencia = true para continuar
    }
  }
}

void resetearRetrocesoEmergencia() {
  ////ln("RESET COMPLETO DEL SISTEMA DE RETROCESO");
  
  // Apagar motores de forma segura
  digitalWrite(MOTOR_RETROCESO_PIN, HIGH); // Apagar motor de retroceso
  delay(100);
  digitalWrite(MOTOR_PIN, HIGH);           // Apagar motor principal temporalmente
  delay(100);
  digitalWrite(LED_PIN, LOW);              // Apagar LED
  
  // Resetear TODAS las variables de estado
  retrocesoPorEmergencia = false;
  motorRetrocesoConfigurado = false;  // IMPORTANTE: Resetear flag
  prioridadActual = PRIORIDAD_NORMAL;

  estadoActual = ESTADO_NORMAL;

  tiempoInicioRetroceso = 0;
  tiempoUltimaVerificacionRetroceso = 0;
  
  // Centrar servo
  direccion.write(SERVO_CENTRADO);
  delay(100);
  
  // Reactivar motor principal
  digitalWrite(MOTOR_PIN, LOW);
  
  ////ln("Reset completo finalizado");
}

// Función adicional para verificar si el sistema puede ejecutar operaciones
bool sistemaListo() {
  return sistemaActivo && sistemaIniciado;
}

// Modificación para tu función setup() - agregar al final
void configurarInterruptor() {
  pinMode(INTERRUPTOR_PIN, INPUT_PULLUP);  // Configurar con pull-up interno
  sistemaActivo = false;  // Sistema inicia desactivado
  sistemaIniciado = false;
  
  // Si el interruptor ya está presionado al encender, iniciar automáticamente
  if (digitalRead(INTERRUPTOR_PIN) == LOW) {
    delay(100);  // Pequeña pausa para estabilizar la lecturaa
    if (digitalRead(INTERRUPTOR_PIN) == LOW) {
      iniciarSistema();
      //delay(1500);
    }
  } else {
    esperarInterruptor();  // Si no está presionado, esperar
  }
}
//----------------- FIN Funciones de evasion y recepcion de serial---------------

// Función que debes llamar en tu loop principal
void verificarArranqueMotor() {
  // Solo verificar si el sistema está activo y el motor no ha arrancado aún
  if (sistemaActivo && !motorArrancado) {
    // Verificar si ha pasado el tiempo de delay
    if (millis() - tiempoInicioSistema >= DELAY_ARRANQUE_MOTOR) {
      // Arrancar el motor
      digitalWrite(MOTOR_PIN, LOW);  // Activar motor (LOW para avanzar según tu configuración)
      digitalWrite(MOTOR_RETROCESO_PIN, HIGH);  // HIGH para desactivar retroceso
      motorArrancado = true;
      // //ln(F("Motor arrancado"));
    }
  }
}

void setup() {
  //  direccion.setPeriodHertz(50); 
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(MOTOR_RETROCESO_PIN, OUTPUT);
   // Configurar pines como salidas digitales
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
    // Asegurar que empiecen apagados
  digitalWrite(LED_PIN, LOW);

  // Inicializar en estado detenido
  digitalWrite(MOTOR_PIN, HIGH);  // Motor apagado
  digitalWrite(MOTOR_RETROCESO_PIN, HIGH);
  // Toca un tono de frecuencia media (ej. 1500 Hz) durante 100 ms
 // tone(BUZZER_PIN, 1500, 100);
  Serial.begin(115200);
  Serial.setTimeout(1);
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(1000);  // Esperar que el sensor se estabilice
  // Configurar pines de sensores originales
  pinMode(TRIG_DERECHO, OUTPUT);
  pinMode(ECHO_DERECHO, INPUT);
  pinMode(TRIG_IZQUIERDO, OUTPUT);
  pinMode(ECHO_IZQUIERDO, INPUT);
  
  // Configurar pines de sensor frontal de emergencia
  pinMode(TRIG_FRONTAL, OUTPUT);
  pinMode(ECHO_FRONTAL, INPUT);

  // Configurar pin del interruptor con pull-up interno
  pinMode(INTERRUPTOR_PIN, INPUT_PULLUP);
  
  // Inicializar sensor de color temprano para calibración
  if (!tcs.begin()) {
    Serial.println("color  >C");
    while (1);
  }
  
  //ln("iniciado");
  mpu.initialize();
  while (!mpu.testConnection()) {
    Serial.println("Fallo la conexión con MPU6050. Reintentando...");
    delay(1000);
  }
    Serial.println("MPU6050 conectado exitosamente.");

  direccion.attach(SERVO_PIN);
 
 
 // Inicializar variables de motor
  tiempoInicioSistema = 0;
  motorArrancado = false;
  
  // Inicializar variables de interruptor
  interruptorAnterior = digitalRead(INTERRUPTOR_PIN);
  tiempoDebounce = 0;
  sistemaActivo = false;
  sistemaIniciado = false;
  
  // Iniciar período de evaluación para retroceso post-giro
  tiempoInicioEvaluacion = millis();
  
  // Calibración inicial
  ////ln(F("Calibrando..."));
  calibrarGyroAcel();
  // //(F("Offset: "));
  ////ln(offsetGz, 4);
  resetearPIDCentrado(&pidCentrado);

  // Inicializar variables de memoria de color
  ultimoColorDetectado = 'N';
  tiempoUltimoColor = 0;
  bufferIndex = 0;
  lastOrientation = 'C';
  
  // Verificar estado inicial del interruptor
  if (digitalRead(INTERRUPTOR_PIN) == LOW) {
    // Si el interruptor ya está en LOW al iniciar, activar sistema
    unsigned long tiempoEspera = millis();
    while (millis() - tiempoEspera < 100) {
      // Espera no bloqueante de 100ms para estabilizar
    }
    iniciarSistema();
     // Serial.print("centrao");
    direccion.write(SERVO_CENTRADO);
  } else {
    // Si está en HIGH, esperar a que se active
    esperarInterruptor();
    direccion.write(SERVO_CENTRADO);
  }
  
  //Serial.print("centrao");
  ////ln(F("Sistema listo"));
  lastTime = millis();
  tiempoUltimaDeteccion = millis();
  ultimoTiempoCentrado = millis();
}

void loop() {
  // PRIMERA PRIORIDAD: Verificar interruptor SIEMPRE
  verificarInterruptor();
  
  verificarArranqueMotor();

  // Si el sistema no está activo, no ejecutar nada más
  if (!sistemaActivo) {
    delay(50);  // Pequeña pausa para no saturar el procesador
    return;
  }
   if (contadorLineas>= limiteLineas){
    estadoActual ==  ESTADO_DETENCION_FINAL;
  }
    // EJECUTAR SIEMPRE (independiente de comandos seriales)
  actualizarGiro();
   detectarColor();
 /*if (estadoActual != ESTADO_DETENCION_FINAL){
    detectarColor();
  }*/
   // MÁQUINA DE ESTADOS - Solo ejecutar si no hay procesos de alta prioridad
  switch(estadoActual) {
    case ESTADO_NORMAL:
      //centrara vehiculo despues del primer giro
       if (!enGiro && 
          prioridadActual == PRIORIDAD_NORMAL && 
          !esquiveDiagonalActivo && 
          !retrocesoPorEmergencia && !primeraDeteccion) {
        centrarVehiculo();
        //Serial.println("normal");
          }
      break;
      
    case SIGUIENDO_OBJETO:
      if (prioridadActual == PRIORIDAD_NORMAL && 
          !esquiveDiagonalActivo && 
          !retrocesoPorEmergencia) {
          //  Serial.println("siguiendo");
        orientarServoSeguimiento();
        
      }
      break;
      
    case ESQUIVANDO:
      if (prioridadActual == PRIORIDAD_NORMAL && 
          !esquiveDiagonalActivo && 
          !retrocesoPorEmergencia) {
         //   Serial.println("esquive");
        procesarEsquive();
      }
      break;
   
    case ESTADO_DETENCION_FINAL:
     if (contadorEntradaEstadoFinal >= 1){
       // Primer pitido (tono agudo por 50 ms)
      //  tone(BUZZER_PIN, 2500, 50);
        delay(100); // Espera 50ms del tono + 50ms de pausa

        // Segundo pitido
     //   tone(BUZZER_PIN, 2500, 50);

        contadorEntradaEstadoFinal += 1; 
      }
      // Avanzar en línea recta durante 2.5 segundos
      if (millis() - tiempoInicioDetencionFinal < 2000) {
        if (!enGiro && 
          prioridadActual == PRIORIDAD_NORMAL) {
        centrarVehiculo();

        //Serial.println("normal");
          }
        
        // Debug cada segundo
       /* static unsigned long ultimoDebug = 0;
        if (millis() - ultimoDebug > 1000) {
          //("Avance final - Tiempo restante: ");
          //(2500 - (millis() - tiempoInicioAvanceFinal));
          //ln(" ms");
          ultimoDebug = millis();
        }*/
        
      } else {
        //ln("Avance final completado - Motor detenido");
        detenerRobot();
       // estadoActual = ESTADO_NORMAL; // Opcional: volver a estado normal
        
      }
      break;
  }  
  // ===== RESTO DE TU CÓDIGO SOLO SE EJECUTA SI EL SISTEMA ESTÁ ACTIVO =====
  
  // MÁXIMA PRIORIDAD: Retroceso de emergencia (SOLO si NO está girando)
  if (prioridadActual == PRIORIDAD_RETROCESO && !giroEnProceso && !retrocesoOrientadoActivo && estadoActual !=ESTADO_DETENCION_FINAL){
    if (retrocesoPorEmergencia) {
      procesarRetrocesoEmergencia();
    }
    return;
  }

  // Verificar y activar retroceso de emergencia (SOLO si NO está girando)
  if (!giroEnProceso && verificarRetrocesoEmergencia() && !retrocesoOrientadoActivo && estadoActual != ESTADO_DETENCION_FINAL) {
    procesarRetrocesoEmergencia();
    return;
  }
  
  // ALTA PRIORIDAD: Retroceso post-giro
  if (procesarRetrocesoPostGiro()  && estadoActual != ESTADO_DETENCION_FINAL) {
    // El retroceso está activo, no ejecutar nada más
    return;
  }
  
  

  // PRIORIDAD ALTA: Procesar comandos seriales
  /*if (Serial.available() > 0) {
    processSerialData();
    if (contadorLineas>= limiteLineas){
    estadoActual ==  ESTADO_DETENCION_FINAL;
    } else if (estadoActual == SIGUIENDO_OBJETO && !emergenciaActivada) {
      if (verificarEmergencia()) {
        estadoActual = ESQUIVANDO;
        etapaEsquive = 0;
        tiempoEsquive = millis();
        colorObjetivo = colorEmergencia;
      }
    }

    if (emergenciaActivada && estadoActual == ESQUIVANDO) {
      emergenciaActivada = false;
    }
  }
  */
 
   if (!primeraDeteccion) {
    
     // Verificar obstáculos diagonales SOLO si NO está girando
      if (!esquiveDiagonalActivo && !giroEnProceso && verificarObstaculoDiagonal()) {
         // Se activó esquive diagonal
       }
       // Procesar esquive diagonal si está activo Y NO está girando
      if (esquiveDiagonalActivo && !giroEnProceso) {
            procesarEsquiveDiagonal();
       }
  
   }

  // Mostrar el valor cada 300 ms
  if (millis() - lastYawPrint >= YAW_INTERVAL) {
        Serial.print("Yaw: ");
        //Serial.print(yawFiltrado);
        Serial.print(anguloZ);
        Serial.print("°  ");
        lastYawPrint = millis();
        
    }
 
  delay(20);
}

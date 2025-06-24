#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <MPU6050.h>
#include <Servo.h>
#include <ColorConverterLib.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
MPU6050 mpu;
Servo direccion;

const int MOTOR_RETROCESO_PIN = 11;  // Pin para motor de retroceso
const int MOTOR_PIN = 10;
const int SERVO_PIN = 3;
const int LED_PIN = 13;

// Pin para sensor ultrasónico frontal (emergencia)
const int TRIG_FRONTAL = 4;
const int ECHO_FRONTAL = 5;

const int INTERRUPTOR_PIN = 2;  // Pin digital para el interruptor (usa pin con interrupt)

// Variables de estado del sistema
bool sistemaIniciado = false;
bool interruptorAnterior = false;
unsigned long tiempoDebounce = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// ===== CONFIGURACIÓN DE SERVO =====
const int SERVO_LEFT = 180;
const int SERVO_CENTER = 90;
const int SERVO_RIGHT = 0;

// Pines para sensores ultrasónicos
const int TRIG_DERECHO = 7;
const int ECHO_DERECHO = 6;
const int TRIG_IZQUIERDO = 8;
const int ECHO_IZQUIERDO = 9;

const int SERVO_CENTRADO = 90;

// Contador de líneas
int contadorLineas = 0;
int limiteLineas = 12;

int16_t ax, ay, az, gx, gy, gz;

float anguloZ = 0.0;

// Ángulos de giro modificables para cada sentido
float giroObjetivoAzul = 70.0;    // Giro a la izquierda para línea azul
float giroObjetivoNaranja = 70.0; // Giro a la derecha para línea naranja (modificable)

float toleranciaGiro = 5.0;
float offsetGz = 0.0;
float umbralGiro = 1.0;
float umbralReposo = 0.5;

unsigned long lastTime, startTime, timeoutGiro = 5000;

// Variables para control de centrado
bool enGiro = false;
unsigned long tiempoFinGiro = 0;
const unsigned long tiempoEsperaPosgiro = 0;
const int anguloCorreccionMax = 50;
const unsigned long intervaloCentrado = 100;
unsigned long ultimoTiempoCentrado = 0;

// Variables para pausar centrado durante detección
bool pausarCentrado = false;
unsigned long tiempoInicioDeteccion = 0;
const unsigned long tiempoPausaCentrado = 1000;

// Variables para detección de líneas
bool lineaNaranjaDetectada = false;
unsigned long tiempoUltimaDeteccion = 0;
const unsigned long tiempoEsperaDeteccion = 1000;
const unsigned long tiempoEsperaLineaAzul = 3000;

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
  .kp = 5.0,          // Ganancia proporcional - respuesta principal al error
  .ki = 0.7,          // Ganancia integral - elimina error residual
  .kd = 0.6,          // Ganancia derivativa - reduce oscilaciones
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
  BUSQUEDA_ESTACIONAMIENTO,    
  ENDEREZADO_ESTACIONAMIENTO,   
   ESTADO_AVANCE_FINAL   //estado para el avance final
};
//controlar el avance final
unsigned long tiempoInicioAvanceFinal = 0;

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
const int DISTANCIA_EMERGENCIA = 1;
// Tiempos de esquive por color (en milisegundos)
const unsigned long TIEMPO_ESQUIVE_VERDE = 800;   // Tiempo para esquivar verde
const unsigned long TIEMPO_ESQUIVE_ROJO = 500;   // Tiempo para esquivar rojo
const unsigned long TIEMPO_RETORNO_VERDE = 600;   // Tiempo de retorno para verde
const unsigned long TIEMPO_RETORNO_ROJO = 600;    // Tiempo de retorno para rojo

// Variables para almacenar los tiempos actuales
unsigned long tiempoEsquiveActual = 0;
unsigned long tiempoRetornoActual = 0;

// Variables para recordar último color detectado
char ultimoColorDetectado = 'N';           // Último color válido recibido
unsigned long tiempoUltimoColor = 0;       // Timestamp del último color detectado
const unsigned long TIEMPO_MEMORIA_COLOR = 2000; // 4 segundos de memoria

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
const int SHARP_DIAGONAL_IZQ_PIN = A0;  // Pin analógico para sensor Sharp izquierdo
const int SHARP_DIAGONAL_DER_PIN = A1;  // Pin analógico para sensor Sharp derecho

// Variables para esquive diagonal por sensores Sharp
bool esquiveDiagonalActivo = false;
unsigned long tiempoInicioEsquiveDiagonal = 0;
unsigned long tiempoUltimaDeteccionSharp = 0;
char direccionEsquiveDiagonal = 'N'; // 'I' = izquierda, 'D' = derecha
int etapaEsquiveDiagonal = 0;

// Configuración para sensores Sharp diagonales
const float DISTANCIA_SHARP_EMERGENCIA = 15.0;     // 10cm de detección
const unsigned long TIEMPO_CRUCE_DIAGONAL = 700;  // Tiempo de cruce diagonal
const unsigned long TIEMPO_ESPERA_SHARP = 200;     // Evitar detecciones múltiples

// Sistema de prioridades para gestión de emergencias
enum PrioridadEmergencia {
  PRIORIDAD_NORMAL,
  PRIORIDAD_RETROCESO,  // Máxima prioridad
  PRIORIDAD_ESQUIVE     // Prioridad normal
};
PrioridadEmergencia prioridadActual = PRIORIDAD_NORMAL;

// Variables mejoradas para retroceso de emergencia
const int DISTANCIA_RETROCESO_EMERGENCIA = 8;    // Distancia mínima para activar retroceso (cm)
const unsigned long TIEMPO_RETROCESO = 2000;     // Tiempo de retroceso en ms
const unsigned long TIMEOUT_RETROCESO = 3000;    // Timeout de seguridad para retroceso
bool retrocesoPorEmergencia = false;    
// Variables de control temporal
unsigned long tiempoInicioRetroceso = 0;          
unsigned long tiempoUltimaVerificacionRetroceso = 0; 
const unsigned long INTERVALO_VERIFICACION_RETROCESO = 100; // Verificación cada 100ms
const unsigned long TIEMPO_ESPERA_POST_RETROCESO = 500;     // Tiempo de espera después del retroceso
bool motorRetrocesoConfigurado = false;  // controlar configuración de motores

bool retrocesoOrientadoActivo = false;
unsigned long tiempoInicioRetrocesoOrientado = 0;
const unsigned long TIEMPO_RETROCESO_ORIENTADO = 700; // 1 segundo (modificable)
int direccionRetroceso = SERVO_CENTRADO;

// Variables para control de retroceso post-giro
bool retrocesoPermanentementeDeshabilitado = false;
bool periodoEvaluacionInicial = true;
unsigned long tiempoInicioEvaluacion = 0;
const unsigned long TIEMPO_EVALUACION_INICIAL = 3000; // 3 segundos

const float GIRO_MAXIMO = 60.0;      // Giro máximo a menos de 35cm
const float GIRO_MEDIO = 40.0;       // Giro a 35-50cm (puedes cambiar este valor)
const float GIRO_MINIMO = 25.0;      // Giro a 50cm o más

const unsigned long TIMEOUT_RETROCESO_MAXIMO = 2500;  // Timeout de seguridad (2.5 segundos)
unsigned long tiempoRetrocesoActual = 800;  // Variable para tiempo dinámico de retroceso
unsigned long tiempoInicioBusqueda = 0;
const unsigned long TIEMPO_BUSQUEDA_ESTACIONAMIENTO = 3000; // 3 segundos

bool sistemaActivo = false;  // Estado del sistema (activo/inactivo)

// Variable para controlar giro antes de estacionamiento
bool giroParaEstacionamiento = false;

// Variable global para el control de tiempo en setup
unsigned long tiempoInicioSetup = 0;
bool setupCompleto = false;

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
// Función para iniciar el sistema completamente
void iniciarSistema() {
  // //ln(F("=== INICIANDO SISTEMA ==="));
  
  // Activar el sistema
  sistemaActivo = true;
  sistemaIniciado = true;
  
  // Reinicializar todas las variables del sistema
  estadoActual = ESTADO_NORMAL;
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
  
  // Centrar servo y configurar motor
  direccion.write(SERVO_CENTRADO); // Dar tiempo para que se posicione
  delay(3000);
  // Activar motor (LOW para avanzar según tu configuración)
  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(MOTOR_RETROCESO_PIN, HIGH);  // HIGH para desactivar retroceso
  
  // LED indicador de sistema activo - 3 parpadeos rápidos
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  digitalWrite(LED_PIN, HIGH); // Mantener LED encendido mientras sistema esté activo
  
  // //ln(F("Sistema iniciado correctamente"));
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
      else { // Es naranja/rojo/amarillo
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
    // **NUEVA LÓGICA**: Verificar si hay un color objetivo válido ACTUAL o RECORDADO
    bool colorActualValido = (colorObjetivo == 'R' || colorObjetivo == 'G');
    bool colorRecordadoValido = (ultimoColorDetectado == 'R' || ultimoColorDetectado == 'G') && 
                                (millis() - tiempoUltimoColor <= TIEMPO_MEMORIA_COLOR);
    
    if (colorActualValido || colorRecordadoValido) {
      emergenciaActivada = true;
      
      // **NUEVA LÓGICA**: Usar color actual si está disponible, sino usar el recordado
      if (colorActualValido) {
        colorEmergencia = colorObjetivo;
      } else {
        colorEmergencia = ultimoColorDetectado;
        // Actualizar colorObjetivo para que el esquive funcione correctamente
        colorObjetivo = ultimoColorDetectado;
      }
      
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
  // NUEVA VERIFICACIÓN: No centrar si hay emergencias activas
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
  //(F("D:"));
  //(distanciaDerecha, 1);
  //(F(" I:"));
  //(distanciaIzquierda, 1);
  */
  
  // Validar lecturas de sensores
  if (distanciaDerecha > 80 || distanciaIzquierda > 80 || 
      distanciaDerecha < 1 || distanciaIzquierda < 1) {
    resetearPIDCentrado(&pidCentrado);
    return;
  }
  
  // NUEVA VERIFICACIÓN: Doble check de emergencias antes de mover servo
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
  while(1) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
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
  if (anguloUtilizado == GIRO_MAXIMO) {
    return 800;  // 1.2 segundos
  } else if (anguloUtilizado == GIRO_MEDIO) {
    return 800;   // 0.8 segundos
  } else if (anguloUtilizado == GIRO_MINIMO) {
    return 1100;   // 0.5 segundos
  } else {
    return 800;   // Valor por defecto (0.8 segundos)
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
    // Si va a la derecha, usar sensor izquierdo
    distanciaSensor = distanciaIzquierda;
  } else if (sentidoVehiculo == IZQUIERDA) {
    // Si va a la izquierda, usar sensor derecho
    distanciaSensor = distanciaDerecha;
  } else {
    // Fallback: usar ángulos fijos si no hay sentido definido
    return giroIzquierda ? giroObjetivoAzul : giroObjetivoNaranja;
  }
  
  // Debug opcional
  /*
  //("Sentido: ");
  //(sentidoVehiculo == DERECHA ? "DERECHA" : "IZQUIERDA");
  //(", Distancia sensor: ");
  //(distanciaSensor);
  //ln(" cm");
  */
  
  // Calcular ángulo basado en distancia
  float anguloCalculado;
  
  if (distanciaSensor < 35.0) {
    anguloCalculado = GIRO_MAXIMO;  // 70 grados
  } else if (distanciaSensor >= 40.0 && distanciaSensor < 50.0) {
    anguloCalculado = GIRO_MEDIO;   // 40 grados (variable)
  } else {
    anguloCalculado = GIRO_MINIMO;  // 30 grados
  }
  
  return anguloCalculado;
}

void realizarGiro(bool giroIzquierda) {
  enGiro = true;
  giroEnProceso = true;
  cruceEnProceso = true;
  interrupcionCruce = false;
  startTime = millis();
  
  // Calcular ángulo dinámico basado en sensores
  float anguloObjetivo = calcularAnguloGiroDinamico(giroIzquierda);
  unsigned long tiempoRetrocesoCalculado = calcularTiempoRetroceso(anguloObjetivo);
  
  // Resetear y configurar PID
  resetearPID(&pidGiro);
  actualizarGiro();
  float anguloInicial = anguloZ;
  anguloObjetivoActual = anguloInicial + (giroIzquierda ? anguloObjetivo : -anguloObjetivo);
  
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
  
  
  // Validación de tiempo de retroceso
  if (tiempoRetrocesoCalculado <= 0 || tiempoRetrocesoCalculado > TIMEOUT_RETROCESO_MAXIMO) {
    tiempoRetrocesoCalculado = 800;
  }
  
  // ACTIVAR RETROCESO POST-GIRO
  retrocesoOrientadoActivo = true;
  tiempoInicioRetrocesoOrientado = millis();
  direccionRetroceso = giroIzquierda ? SERVO_RIGHT : SERVO_LEFT;
  tiempoRetrocesoActual = tiempoRetrocesoCalculado;
  
  // IMPORTANTE: NO cambiar estadoActual aquí
  // Se cambiará automáticamente a ESTADO_NORMAL cuando termine el retroceso
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
    estadoActual = ESTADO_AVANCE_FINAL;
    tiempoInicioAvanceFinal = millis();
    digitalWrite(MOTOR_RETROCESO_PIN, HIGH);
    digitalWrite(MOTOR_PIN, LOW);
    giroParaEstacionamiento = false;  // Resetear bandera
    //ln("Iniciando avance final - 2 segundos");
    delay(4000);
    detenerRobot();
  } else {
    estadoActual = ESTADO_NORMAL;
  }
  // Paso 6: ESTABLECER ESTADO NORMAL EXPLÍCITAMENTE
  estadoActual = ESTADO_NORMAL;
  
  // Paso 7: Resetear PID para estado limpio
  resetearPIDCentrado(&pidCentrado);
  
  // Paso 8: Reactivar avance normal
 if (estadoActual != ESTADO_AVANCE_FINAL) {
    digitalWrite(MOTOR_PIN, LOW);
  }
  // Debug opcional (comentar en producción)
  // //ln(F("Retroceso post-giro completado - Estado NORMAL activado"));
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

void actualizarGiro() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float velocidadAngularZ = (gz / 131.0) - offsetGz;

  if (abs(velocidadAngularZ) < umbralReposo) {
    velocidadAngularZ = 0;
  }

  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();
  
  if (dt > 0.1) dt = 0.1;

  if (abs(velocidadAngularZ) >= umbralGiro) {
    anguloZ += velocidadAngularZ * dt;
    
    while (anguloZ >= 360) anguloZ -= 360;
    while (anguloZ < 0) anguloZ += 360;
  }

  
}

void calibrarGyro() {
  const int muestras = 200;
  float sumaGz = 0.0;

  for (int i = 0; i < muestras; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumaGz += gz / 131.0;
    delay(10);
  }

  offsetGz = sumaGz / muestras;
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
     if (hue >= 180 && hue <= 250 &&
          saturation > 0.45 &&
          value > 0.30 && value < 0.80 &&
          proporcionAzul > 0.8 &&
          !esBlanco &&
          millis() - tiempoUltimaDeteccion > tiempoEsperaLineaAzul) {
          
        if (confirmarColor(180, 250, 0.45, 0.30, 0.80, 5, 1)) {
          
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
      else if (((hue >= 20 && hue <= 45)) &&
               saturation > 0.40 &&
               value > 0.32 && value < 0.62 &&
               !esBlanco) {
               
        if (confirmarColor(20, 45, 0.40, 0.32, 0.62, 5, 1)) {
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
          if (contadorLineas >= limiteLineas) {
            //ln("limite de lineas alcanzaado");

            resetearPIDCentrado(&pidCentrado);
            
            realizarGiro(true); // Giro a la izquierda
            // REALIZAR EL GIRO PRIMERO antes de buscar estacionamiento
            // Marcar que después del giro debe iniciar búsqueda de estacionamiento
            // Esto se manejará en la función actualizarGiro() cuando termine el giro
            giroParaEstacionamiento = true;
            
          } else {
            resetearPIDCentrado(&pidCentrado);
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
          if (contadorLineas >= limiteLineas) {
            //ln("limite de lineas alcanzaado");
            // REALIZAR EL GIRO PRIMERO antes de buscar estacionamiento
            // Marcar que después del giro debe iniciar búsqueda de estacionamiento
            // Esto se manejará en la función actualizarGiro() cuando termine el giro
            giroParaEstacionamiento = true;
            resetearPIDCentrado(&pidCentrado);
           
            realizarGiro(false); // Giro a la derecha
            
            
          } else {
            resetearPIDCentrado(&pidCentrado);
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
  // **NUEVA VERIFICACIÓN**: Ignorar comandos durante AVANCE FINAL
  if (estadoActual == ESTADO_AVANCE_FINAL) {
    // Durante el avance final, ignorar todos los comandos de detección
    // Solo permitir comandos de emergencia críticos si es necesario
    return;
  }
  
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
      
    case SIGUIENDO_OBJETO:
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
      
    case BUSQUEDA_ESTACIONAMIENTO:
      // Durante búsqueda de estacionamiento, ignorar comandos de seguimiento
      // pero permitir comandos de emergencia si es necesario
      break;
      
    case ENDEREZADO_ESTACIONAMIENTO:
      // Durante enderezado, ignorar comandos normales
      break;
      
    case ESTADO_AVANCE_FINAL:
      // **NUEVO CASE**: Durante avance final, ignorar TODOS los comandos
      // El robot debe completar su avance sin distracciones
      //ln("Comando ignorado - Robot en avance final");
      break;
  }
}
void handleNoDetection() {
  // **NUEVA VERIFICACIÓN**: No procesar durante AVANCE FINAL
  if (estadoActual == ESTADO_AVANCE_FINAL) {
    return; // Ignorar completamente durante avance final
  }
  
  // Si estabas siguiendo un objeto y ya no lo ves...
  if (estadoActual == SIGUIENDO_OBJETO) {
    // Centra el servo
    if (lastOrientation != 'C') {
      direccion.write(SERVO_CENTRADO);
      lastOrientation = 'C';
    }
    
    // **NUEVA LÓGICA**: Solo regresar a estado normal si ha pasado suficiente tiempo
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
        servoPosition = 130; // Izquierda moderada para seguimiento
        break;
      case 'D':
        servoPosition = 50; // Derecha moderada para seguimiento
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
    tiempoEsquiveActual = 1000;
    tiempoRetornoActual = 750;
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
  float voltaje = valorAnalogico * (5.0 / 1023.0);
  
  // Conversión aproximada para sensor Sharp GP2Y0A21YK0F (10-80cm)
  // Fórmula: distancia = 27.728 / (voltaje - 0.1696)
  if (voltaje < 0.4) return 999; // Fuera de rango
  
  float distancia = 27.728 / (voltaje - 0.1696);
  
  // Limitar rango válido
  if (distancia < 5 || distancia > 80) return 999;
  
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
      
      // RESETEAR TODAS LAS VARIABLES CORRECTAMENTE
      retrocesoPorEmergencia = false;
      motorRetrocesoConfigurado = false;  // IMPORTANTE: Resetear para próxima vez
      prioridadActual = PRIORIDAD_NORMAL;
      
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
  tiempoInicioRetroceso = 0;
  tiempoUltimaVerificacionRetroceso = 0;
  
  // Centrar servo
  direccion.write(SERVO_CENTRADO);
  delay(100);
  
  // Reactivar motor principal
  digitalWrite(MOTOR_PIN, LOW);
  
  ////ln("Reset completo finalizado");
}
// Función para iniciar búsqueda de estacionamiento
void iniciarBusquedaEstacionamiento() {
  estadoActual = BUSQUEDA_ESTACIONAMIENTO;
  tiempoInicioBusqueda = millis();
 // //ln(F("INICIANDO BUSQUEDA DE ESTACIONAMIENTO"));
}
// Función para manejar el estado de búsqueda de estacionamiento
void manejarBusquedaEstacionamiento() {
    // Verificar si ha pasado el tiempo de búsqueda
  if (millis() - tiempoInicioBusqueda >= TIEMPO_BUSQUEDA_ESTACIONAMIENTO) {
    ////ln(F("TIEMPO DE BUSQUEDA COMPLETADO - DETENIENDO VEHICULO"));
    detenerRobot();
    return;
  }
  
  // Continuar con todos los procesos normales del vehículo
  // (seguimiento de línea, PID, etc.)
  // El vehículo funcionará normalmente durante estos 3 segundos
}

// Función para activar estado de enderezado
void activarEnderezadoEstacionamiento() {
  estadoActual = ENDEREZADO_ESTACIONAMIENTO;
  ////ln(F("ESTADO: ENDEREZADO_ESTACIONAMIENTO ACTIVADO"));
  // Aquí puedes agregar la lógica específica más tarde
}
// Función para verificar comando 'E' (agregar en tu función de lectura serial)
void verificarComandoEstacionamiento() {
  if (Serial.available() > 0) {
    char comando = Serial.read();
    
    if (comando == 'E' || comando == 'e') {
      if (estadoActual == BUSQUEDA_ESTACIONAMIENTO) {
        activarEnderezadoEstacionamiento();
      }
    }
  }
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
    delay(100);  // Pequeña pausa para estabilizar la lectura
    if (digitalRead(INTERRUPTOR_PIN) == LOW) {
      iniciarSistema();
      delay(1500);
    }
  } else {
    esperarInterruptor();  // Si no está presionado, esperar
  }
}
//----------------- FIN Funciones de evasion y recepcion de serial---------------
void setup() {
  // Inicializar tiempo de inicio
  tiempoInicioSetup = millis();
  
  Serial.begin(115200);
  Serial.setTimeout(1);
  Wire.begin();

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
    while (1);
  }
  
  // Esperar 2 segundos para calibración del sensor de color
  while (millis() - tiempoInicioSetup < 5000) {
    // Durante este tiempo el sensor de color se puede calibrar
    // Puedes agregar lecturas del sensor aquí si necesitas
    // tcs.getRawData(&r, &g, &b, &c); // Ejemplo de lectura durante calibración
  }
  
  //ln("iniciado");
  mpu.initialize();
  if (!mpu.testConnection()) {
    while (1);
  }
   //ln("iniciado");

  direccion.attach(SERVO_PIN);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(MOTOR_RETROCESO_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Inicializar en estado detenido
  digitalWrite(MOTOR_PIN, HIGH);  // Motor apagado
  digitalWrite(MOTOR_RETROCESO_PIN, HIGH);
  direccion.write(SERVO_CENTRADO);
  
  // Inicializar variables de interruptor
  interruptorAnterior = digitalRead(INTERRUPTOR_PIN);
  tiempoDebounce = 0;
  sistemaActivo = false;
  sistemaIniciado = false;
  
  // Iniciar período de evaluación para retroceso post-giro
  tiempoInicioEvaluacion = millis();
  
  // Calibración inicial
  ////ln(F("Calibrando..."));
  calibrarGyro();
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
  } else {
    // Si está en HIGH, esperar a que se active
    esperarInterruptor();
  }
  
  ////ln(F("Sistema listo"));
  lastTime = millis();
  tiempoUltimaDeteccion = millis();
  ultimoTiempoCentrado = millis();
  setupCompleto = true;
}

void loop() {
  // PRIMERA PRIORIDAD: Verificar interruptor SIEMPRE
  verificarInterruptor();
  
  // Si el sistema no está activo, no ejecutar nada más
  if (!sistemaActivo) {
    delay(50);  // Pequeña pausa para no saturar el procesador
    return;
  }
  
  // ===== RESTO DE TU CÓDIGO SOLO SE EJECUTA SI EL SISTEMA ESTÁ ACTIVO =====
  
  // MÁXIMA PRIORIDAD: Retroceso de emergencia (SOLO si NO está girando)
  if (prioridadActual == PRIORIDAD_RETROCESO && !giroEnProceso) {
    if (retrocesoPorEmergencia) {
      procesarRetrocesoEmergencia();
    }
    return;
  }

  // Verificar y activar retroceso de emergencia (SOLO si NO está girando)
  if (!giroEnProceso && verificarRetrocesoEmergencia()) {
    procesarRetrocesoEmergencia();
    return;
  }
  
  // ALTA PRIORIDAD: Retroceso post-giro
  if (procesarRetrocesoPostGiro()) {
    // El retroceso está activo, no ejecutar nada más
    return;
  }
  
  // PRIORIDAD ALTA: Procesar comandos seriales
  if (Serial.available() > 0) {
    processSerialData();
    
    if (estadoActual == SIGUIENDO_OBJETO && !emergenciaActivada) {
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
  
  // MÁQUINA DE ESTADOS - Solo ejecutar si no hay procesos de alta prioridad
  switch(estadoActual) {
    case ESTADO_NORMAL:
      //centrarVehiculo();
       if (!enGiro && 
          prioridadActual == PRIORIDAD_NORMAL && 
          !esquiveDiagonalActivo && 
          !retrocesoPorEmergencia) {
        centrarVehiculo();
          }
      break;
      
    case SIGUIENDO_OBJETO:
      if (prioridadActual == PRIORIDAD_NORMAL && 
          !esquiveDiagonalActivo && 
          !retrocesoPorEmergencia) {
        orientarServoSeguimiento();
      }
      break;
      
    case ESQUIVANDO:
      if (prioridadActual == PRIORIDAD_NORMAL && 
          !esquiveDiagonalActivo && 
          !retrocesoPorEmergencia) {
        procesarEsquive();
      }
      break;
      
    case BUSQUEDA_ESTACIONAMIENTO:
      manejarBusquedaEstacionamiento();
      verificarComandoEstacionamiento(); // Verificar comando 'E'
      break;
      
    case ENDEREZADO_ESTACIONAMIENTO:
      // Lógica para enderezado (implementar más tarde)
      break;
    case ESTADO_AVANCE_FINAL:
      //ln("Estado: AVANCE_FINAL activo");
      
      // Avanzar en línea recta durante 2.5 segundos
      if (millis() - tiempoInicioAvanceFinal < 2500) {
        digitalWrite(MOTOR_PIN, LOW);        // Motor encendido
        direccion.write(SERVO_CENTRADO);     // Dirección centrada
        
        // Debug cada segundo
        static unsigned long ultimoDebug = 0;
        if (millis() - ultimoDebug > 1000) {
          //("Avance final - Tiempo restante: ");
          //(2500 - (millis() - tiempoInicioAvanceFinal));
          //ln(" ms");
          ultimoDebug = millis();
        }
      } else {
        //ln("Avance final completado - Motor detenido");
        detenerRobot();
        estadoActual = ESTADO_NORMAL; // Opcional: volver a estado normal
      }
      break;
  }
  
  // EJECUTAR SIEMPRE (independiente de comandos seriales)
  detectarColor();
  actualizarGiro();

  // Verificar obstáculos diagonales SOLO si NO está girando
  if (!esquiveDiagonalActivo && !giroEnProceso && verificarObstaculoDiagonal()) {
    // Se activó esquive diagonal
  }

  // Procesar esquive diagonal si está activo Y NO está girando
  if (esquiveDiagonalActivo && !giroEnProceso) {
    procesarEsquiveDiagonal();
  }

  delay(20);
}




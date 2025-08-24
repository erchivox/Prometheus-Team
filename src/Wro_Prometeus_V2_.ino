//acuerdate que este codigo tiene 4 segundos para deteccion de una nueva linea
// advertencia : toma 2 tasas de cafe, guarda una copia de seguridad y cuidate del bug que solo pasa a las 
// 12pm, de resto suerte. 
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

//******************** colores 
// Variables para la detección del color azul
const int HUE_AZUL_MIN = 200;
const int HUE_AZUL_MAX = 235;
const float SATURACION_AZUL_MIN = 0.70;
const float VALOR_AZUL_MIN = 0.45;
const float VALOR_AZUL_MAX = 0.82;
const float PROPORCION_AZUL_MIN = 1.66;
const int CONFIRMAR_AZUL_TOLERANCIA = 5;
const int CONFIRMAR_AZUL_DELAY = 1;

// Variables para la detección del color naranja
const int HUE_NARANJA_MIN = 3;
const int HUE_NARANJA_MAX = 45;
const float SATURACION_NARANJA_MIN = 0.40;
const float VALOR_NARANJA_MIN = 0.34;
const float VALOR_NARANJA_MAX = 0.72;
const int CONFIRMAR_NARANJA_TOLERANCIA = 5;
const int CONFIRMAR_NARANJA_DELAY = 1;
//*****888*******************************
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

float anguloZ = 0.0;
float pitch = 0.0;          // Ángulo Pitch (inclinación adelante/atrás)
float roll = 0.0;           // Ángulo Roll (inclinación lateral)


// Factores de peso para el filtro complementario
// K_ACC: Peso del acelerómetro (0.0 a 1.0). Un valor pequeño es común.
// K_GYRO: Peso del giroscopio (1.0 - K_ACC).
const float K_ACC = 0.02; // Puedes ajustar este valor. Más alto = más influencia del acelerómetro, más bajo = más influencia del giroscopio.
const float K_GYRO = 1.0 - K_ACC;


const unsigned long TIEMPO_RETROCESO_MAXIMO = 1000;      // Mayor tiempo para el giro más cerrado
const unsigned long TIEMPO_RETROCESO_ALTO = 800;        // Tiempo intermdio
const unsigned long TIEMPO_RETROCESO_MEDIO_ALTO = 600;  // Menor tiempo

// Ángulos de giro modificables para cada sentido
float giroObjetivoAzul = 70.0;    // Giro a la izquierda para línea azul
float giroObjetivoNaranja = 70.0; // Giro a la derecha para línea naranja (modificable)

float offsetGz = 0.0;
float offsetGx = 0.0; // Offset del giroscopio en X (Roll)
float offsetGy = 0.0; // Offset del giroscopio en Y (Pitch)

float umbralGiro = 1.0;
float umbralReposo = 0.5;
// Control de tiempo
unsigned long lastYawPrint = 0;
const unsigned long YAW_INTERVAL = 300;

unsigned long lastTime, startTime, timeoutGiro = 2000;


// ===== VARIABLES DE CONTROL DE DIRECCIÓN =====
bool corrigiendo = false;           // Estado de corrección activa
unsigned long tiempoInicioCorreccion = 0;
const unsigned long TIEMPO_MAX_CORRECCION = 4000; // 3 segundos máximo corrigiendo
const unsigned long TIEMPO_ESTABILIZACION = 200;  // 200ms para estabilizar antes de verificar

float anguloObjetivo = 0.0;         // Ángulo objetivo dinámico
const float TOLERANCIA_ANGULO = 4.0; //4 antes Tolerancia de ±15° para considerar "recto"

// ===== ESTADOS DE CORRECCIÓN =====
enum EstadoCorreccion {
  RECTO,                  // Vehículo va recto (0° ± margen)
  CORRIGIENDO_IZQUIERDA,  // Corrigiendo hacia la izquierda
  CORRIGIENDO_DERECHA,    // Corrigiendo hacia la derecha
  ESTABILIZANDO           // Estabilizando después de corrección
};

EstadoCorreccion estadoActualCorreccion = RECTO;
bool correccionCompletada = false;  // Flag para indicar que la corrección terminó

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
const unsigned long tiempoPausaCentrado = 100;

// Variables para detección de líneas
bool lineaNaranjaDetectada = false;
unsigned long tiempoUltimaDeteccion = 0;
const unsigned long tiempoEsperaDeteccion = 8000;//4000
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

// Pines para sensores de estacionamiento (el frontal ya está definido)
const int ULTRASONIDO_TRASERO_TRIG_PIN  = 18; // Pin para Trig del sensor ultrasónico trasero
const int ULTRASONIDO_TRASERO_ECHO_PIN = 35; // Pin para Echo del sensor ultrasónico trasero

//-------variables de evasion de objetos--------------

// Estados del sistema integrado
enum EstadoSistema {
  ESTADO_NORMAL,
  ESTADO_DETENCION_FINAL,
  ESPERANDO_COMANDO_CARRIL,
  EVALUANDO_POSICION,
  GIRANDO_HACIA_CARRIL,
  ENDEREZANDO_CARRIL,
  MANTENIENDO_CARRIL
};

// Variables para recepción serial optimizada (del código de esquive)
const int BUFFER_SIZE = 32;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

// Variables para parsing ultra rápido
char colorDetectado = 'N';
int distanciaObjeto = 0;
char orientacionObjeto = 'C';

// Variables de estado
EstadoSistema estadoActual = ESPERANDO_COMANDO_CARRIL;
char colorObjetivo = 'N';
unsigned long tiempoEsquive = 0;
int etapaEsquive = 0;
char lastOrientation = 'X';

// Configuración de distancias para esquive
const int DISTANCIA_ROJO = 35;//40;    // Distancia de esquive para objetos rojos
const int DISTANCIA_VERDE = 40;//45;   // Distancia de esquive para objetos verdes
const int DISTANCIA_EMERGENCIA = 8;
// Tiempos de esquive por color (en milisegundos)
const unsigned long TIEMPO_ESQUIVE_VERDE = 600;   // Tiempo para esquivar verde
const unsigned long TIEMPO_ESQUIVE_ROJO = 400;   // Tiempo para esquivar rojo
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

 //CONSTANTES PARA CORRECCIÓN EN PARED *****
const float DISTANCIA_MINIMA_SHARP_PARED = 12.0; // Distancia en cm para activar la corrección
const int ANGULO_CORRECCION_SHARP_IZQ = 110;     // Ángulo para girar a la izquierda (alejarse de la pared derecha)
const int ANGULO_CORRECCION_SHARP_DER = 70;      // Ángulo para girar a la derecha (alejarse de la pared izquierda)

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
const int DISTANCIA_RETROCESO_EMERGENCIA = 10;    // Distancia mínima para activar retroceso (cm)
const unsigned long TIEMPO_RETROCESO = 2000;     // Tiempo de retroceso en ms
const unsigned long TIMEOUT_RETROCESO = 2500;    // Timeout de seguridad para retroceso
bool retrocesoPorEmergencia = false;    
// Variables de control temporal
unsigned long tiempoInicioRetroceso = 0;          
unsigned long tiempoUltimaVerificacionRetroceso = 0; 
const unsigned long INTERVALO_VERIFICACION_RETROCESO = 100; // Verificación cada 100ms
const unsigned long TIEMPO_ESPERA_POST_RETROCESO = 100;     // Tiempo de espera después del retroceso
bool motorRetrocesoConfigurado = false;  // controlar configuración de motores

bool retrocesoOrientadoActivo = false;
unsigned long tiempoInicioRetrocesoOrientado = 0;
const unsigned long TIEMPO_RETROCESO_ORIENTADO = 700; // 1 segundo (modificable)
int direccionRetroceso = SERVO_CENTRADO;

// Variables para control de retroceso post-giro
unsigned long tiempoInicioEvaluacion = 0;
// ============== CONSTANTES DE ÁNGULOS ==============
const float GIRO_MAXIMO = 55.0;      // Giro máximo - obstáculo muy cerca
const float GIRO_ALTO = 50.0;        // Giro alto - obstáculo cerca
const float GIRO_MEDIO_ALTO = 45.0;  // Giro medio-alto
const float GIRO_MEDIO = 43.0;       // Giro medio
const float GIRO_MEDIO_BAJO = 40.0;  // Giro medio-bajo
const float GIRO_BAJO = 38.0;        // Giro bajo - obstáculo lejano
const float GIRO_MINIMO = 35.0;//30.0 Giro mínimo - obstáculo muy lejano

// Variable para distancia mínima para terminar el retroceso post giro
float DISTANCIA_MINIMA_RETROCESO = 30.0; // cm
const unsigned long TIMEOUT_RETROCESO_MAXIMO = 5000;    // Timeout de seguridad de retroceso orientado
// Variable global para medir tiempo de retroceso durante corrección
unsigned long tiempoTotalRetroceso = 0;
bool huboRetrocesoEnCorreccion = false;

// Variables para el giro de apertura
bool giroAperturaActivo = false;
bool giroAperturaCompletado = false;
unsigned long tiempoInicioApertura = 0;
const unsigned long DURACION_APERTURA_MINIMO = 500;     // Tiempo para el giro más suave
const unsigned long DURACION_APERTURA_BAJO = 300;       // tiempo, más corto
const unsigned long DURACION_APERTURA_MEDIO_BAJO = 100; //  tiempo, el más corto
bool esGiroApertura = false;
unsigned long duracionAperturaActual = DURACION_APERTURA_MINIMO; // Variable para almacenar la duración del giro actualbool esGiroApertura = false;
bool direccionGiroAperturaIzquierda = false;

bool sistemaActivo = false;  // Estado del sistema (activo/inactivo)

// Variable para controlar giro antes de estacionamiento
bool giroParaEstacionamiento = false;

// tiempo de finalizacion del codigo para avance final
unsigned long tiempoInicio = millis();
//unsigned long duracionCiclo = 3000; // 5 segundos en milisegundos

unsigned long tiempoInicioDetencionFinal = 0;

// Variables globales para el control de arranque del motor
unsigned long tiempoInicioSistema = 0;
bool motorArrancado = false;
const unsigned long DELAY_ARRANQUE_MOTOR = 1000; // 1 segundo en milisegundos

// Constantes para corrección de paredes laterales
const float DISTANCIA_MINIMA_PARED = 10.0;  // Distancia mínima a la pared antes de corregir
const float DISTANCIA_MAXIMA_PARED = 35.0;  // Distancia máxima para considerar corrección
const int CORRECCION_PARED_ANGULO = 15;     // Ángulos de corrección por proximidad a pared
const unsigned long INTERVALO_CORRECCION_PARED = 50;  // Intervalo entre correcciones

// Variables para corrección de paredes
unsigned long ultimoTiempoCorreccionPared = 0;

// Variable global para contar líneas desde la última recalibración
int contadorLineasRecalibracion = 0;

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

// Variables globales necesarias (agregar a tu código principal)
float declinacionMagnetica = -9.0; // Para Venezuela - ajustar según tu ubicación
float offsetAngular = 0.0; // Offset adicional si necesitas ajustar el norte de referencia
/*
// Valores de calibración - USAR LOS OBTENIDOS DE TU CÓDIGO DE CALIBRACIÓN
float xOffset = -242.5;    // Tu valor obtenido
float yOffset = 92.5;      // Tu valor obtenido  
float xScale = 1.008;      // Tu valor obtenido
float yScale = 0.992;      // Tu valor obtenido
*/
float xOffset = -2.5;//0.0;    // Reemplazar con valor obtenido
float yOffset = 651.0;//0.0;    // Reemplazar con valor obtenido  
float xScale = 1.028;//1.0;     // Reemplazar con valor obtenido
float yScale = 0.973;//1.0;     // Reemplazar con valor obtenido
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

// variables de giro y retroceso por giro orientado
static bool secuenciaRecuperacionIniciada = false;
static bool direccionEnderezada = false;
static bool avanceRealizado = false;
static unsigned long tiempoInicioAvance = 0;
static bool reintentoRealizado = false;

// Variable para pausa de prueba (configurable)
static unsigned long PAUSA_PRUEBA_MS = 5000; // 5 segundos - cambiar a 0 para desactivar
static bool pausaPruebaActiva = false;
static unsigned long tiempoInicioPausa = 0;

// Configuración para control angular prioritario
#define UMBRAL_ANGULO_MAX 20.0      // Umbral máximo de desviación angular (±20°)
#define TOLERANCIA_ANGULO_FINO 5.0  // Tolerancia para considerar ángulo correcto
#define FACTOR_CORRECCION_ANGULAR 0.8  // Factor fuerte para corrección angular
#define FACTOR_CORRECCION_DISTANCIA 0.3  // Factor menor para distancia cuando hay error angular


// ==================== CONFIGURACIÓN DEL SISTEMA DE CARRILES ====================
struct SistemaCarriles {
  // Definición de carriles
  float distanciaCarril1;
  float distanciaCarril2;  
  float toleranciaCarril;
  float umbralCambio;
  
  // Parámetros de control
  float velocidadGiro;
  unsigned long tiempoGiro;
  unsigned long tiempoGiroCarril1;
  unsigned long tiempoGiroCarril2;
  float toleranciaAngulo;
  
  // Variables de control
  int carrilObjetivo;
  char ladoControl;
  unsigned long tiempoInicioEstado;
  float direccionGiro;
  bool necesitaCambio;
  bool esComandoC21;
  bool esComandoC14; 
};

SistemaCarriles carriles = {
  .distanciaCarril1 = 20.0,
  .distanciaCarril2 = 20.0,
  .toleranciaCarril = 3.0,
  .umbralCambio = 1.0,
  .velocidadGiro = 45.0,
  .tiempoGiro = 1500,
  .tiempoGiroCarril1 = 1800,
  .tiempoGiroCarril2 = 1500,
  .toleranciaAngulo = 5.0,
  .carrilObjetivo = 1,
  .ladoControl = 'D',
  .tiempoInicioEstado = 0,
  .direccionGiro = 0,
  .necesitaCambio = false,
  .esComandoC21 = false,
  .esComandoC14 = false,
};
int ultimoCarrilActivo = 0; 
// Variables de tiempo y monitoreo de carriles
unsigned long ultimaActualizacion = 0;
const unsigned long INTERVALO_CONTROL = 50;
unsigned long ultimoReporte = 0;
const unsigned long INTERVALO_REPORTE = 300;

// Estructura para el control lateral
struct FiltroDistancia {
  float historial[5];
  int indice;
  bool inicializado;
  float ultimaDistanciaValida;
  float umbralCambioMaximo;
  
  void inicializar() {
    for (int i = 0; i < 5; i++) historial[i] = 0;
    indice = 0;
    inicializado = false;
    ultimaDistanciaValida = 0;
    umbralCambioMaximo = 50.0;
  }
  
  float filtrar(float nuevaLectura) {
    if (nuevaLectura <= 0 || nuevaLectura > 150) {
      return ultimaDistanciaValida;
    }
    if (!inicializado || abs(nuevaLectura - ultimaDistanciaValida) <= umbralCambioMaximo) {
      historial[indice] = nuevaLectura;
      indice = (indice + 1) % 5;
      if (!inicializado) {
        for (int i = 0; i < 5; i++) historial[i] = nuevaLectura;
        inicializado = true;
      }
      float suma = 0;
      for (int i = 0; i < 5; i++) suma += historial[i];
      ultimaDistanciaValida = suma / 5.0;
      return ultimaDistanciaValida;
    } else {
      return ultimaDistanciaValida;
    }
  }
};

struct ControlLateral {
  float distanciaObjetivo;
  float toleranciaDistancia;
  float toleranciaAngulo;
  float anguloObjetivo;
  float anguloUmbral;
  float kpDistancia, kiDistancia, kdDistancia;
  float correccionMaxima;
  float factorGiroIzquierda;
  float factorGiroDerecha; 
  float errorDistanciaAnterior;
  float integralDistancia;
  float integralMaxDistancia;
  char ladoControl;
  FiltroDistancia filtroIzquierda;
  FiltroDistancia filtroDerecha;
};

ControlLateral controlLateral = {
  .toleranciaDistancia = 3.0,
  .toleranciaAngulo = 60.0,//30
  .anguloObjetivo = 0.0,
  .anguloUmbral = 20.0, 
  .kpDistancia = 3.5,
  .kiDistancia = 0.6,
  .kdDistancia = 1.2,
  .correccionMaxima = 70.0,
  .factorGiroIzquierda = 0.8,//1.0
  .factorGiroDerecha = 0.8,
  .errorDistanciaAnterior = 0,
  .integralDistancia = 0,
  .integralMaxDistancia = 20.0,
  .ladoControl = 'D'
};
unsigned long ultimaActualizacionf = 0;
const unsigned long INTERVALO_CONTROLf = 50;
unsigned long ultimoReportef = 0; // Añade esta línea
const unsigned long INTERVALO_REPORTEf = 300; // Añade esta línea (imprime ~3 veces/seg)
// ==================== FUNCIONES DE CARRIL ====================
float obtenerDistanciaObjetivo() {
  if (carriles.carrilObjetivo == 1) {
    return carriles.distanciaCarril1;
  } else {
    return carriles.distanciaCarril2;
  }
}

char obtenerLadoControlPorCarril() {
  if (carriles.carrilObjetivo == 1) {
    return 'D';  // Carril 1 siempre usa sensor derecho
  } else {
    return 'I';  // Carril 2 siempre usa sensor izquierdo
  }
}

String obtenerNombreEstado(EstadoSistema estado) { // 1. Cambiado el tipo de parámetro
  switch (estado) {
    // Estados generales
    case ESTADO_NORMAL: return "ESTADO_NORMAL";
    case ESTADO_DETENCION_FINAL: return "ESTADO_DETENCION_FINAL";
    
    // Estados de carril (usando los nombres de EstadoSistema)
    case ESPERANDO_COMANDO_CARRIL: return "ESPERANDO_COMANDO_CARRIL";
    case EVALUANDO_POSICION: return "EVALUANDO_POSICION";
    case GIRANDO_HACIA_CARRIL: return "GIRANDO_HACIA_CARRIL";
    case ENDEREZANDO_CARRIL: return "ENDEREZANDO_CARRIL";
    case MANTENIENDO_CARRIL: return "MANTENIENDO_CARRIL";
    
    default: return "DESCONOCIDO";
  }
}

void cambiarEstado(EstadoSistema nuevoEstado) { // 1. Cambiado el tipo de parámetro
  if (estadoActual != nuevoEstado) { // 2. Se usa la variable global estadoActual
    Serial.println("🔄 Cambio estado: " + obtenerNombreEstado(estadoActual) + 
                  " → " + obtenerNombreEstado(nuevoEstado));
    //*********
    // Si entramos en el estado de mantener carril, guardamos cuál es el carril activo.
    // Esta será nuestra "memoria" para cuando se detecte una línea.
    if (nuevoEstado == MANTENIENDO_CARRIL) {
      ultimoCarrilActivo = carriles.carrilObjetivo;
      Serial.println("✍️ Último carril activo actualizado a: " + String(ultimoCarrilActivo));
    }
    //******
    estadoActual = nuevoEstado; // 3. Se actualiza la variable global
    carriles.tiempoInicioEstado = millis();
    
    // Resetear variables de control del mantenimiento de pared
    controlLateral.integralDistancia = 0;
    controlLateral.errorDistanciaAnterior = 0;

    controlLateral.filtroIzquierda.inicializar();
    controlLateral.filtroDerecha.inicializar();
       // Reset flag C21 si se cambia manualmente de estado
    if (nuevoEstado == ESPERANDO_COMANDO_CARRIL) { // 4. Usar el nombre de EstadoSistema
      carriles.esComandoC21 = false;
      carriles.esComandoC14 = false;
    }
  }
}

// ==================== MÁQUINA DE ESTADOS DE CARRIL ====================
float procesarEstadoEsperandoComando() {
  // Sistema detenido esperando comando
  return 90.0;
}

float procesarEstadoEvaluandoPosicion() {
  
  float distanciaIzq = medirDistancia(TRIG_IZQUIERDO, ECHO_IZQUIERDO);
  float distanciaDer = medirDistancia(TRIG_DERECHO, ECHO_DERECHO);
  // Determinar qué sensor usar para el control principal según 'sentidoVehiculo'
  char ladoControlEvaluacion = (sentidoVehiculo == DERECHA) ? 'I' : 'D';
  float distanciaControl = (ladoControlEvaluacion == 'I') ? distanciaIzq : distanciaDer;

  // El sensor contrario es el que no estamos usando para el control principal
  char ladoContrario = (ladoControlEvaluacion == 'I') ? 'D' : 'I';
    
  // El sensor contrario es el que no estamos usando para el control principal
  float distanciaContraria = (ladoContrario == 'I') ? distanciaIzq : distanciaDer;
  
  // Si el sensor contrario detecta algo < 30cm, FORZAR cambio de carril
  if (distanciaContraria > 0 && distanciaContraria < 30.0) {
    Serial.println(" OBSTÁCULO detectado en sensor " + String(ladoContrario) + 
                  " (" + String(distanciaContraria, 1) + "cm < 30cm)");
    Serial.println(" FORZANDO cambio de carril por seguridad");
    
    // Determinar dirección de giro para alejarse del obstáculo
    if (ladoContrario == 'I') {
      // Obstáculo a la izquierda → girar a la derecha para alejarse
      carriles.direccionGiro = -1.0;
    } else {
      // Obstáculo a la derecha → girar a la izquierda para alejarse
      carriles.direccionGiro = 1.0;
    }
    
  // Actualizar el lado de control según el carril
  controlLateral.ladoControl = ladoControlEvaluacion;
      
    Serial.println(" Giro forzado hacia carril " + String(carriles.carrilObjetivo) + 
                  " (dirección: " + String(carriles.direccionGiro > 0 ? "derecha" : "izquierda") + 
                  " para evitar obstáculo)");
    
    cambiarEstado(GIRANDO_HACIA_CARRIL);
    return 90.0;
  }
  
  // CONTINUAR CON LÓGICA NORMAL: No hay obstáculos, evaluar normalmente
  Serial.println(" Sin obstáculos detectados, evaluando posición normal");
  
  if (distanciaControl < 0) {
    Serial.println("⚠ No se puede leer distancia, manteniendo posición");
    cambiarEstado(ESPERANDO_COMANDO_CARRIL);
    return 90.0;
  }
  
  float distanciaObjetivo = obtenerDistanciaObjetivo();
  float diferencia = distanciaControl - distanciaObjetivo;
  
  Serial.println(" Carril " + String(carriles.carrilObjetivo) + 
                " | Sensor: " + String(ladoControlEvaluacion) +
                " | Distancia actual: " + String(distanciaControl, 1) + 
                "cm | Objetivo: " + String(distanciaObjetivo, 1) + 
                "cm | Diferencia: " + String(diferencia, 1) + "cm");
  
  // Si la diferencia es menor al umbral, ir directo a mantenimiento
  if (abs(diferencia) <= carriles.umbralCambio) {
    Serial.println(" Ya estamos cerca del carril objetivo, manteniendo posición");
    // Actualizar el lado de control según el carril
    controlLateral.ladoControl = ladoControlEvaluacion;
    cambiarEstado(MANTENIENDO_CARRIL);
    return 90.0;
  }

  // Determinar la dirección del giro basado únicamente en el carril objetivo.
  if (carriles.carrilObjetivo == 1) {
    // Para ir al carril 1, SIEMPRE giramos a la DERECHA.
    // Un valor positivo (1.0) resulta en un ángulo de servo bajo (ej: 10), que es un giro a la derecha.
    carriles.direccionGiro = 1.0; 
  } else { // Si el objetivo es el carril 2
    // Para ir al carril 2, SIEMPRE giramos a la IZQUIERDA.
    // Un valor negativo (-1.0) resulta en un ángulo de servo alto (ej: 170), que es un giro a la izquierda.
    carriles.direccionGiro = -1.0;
  }

  // Actualizar el lado de control según el carril
  controlLateral.ladoControl = ladoControlEvaluacion;
  
  Serial.println(" Iniciando giro hacia carril " + String(carriles.carrilObjetivo) + 
                " (sensor: " + String(ladoControlEvaluacion) + 
                ", dirección: " + String(carriles.direccionGiro > 0 ? "derecha" : "izquierda") + ")");
  
  cambiarEstado(GIRANDO_HACIA_CARRIL);
  return 90.0;
}
// Función auxiliar para verificar si el ángulo está dentro del umbral
bool estaEnRangoAngulo(float anguloActual, float anguloObjetivo, float umbral = 3.0) {
  float diferencia = abs(anguloActual - anguloObjetivo);
  
  // Manejar la transición 0°-360°
  if (diferencia > 180.0) {
    diferencia = 360.0 - diferencia;
  }
  
  return diferencia <= umbral;
}
float procesarEstadoGirandoHaciaCarril() {
  // Variables estáticas para recordar el estado de la interrupción
  static bool sharpInterrupcionActiva = false;
  static bool lineaInterrupcionActiva = false;
  static unsigned long tiempoInicioInterrupcion = 0;
  static bool pausaPorSharpRealizada = false;
  
  // Variables estáticas para el control de las fases
  static float anguloGiroFijo = 90.0;
  static bool anguloCalculado = false;
  static bool mensajeFase1Mostrado = false;
  static bool mensajeFase2Mostrado = false;

  // Resetear el estado cada vez que se entra a este estado de giro
  static unsigned long ultimoTiempoInicioEstado = 0;
  if (carriles.tiempoInicioEstado != ultimoTiempoInicioEstado) {
    sharpInterrupcionActiva = false;
    lineaInterrupcionActiva = false;
    pausaPorSharpRealizada = false;
    anguloCalculado = false; 
    
    // Reseteamos los flags de los mensajes para el nuevo giro
    mensajeFase1Mostrado = false;
    mensajeFase2Mostrado = false;

    ultimoTiempoInicioEstado = carriles.tiempoInicioEstado;
    Serial.println("🔄 Nuevo giro iniciado - Sistemas de interrupción reseteados.");
  }

  unsigned long tiempoEnGiro = millis() - carriles.tiempoInicioEstado;
  unsigned long tiempoGiroActual;
  if (carriles.esComandoC21 && carriles.carrilObjetivo == 2) tiempoGiroActual = 2500;
  else if (carriles.esComandoC14 && carriles.carrilObjetivo == 1) tiempoGiroActual = 2500;
  else if (carriles.carrilObjetivo == 1) tiempoGiroActual = carriles.tiempoGiroCarril1;
  else if (carriles.carrilObjetivo == 2) tiempoGiroActual = carriles.tiempoGiroCarril2;
  else tiempoGiroActual = carriles.tiempoGiro;

  // LÓGICA DE INTERRUPCIÓN 
  if (!sharpInterrupcionActiva && !lineaInterrupcionActiva) {
    float distanciaSharp = 999;
    if (carriles.carrilObjetivo == 1) { 
      distanciaSharp = leerSensorSharp(SHARP_DIAGONAL_DER_PIN);
    } else { 
      distanciaSharp = leerSensorSharp(SHARP_DIAGONAL_IZQ_PIN);
    }
    if (distanciaSharp <= 25.0 && distanciaSharp > 0) {
      Serial.println(" INTERRUPCIÓN POR SHARP! (" + String(distanciaSharp, 1) + "cm). Saltando a la fase de corrección AHORA.");
      sharpInterrupcionActiva = true;
      tiempoInicioInterrupcion = millis();
    }
    else {
        if (millis() - tiempoUltimaDeteccion > tiempoEsperaDeteccion) {
            bool lineaDetectada = false;
            if (sentidoVehiculo == IZQUIERDA) {
                if (confirmarColor(HUE_AZUL_MIN, HUE_AZUL_MAX, SATURACION_AZUL_MIN, VALOR_AZUL_MIN, VALOR_AZUL_MAX, CONFIRMAR_AZUL_TOLERANCIA, CONFIRMAR_AZUL_DELAY)) {
                    lineaDetectada = true;
                    Serial.println(" INTERRUPCIÓN POR LÍNEA AZUL! Saltando a la fase de corrección AHORA.");
                }
            }
            else if (sentidoVehiculo == DERECHA) {
                if (confirmarColor(HUE_NARANJA_MIN, HUE_NARANJA_MAX, SATURACION_NARANJA_MIN, VALOR_NARANJA_MIN, VALOR_NARANJA_MAX, CONFIRMAR_NARANJA_TOLERANCIA, CONFIRMAR_NARANJA_DELAY)) {
                    lineaDetectada = true;
                    Serial.println(" INTERRUPCIÓN POR LÍNEA NARANJA! Saltando a la fase de corrección AHORA.");
                }
            }
            if (lineaDetectada) {
                lineaInterrupcionActiva = true;
                tiempoInicioInterrupcion = millis();
            }
        }
      }
  }

  // ==================== ====================
  // FASE DE CORRECCIÓN Y ENDEREZADO (Lógica Unificada)
  if (tiempoEnGiro >= tiempoGiroActual || sharpInterrupcionActiva || lineaInterrupcionActiva) { 
    
    // Manejo de la interrupción por Sharp (sin cambios)
    if (sharpInterrupcionActiva && !pausaPorSharpRealizada) {
      Serial.println("⏸️ SHARP: Pausando motor para corregir...");
      digitalWrite(MOTOR_PIN, HIGH);
      direccion.write(90);
      delay(150); 
       // 2. Realizar un pequeño retroceso
      Serial.println("⏪ SHARP: Realizando retroceso corto...");
      digitalWrite(MOTOR_RETROCESO_PIN, LOW); // Activa el motor de retroceso

      delay(400); // Duración del retroceso (ajustable)
      digitalWrite(MOTOR_RETROCESO_PIN, HIGH); // Detiene el motor de retroceso
      delay(100);

       if (carriles.direccionGiro < 0) {
        direccion.write(170);
      } else {
        direccion.write(10);
      }
      delay(300); 
      // 4. Reanudar la marcha hacia adelante
      digitalWrite(MOTOR_PIN, LOW);
      Serial.println("▶️ SHARP: Reanudando marcha.");
      pausaPorSharpRealizada = true;
      tiempoInicioInterrupcion = millis();
    }

    // 1. Calcular el ángulo objetivo final (hacia dónde debe apuntar al final del giro)
    calcularAnguloObjetivoMantener();

    // 2. Verificar si ya hemos llegado a ese ángulo
    if (estaEnRangoAngulo(anguloZ, anguloObjetivo, 5.0)) {
        // Si ya está derecho, el giro ha terminado.
        Serial.println(" Ángulo corregido. Pasando a mantener carril.");
        cambiarEstado(MANTENIENDO_CARRIL);
        return 90.0; // Centrar ruedas y finalizar.
    } else {
        // Si NO está derecho, aplicar el contra-giro para forzarlo a enderezarse.
        if (!mensajeFase2Mostrado) {
            Serial.println(" Fase 2: Ejecutando contra-giro hasta enderezar.");
            mensajeFase2Mostrado = true;
        }
        
        // La lógica del contra-giro es simple y se basa en el carril de destino:
        if (carriles.carrilObjetivo == 1) {
            // El giro inicial fue a la DERECHA, el contra-giro es a la IZQUIERDA.
            return 170.0;
        } else { // Si el carril objetivo es el 2
            // El giro inicial fue a la IZQUIERDA, el contra-giro es a la DERECHA.
            return 10.0;
        }
    }
  }
  // ===================== ======================

  // FASE B: GIRO INICIAL 
  else {
    if (!anguloCalculado) {
      if (!mensajeFase1Mostrado) {
        Serial.println("🔄 Fase 1: Ejecutando giro inicial por tiempo (cálculo único).");
        mensajeFase1Mostrado = true;
      }
    
      float servoAngleTemporal = 90.0;
      bool calcularAnguloVariable = (carriles.carrilObjetivo == 1 && sentidoVehiculo == DERECHA) ||
                                    (carriles.carrilObjetivo == 2 && sentidoVehiculo == IZQUIERDA);
                                    
      if (calcularAnguloVariable) {
          float distanciaLateral = -1.0;
          if (carriles.carrilObjetivo == 1) {
              distanciaLateral = medirDistancia(TRIG_IZQUIERDO, ECHO_IZQUIERDO);
          } else {
              distanciaLateral = medirDistancia(TRIG_DERECHO, ECHO_DERECHO);
          }
          
          Serial.print("Distancia lateral medida: ");
          Serial.print(distanciaLateral);
          Serial.println(" cm");

          if (distanciaLateral > 0 && distanciaLateral < 100) {
              float ajuste = map(constrain(distanciaLateral, 32.0, 50.0), 32.0, 50.0, 0.0, 20.0);
              if (carriles.direccionGiro < 0) { 
                  servoAngleTemporal = 170.0 - ajuste; 
              } else { 
                  servoAngleTemporal = 10.0 + ajuste; 
              }
              
              Serial.print("Ajuste de ángulo: ");
              Serial.print(ajuste);
              Serial.print("°, Servo final: ");
              Serial.println(servoAngleTemporal);

          } else {
              if (carriles.direccionGiro < 0) servoAngleTemporal = 170.0;
              else servoAngleTemporal = 10.0;
              Serial.println("Lectura de distancia inválida, usando ángulo fijo.");
          }

      } else {
          if (carriles.direccionGiro < 0) servoAngleTemporal = 170.0;
          else servoAngleTemporal = 10.0;
      }
      
      anguloGiroFijo = constrain(servoAngleTemporal, 0, 180);
      anguloCalculado = true;
    }
    
    return anguloGiroFijo;
  }
}

float procesarEstadoEnderezandoCarril() {

  
  if (abs(anguloZ) <= carriles.toleranciaAngulo) {
    Serial.println("📐 Vehículo enderezado, iniciando mantenimiento de carril");
    cambiarEstado(MANTENIENDO_CARRIL);
    return 90.0;
  }
  
  // Control proporcional para enderezar
  float correccionAngulo = -anguloZ * 2.0;  // Ganancia proporcional
  correccionAngulo = constrain(correccionAngulo, -40, 50);
  
  return constrain(90.0 + correccionAngulo, 0, 180);
}
float obtenerDistanciaPorSentido(char lado) {
  float lectura;
  if (lado == 'I') {
    lectura = medirDistancia(TRIG_IZQUIERDO, ECHO_IZQUIERDO);
    return controlLateral.filtroIzquierda.filtrar(lectura);
  } else {
    lectura = medirDistancia(TRIG_DERECHO, ECHO_DERECHO);
    return controlLateral.filtroDerecha.filtrar(lectura);
  }
}

float procesarMantenimientoPared() {

   // Prioridad máxima: si un sensor Sharp detecta una pared demasiado cerca,
  // se anula el PID y se ejecuta una corrección inmediata.

  if (carriles.carrilObjetivo == 1) {
    // Carril 1: controlado por la derecha, usamos el sensor Sharp derecho.
    float distSharpDer = leerSensorSharp(SHARP_DIAGONAL_DER_PIN);
    if (distSharpDer > 0 && distSharpDer < DISTANCIA_MINIMA_SHARP_PARED) {
      Serial.println(" SHARP DERECHO: Pared muy cerca (" + String(distSharpDer, 1) + "cm). Corrigiendo a la IZQUIERDA.");
      actualizarGiro(); // Actualizamos el ángulo mientras giramos
      return ANGULO_CORRECCION_SHARP_IZQ; // Devuelve un ángulo para girar a la izquierda.
    }
  } else { // carriles.carrilObjetivo == 2
    // Carril 2: controlado por la izquierda, usamos el sensor Sharp izquierdo.
    float distSharpIzq = leerSensorSharp(SHARP_DIAGONAL_IZQ_PIN);
    if (distSharpIzq > 0 && distSharpIzq < DISTANCIA_MINIMA_SHARP_PARED) {
      Serial.println(" SHARP IZQUIERDO: Pared muy cerca (" + String(distSharpIzq, 1) + "cm). Corrigiendo a la DERECHA.");
      actualizarGiro(); // Actualizamos el ángulo mientras giramos
      return ANGULO_CORRECCION_SHARP_DER; // Devuelve un ángulo para girar a la derecha.
    }
  }
  // Usar el sensor correspondiente al carril actual
  char ladoControlCarril = obtenerLadoControlPorCarril();
  
  float distanciaIzq = obtenerDistanciaPorSentido('I');
  float distanciaDer = obtenerDistanciaPorSentido('D');
  float distanciaControl = (ladoControlCarril == 'I') ? distanciaIzq : distanciaDer;
  
  if (distanciaControl <= 0) return 90.0;

  // PID para distancia
  float distanciaObjetivo = obtenerDistanciaObjetivo();
  float errorDistancia = distanciaObjetivo - distanciaControl;
  controlLateral.integralDistancia += errorDistancia;
  controlLateral.integralDistancia = constrain(controlLateral.integralDistancia,
                                               -controlLateral.integralMaxDistancia,
                                               controlLateral.integralMaxDistancia);
  
  float derivadaDistancia = errorDistancia - controlLateral.errorDistanciaAnterior;
  controlLateral.errorDistanciaAnterior = errorDistancia;
  
  float correccionDistancia = (controlLateral.kpDistancia * errorDistancia) +
                              (controlLateral.kiDistancia * controlLateral.integralDistancia) +
                              (controlLateral.kdDistancia * derivadaDistancia);

  // Corrección angular suave
  float correccionAngulo = 0.0;
  calcularAnguloObjetivoMantener();
  controlLateral.anguloObjetivo = anguloObjetivo;
  float errorAngulo = controlLateral.anguloObjetivo - anguloZ;

  if (errorAngulo > 180.0) {
      errorAngulo -= 360.0;
  } else if (errorAngulo < -180.0) {
      errorAngulo += 360.0;
  }

  float correccionTotal = 0.0;

  // PRIORIDAD 1: Si el ángulo está muy desviado, corregirlo es lo único que importa.
  if (abs(errorAngulo) > controlLateral.anguloUmbral) {
      // La corrección se basa únicamente en el ángulo.
      correccionTotal = errorAngulo * 0.8; // Aumentamos un poco el factor para una corrección más decidida.
  } 
  // PRIORIDAD 2: Si el ángulo es aceptable, entonces nos enfocamos en la distancia.
  else {
      // La corrección se basa únicamente en el PID de distancia.
      correccionTotal = correccionDistancia;
  }

  // Se mantiene la lógica para invertir la corrección según el lado de control.
  if (ladoControlCarril == 'I') {
    correccionTotal = -correccionTotal;
  }
  
  // Se mantiene la lógica para el control asimétrico (ajuste fino).
  if (correccionTotal > 0) {
    correccionTotal *= controlLateral.factorGiroIzquierda;
  } else if (correccionTotal < 0) {
    correccionTotal *= controlLateral.factorGiroDerecha;
  }
  
  // Se limita la corrección total para no saturar el servo.
  correccionTotal = constrain(correccionTotal,
                              -controlLateral.correccionMaxima,
                              controlLateral.correccionMaxima);

  // Calcular el valor de retorno normal del control PID
  float valorServo = constrain(90.0 + correccionTotal, 0, 180);

  // LÓGICA SIMPLIFICADA PARA C21: Cambio inmediato sin esperar estabilización
  if (carriles.esComandoC21 && carriles.carrilObjetivo == 1) {
    static unsigned long tiempoInicioCarril1 = 0;
    static bool primerIngresoCarril1 = true;
    
    // Marcar tiempo de entrada al carril 1
    if (primerIngresoCarril1) {
      tiempoInicioCarril1 = millis();
      primerIngresoCarril1 = false;
      Serial.println(" C21: Llegó a carril 1, iniciando cuenta para cambio automático...");
    }
    
    // Después de en carril 1, cambiar automáticamente a carril 2
      carriles.carrilObjetivo = 2;
      controlLateral.ladoControl = 'I';  // Cambiar a control izquierdo para carril 2
      Serial.println(" C21: Cambiando automáticamente a carril 2 (sin esperar estabilización)");
      cambiarEstado(EVALUANDO_POSICION);
      primerIngresoCarril1 = true;  // Reset para próxima vez
      return -1;  // CAMBIO: Valor especial indicando que no se debe cambiar la dirección
    
  }
  // LÓGICA C15: TRANSICIÓN INSTANTÁNEA Carril 2 → Carril 1
  if (carriles.esComandoC14 && carriles.carrilObjetivo == 2) {
    carriles.carrilObjetivo = 1;
    controlLateral.ladoControl = 'D';  // Cambiar a control derecho para carril 1
    Serial.println(" C14: Transición INSTANTÁNEA de carril 2 → carril 1");
    cambiarEstado(EVALUANDO_POSICION);
    return -1;  // CAMBIO: Valor especial indicando que no se debe cambiar la dirección
  }
  
  // Retorno normal del control PID
  return valorServo;
}

float calcularControlCarriles() {
  switch (estadoActual) { // Usa la variable global 'estadoActual'
    case ESPERANDO_COMANDO_CARRIL: // Usa el nombre correcto del enum
      return procesarEstadoEsperandoComando();
      
    case EVALUANDO_POSICION:
      return procesarEstadoEvaluandoPosicion();
      
    case GIRANDO_HACIA_CARRIL:
      return procesarEstadoGirandoHaciaCarril();
      
    case ENDEREZANDO_CARRIL:
      return procesarEstadoEnderezandoCarril();

    case MANTENIENDO_CARRIL: {
      float resultado = procesarMantenimientoPared();
      // Si es -1, significa que no se debe cambiar la dirección
      if (resultado == -1) {
        return -1;  // Propagar el valor especial
      }
      return resultado;
    }

      
    default:
      return 90.0;
  }
}

void procesarComandoCarril(String comando) {
  int carrilAnterior = carriles.carrilObjetivo;
  // VERDE se esquiva por la izquierda. y posicionado a la izquierda **apartir del 13 son dobles verdes
  if ( comando == "C07" || comando == "C09" || comando == "C37" || comando == "C13" || comando == "C19"|| comando == "C31") {
  carriles.carrilObjetivo = 2;
  carriles.distanciaCarril2 = 15.0; // Restablece la distancia al valor por defecto
  carriles.esComandoC21 = false;
  
  //  Casaos Verde a la derecha. **apartir del 13 son dobles verdes **apartir del 25 son dobles verdes 
  } else if (comando == "C01" || comando == "C03"|| comando == "C05"|| comando == "C25") {
    carriles.carrilObjetivo = 2;
    carriles.distanciaCarril2 = 30.0; // Cambia la distancia solo para este comando
    carriles.esComandoC21 = false;

  // Comando especial C21 - primero va a carril 1, luego a carril 2
  } else if (comando == "C21"||comando == "C27"||comando == "C33" || comando == "C15") {
    carriles.carrilObjetivo = 1;    // Primero ir a carril 1
    carriles.esComandoC21 = true;   // Marcar como comando especial
    Serial.println("🔄 Comando C21: Primero carril 1, luego carril 2 automáticamente");
  
  // Comando especial C15 - primero va a carril 2, luego a carril 1
  } else if (comando == "C14"||comando == "C20"||comando == "C32") {
    carriles.carrilObjetivo = 2;    // Primero ir a carril 2
    carriles.esComandoC14 = true;   // Marcar como comando especial
    carriles.esComandoC21 = false;  // Resetear flag C21
    Serial.println("🔄 Comando C15: Primero carril 2, luego carril 1 automáticamente");
  
  // Comandos para carril 1  DERECHA COMANDOS PARA ROJO. rojos a la derecha.
  } else if (comando == "C02" || comando == "C04"  ||comando == "C06" ||comando == "C18"||comando == "C24"||comando == "C30") {
    carriles.carrilObjetivo = 1;
    carriles.distanciaCarril1 = 15.0; 
    carriles.esComandoC21 = false;  // Resetear flag C21
    
  //  Casaos Rojos a la izquierda. mayor distancia con la pared. 
  } else if (comando == "C08" || comando == "C10"||comando == "C38"||comando == "C36") {
    carriles.carrilObjetivo = 1;
    carriles.distanciaCarril1 = 30.0; // Cambia la distancia solo para este comando
    carriles.esComandoC21 = false;

  // Comando especial C21 - primero va a carril 1, luego a carril 2
  }  else {
    Serial.println(" Comando no reconocido: " + comando);
    return;
  }
  
  Serial.println(" Comando recibido: " + comando + " → Carril " + String(carriles.carrilObjetivo));
  
  // Si cambió el carril objetivo, iniciar evaluación
 if (carrilAnterior != carriles.carrilObjetivo || estadoActual == ESPERANDO_COMANDO_CARRIL) { // Corregido aquí
    cambiarEstado(EVALUANDO_POSICION);
  }
}
void resetSistema(bool conservarAngulo = true) {
  // Reset de variables de control lateral
  controlLateral.integralDistancia = 0;
  controlLateral.errorDistanciaAnterior = 0;
  controlLateral.filtroIzquierda.inicializar();
  controlLateral.filtroDerecha.inicializar();

  // Reset de flags de comandos especiales
  carriles.esComandoC21 = false;
  carriles.esComandoC14 = false;

  // Reset de estado del sistema de carriles
  carriles.carrilObjetivo = 1;
  carriles.ladoControl = 'D';
  estadoActual = ESTADO_NORMAL; // Asigna a la variable global 'estadoActual'
  carriles.tiempoInicioEstado = millis();
  carriles.necesitaCambio = false;

  // Recentrar servo
 // direccion.write(90);

  Serial.println("🔄 Sistema reseteado" + String(conservarAngulo ? " (ángulo conservado)" : " (ángulo reiniciado)"));
}
// Función auxiliar para generar pulsos cortos (volumen bajo)
void buzzerVolumenBajo(int duracionTotal) {
  int tiempoTranscurrido = 0;
  while (tiempoTranscurrido < duracionTotal) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(20);  // Pulso muy corto
    digitalWrite(BUZZER_PIN, LOW);
    delay(80);  // Pausa larga
    tiempoTranscurrido += 100;  // 20ms + 80ms = 100ms por ciclo
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
 //   buzzerVolumenBajo(100);  // 100ms con pulsos cortos
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
  
  Serial.println(F("Sistema detenido completamente"));
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
  if (millis() - tiempoUltimaEmergencia < TIEMPO_ESPERA_EMERGENCIA) {
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
      
      // Usar color actual si está disponible, sino usar el recordado
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

// Función auxiliar para normalizar la diferencia angular
float normalizarDiferenciaAngular(float diferencia) {
  while (diferencia > 180.0) diferencia -= 360.0;
  while (diferencia < -180.0) diferencia += 360.0;
  return diferencia;
}

void calcularAnguloObjetivoCenter() {
  if (sentidoVehiculo == IZQUIERDA) {
    // Giro a la izquierda: 1->0°, 2->90°, 3->180°, 4->270°, 5->0°, 6->90°...
    int ciclo = (contadorLineas - 1) % 4;  // Ajustar para que comience en 1
    switch (ciclo) {
      case 0: anguloObjetivo = 0.0; break;   // contadorLineas = 1, 5, 9...
      case 1: anguloObjetivo = 90.0; break;  // contadorLineas = 2, 6, 10...
      case 2: anguloObjetivo = 180.0; break; // contadorLineas = 3, 7, 11...
      case 3: anguloObjetivo = 270.0; break; // contadorLineas = 4, 8, 12...
    }
  } else {
    // Giro a la derecha: 1->0°, 2->270°, 3->180°, 4->90°, 5->0°, 6->270°...
    int ciclo = (contadorLineas - 1) % 4;  // Ajustar para que comience en 1
    switch (ciclo) {
      case 0: anguloObjetivo = 0.0; break;   // contadorLineas = 1, 5, 9...
      case 1: anguloObjetivo = 270.0; break; // contadorLineas = 2, 6, 10...
      case 2: anguloObjetivo = 180.0; break; // contadorLineas = 3, 7, 11...
      case 3: anguloObjetivo = 90.0; break;  // contadorLineas = 4, 8, 12...
    }
  }
}
void calcularAnguloObjetivoMantener() {
  if (sentidoVehiculo == IZQUIERDA) {
    // Giro a la izquierda: 1->90°, 2->180°, 3->270°, 4->0°, etc.
    int ciclo = (contadorLineas -1) % 4; // Ajuste para que la primera línea sea ciclo 0
    switch (ciclo) {
      case 0: anguloObjetivo = 90.0; break;  // Línea 1, 5, 9...
      case 1: anguloObjetivo = 180.0; break; // Línea 2, 6, 10...
      case 2: anguloObjetivo = 270.0; break; // Línea 3, 7, 11...
      case 3: anguloObjetivo = 0.0; break;   // Línea 4, 8, 12...
    }
  } else { // sentidoVehiculo == DERECHA
    // Giro a la derecha: 1->270°, 2->180°, 3->90°, 4->0°, etc.
    int ciclo = (contadorLineas - 1) % 4; // Ajuste para que la primera línea sea ciclo 0
    switch (ciclo) {
      case 0: anguloObjetivo = 270.0; break; // Línea 1, 5, 9...
      case 1: anguloObjetivo = 180.0; break; // Línea 2, 6, 10...
      case 2: anguloObjetivo = 90.0; break;  // Línea 3, 7, 11...
      case 3: anguloObjetivo = 0.0; break;   // Línea 4, 8, 12...
    }
  }
}
// Función para centrar el vehículo con PRIORIDAD ANGULAR
void centrarVehiculo() {
  // NUEVA VERIFICACIÓN: No centrar si hay emergencias activas
  if (prioridadActual != PRIORIDAD_NORMAL || 
      esquiveDiagonalActivo || 
      retrocesoPorEmergencia) {
    if (pidCentrado.inicializado) {
      resetearPIDCentrado(&pidCentrado);
    }
    return;
  }
  
  // No centrar si está en giro, recién terminó giro, pausado por detección, o en esquive
  if (enGiro || 
      millis() - tiempoFinGiro < tiempoEsperaPosgiro ||
      (pausarCentrado && millis() - tiempoInicioDeteccion < tiempoPausaCentrado)) {
    
    if (pidCentrado.inicializado) {
      resetearPIDCentrado(&pidCentrado);
    }
    return;
  }
  
  if (millis() - ultimoTiempoCentrado < intervaloCentrado) {
    return;
  }
  
  // Calcular el ángulo objetivo SIEMPRE antes de proceder
  calcularAnguloObjetivoCenter();
  
  // Leer sensores con pequeña pausa para estabilidad
  float distanciaDerecha = medirDistancia(TRIG_DERECHO, ECHO_DERECHO);
  delay(5);
  float distanciaIzquierda = medirDistancia(TRIG_IZQUIERDO, ECHO_IZQUIERDO);
  
  // Validar lecturas de sensores
  if (distanciaDerecha > 80 || distanciaIzquierda > 80 || 
      distanciaDerecha < 1 || distanciaIzquierda < 1) {
    resetearPIDCentrado(&pidCentrado);
    return;
  }
  
  // VERIFICACIÓN FINAL antes de mover servo
  if (prioridadActual != PRIORIDAD_NORMAL || 
      esquiveDiagonalActivo || 
      retrocesoPorEmergencia) {
    resetearPIDCentrado(&pidCentrado);
    return;
  }
  
  // ===== CÁLCULO CON PRIORIDAD ANGULAR =====
  
  // 1. Calcular diferencia angular
  float diferenciaAngular = normalizarDiferenciaAngular(anguloZ - anguloObjetivo);
  
  // 2. VERIFICAR UMBRAL MÁXIMO - Si excede ±20°, SOLO corregir ángulo
  if (abs(diferenciaAngular) > UMBRAL_ANGULO_MAX) {
    // EMERGENCIA ANGULAR: Solo corregir ángulo, ignorar distancia
    float errorFinal = diferenciaAngular * FACTOR_CORRECCION_ANGULAR;
    
    // Debug para emergencia angular
    /*
    Serial.print(F("EMERGENCIA_ANG - AngZ:"));
    Serial.print(anguloZ, 1);
    Serial.print(F(" AngObj:"));
    Serial.print(anguloObjetivo, 1);
    Serial.print(F(" Dif:"));
    Serial.print(diferenciaAngular, 1);
    Serial.print(F(" ErrFinal:"));
    Serial.println(errorFinal, 1);
    */
    
    // Aplicar corrección PID solo angular
    float salidaPID = calcularPIDCentrado(&pidCentrado, errorFinal);
    salidaPID = constrain(salidaPID, -ERROR_MAXIMO_CENTRADO, ERROR_MAXIMO_CENTRADO);
    
    // Convertir a ángulo de servo
    float factorEscala = anguloCorreccionMax / ERROR_MAXIMO_CENTRADO;
    int correccionAngulo = salidaPID * factorEscala;
    
    static int anguloAnterior = SERVO_CENTRADO;
    int anguloDeseado = SERVO_CENTRADO - correccionAngulo;
    anguloDeseado = constrain(anguloDeseado, SERVO_CENTRADO - anguloCorreccionMax, 
                              SERVO_CENTRADO + anguloCorreccionMax);
    
    int anguloFinal = anguloAnterior * (1 - FACTOR_SUAVIZADO) + anguloDeseado * FACTOR_SUAVIZADO;
    anguloAnterior = anguloFinal;
    
    // Aplicar al servo
    if (prioridadActual == PRIORIDAD_NORMAL && 
        !esquiveDiagonalActivo && 
        !retrocesoPorEmergencia) {
      direccion.write(anguloFinal);
    }
    
    ultimoTiempoCentrado = millis();
    return; // Salir sin considerar distancia
  }
  
  // 3. Si está dentro del umbral, proceder con lógica de prioridad
  float errorDistancia = distanciaDerecha - distanciaIzquierda;
  float errorFinal = 0;
  
  // PRIORIDAD 1: Corregir ángulo si está fuera de tolerancia fina
  if (abs(diferenciaAngular) > TOLERANCIA_ANGULO_FINO) {
    // El ángulo es la prioridad principal
    errorFinal = diferenciaAngular * FACTOR_CORRECCION_ANGULAR;
    
    // Solo agregar corrección de distancia si también está muy descentrado
    if (abs(errorDistancia) > TOLERANCIA_CENTRADO_PID * 2) {
      errorFinal += errorDistancia * FACTOR_CORRECCION_DISTANCIA;
    }
    
    /*
    Serial.print(F("MODO_ANGULAR - Dif:"));
    Serial.print(diferenciaAngular, 1);
    Serial.print(F(" ErrDist:"));
    Serial.print(errorDistancia, 1);
    Serial.print(F(" ErrFinal:"));
    Serial.println(errorFinal, 1);
    */
    
  } 
  // PRIORIDAD 2: Si el ángulo está bien, entonces corregir distancia
  else if (abs(errorDistancia) > TOLERANCIA_CENTRADO_PID) {
    // Solo corrección por distancia
    errorFinal = errorDistancia;
    
    /*
    Serial.print(F("MODO_DISTANCIA - ErrDist:"));
    Serial.print(errorDistancia, 1);
    Serial.print(F(" ErrFinal:"));
    Serial.println(errorFinal, 1);
    */
  }
  // PRIORIDAD 3: Todo está bien
  else {
    // Ya está centrado tanto en ángulo como en distancia
    direccion.write(SERVO_CENTRADO);
    pidCentrado.integral = 0;
    pausarCentrado = false;
    
    /*
    Serial.println(F("CENTRADO_TOTAL"));
    */
    
    ultimoTiempoCentrado = millis();
    return;
  }
  
  // ===== APLICAR CONTROL PID =====
  float salidaPID = calcularPIDCentrado(&pidCentrado, errorFinal);
  salidaPID = constrain(salidaPID, -ERROR_MAXIMO_CENTRADO, ERROR_MAXIMO_CENTRADO);
  
  // Convertir salida PID a ángulo de servo
  float factorEscala = anguloCorreccionMax / ERROR_MAXIMO_CENTRADO;
  int correccionAngulo = salidaPID * factorEscala;
  
  // Aplicar suavizado para evitar cambios bruscos
  static int anguloAnterior = SERVO_CENTRADO;
  int anguloDeseado = SERVO_CENTRADO - correccionAngulo;
  anguloDeseado = constrain(anguloDeseado, SERVO_CENTRADO - anguloCorreccionMax, 
                            SERVO_CENTRADO + anguloCorreccionMax);
  
  // Aplicar suavizado
  int anguloFinal = anguloAnterior * (1 - FACTOR_SUAVIZADO) + anguloDeseado * FACTOR_SUAVIZADO;
  anguloAnterior = anguloFinal;
  
  // VERIFICACIÓN FINAL antes de mover servo
  if (prioridadActual == PRIORIDAD_NORMAL && 
      !esquiveDiagonalActivo && 
      !retrocesoPorEmergencia) {
    direccion.write(anguloFinal);
  }
  
  // Debug de la corrección PID
  /*
  Serial.print(F(" PID:"));
  Serial.print(salidaPID, 1);
  Serial.print(F(" Servo:"));
  Serial.print(anguloFinal);
  Serial.print(F(" Dif:"));
  Serial.print(diferenciaAngular, 1);
  Serial.print(errorFinal > 0 ? F(" <-") : F(" ->"));
  Serial.println();
  */
  
  ultimoTiempoCentrado = millis();
}

void detenerRobot() {
  // Detener ambos motores para asegurar una parada completa
  digitalWrite(MOTOR_PIN, HIGH);           
  digitalWrite(MOTOR_RETROCESO_PIN, HIGH); 

  // Centrar el servo de la dirección
  direccion.write(SERVO_CENTRADO); 

  // Resetear cualquier flag de estado de movimiento
  enGiro = false; 
  
  // YA NO HAY BUCLE INFINITO. La función termina aquí.
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
    Serial.println("VAINA RARA EN calcularAnguloGiroDinamico ME VUELVE EL ES GIRO APERTURA A FALSE");
    //esGiroApertura = false; // No es giro mínimo en fallback
    return giroParaEstacionamiento ? anguloFallback + 5.0 : anguloFallback;
  }
  
  // Calcular ángulo basado en distancia con más subdivisiones
  float anguloCalculado;
  
  if (distanciaSensor < 25.0) {
    // Obstáculo muy cerca - giro máximo
    anguloCalculado = GIRO_MAXIMO;
    esGiroApertura = false;
  } else if (distanciaSensor >= 25.0 && distanciaSensor < 30.0) {
    // Obstáculo cerca - giro alto
    anguloCalculado = GIRO_ALTO;
    esGiroApertura = false;
  } else if (distanciaSensor >= 30.0 && distanciaSensor < 35.0) {
    // Obstáculo medio-cerca - giro medio-alto
    anguloCalculado = GIRO_MEDIO_ALTO;
    esGiroApertura = false;
  } else if (distanciaSensor >= 35.0 && distanciaSensor < 40.0) {
    // Obstáculo medio - giro medio
    anguloCalculado = GIRO_MEDIO;
    esGiroApertura = false;
  } else if (distanciaSensor >= 40.0 && distanciaSensor < 45.0) {
    // Obstáculo medio-lejos - giro medio-bajo
    anguloCalculado = GIRO_MEDIO_BAJO;
    esGiroApertura = true; 
  } else if (distanciaSensor >= 45.0 && distanciaSensor < 50.0) {
    // Obstáculo lejos - giro bajo
    anguloCalculado = GIRO_BAJO; 
    esGiroApertura = true; 
  } else {
    // Obstáculo muy lejos o no detectado - giro mínimo
   anguloCalculado = GIRO_MINIMO; 
    esGiroApertura = true; 
  }
  // Si está en modo de estacionamiento, agregar 5 grados
  if (giroParaEstacionamiento) {
    anguloCalculado += 5.0;
  }
  
  // Debug opcional - descomenta para ver valores
  
  Serial.print("Distancia: ");
  Serial.print(distanciaSensor);
  Serial.print(" cm, Ángulo: ");
  Serial.print(anguloCalculado);
  Serial.println("°");
  
  
  return anguloCalculado;
}
void realizarGiroApertura(bool giroIzquierda) {
  Serial.println(" INICIANDO GIRO DE APERTURA...");
  Serial.println("   Duración objetivo: " + String(duracionAperturaActual) + " ms");
  
  // Activar flags de apertura
  giroAperturaActivo = true;
  giroAperturaCompletado = false;
  tiempoInicioApertura = millis();

  //Guardar la dirección del giro para usarla después
  direccionGiroAperturaIzquierda = giroIzquierda;
  
  // Determinar dirección del servo para apertura (sentido CONTRARIO al giro actual)
  int servoApertura;
  if (giroIzquierda) {
    // Si el giro principal es a la IZQUIERDA, la apertura es a la DERECHA.
    servoApertura = SERVO_RIGHT;
  } else {
    // Si el giro principal es a la DERECHA, la apertura es a la IZQUIERDA.
    servoApertura = SERVO_LEFT;
  }
  
  // Imprimir un mensaje de debug para confirmar el ángulo que se va a usar
  Serial.println("   Giro Apertura: Orientando servo a " + String(servoApertura) + "°");

  // Activar motor y orientar servo para apertura
  digitalWrite(MOTOR_PIN, LOW);
  direccion.write(servoApertura);
  
  // Debug opcional
  /*
  Serial.print("Iniciando giro de apertura - Servo: ");
  Serial.print(servoApertura);
  Serial.print(", Sentido vehículo: ");
  Serial.println(sentidoVehiculo);
  */
}
bool procesarGiroApertura() {
  // Si no está activo el giro de apertura, no hacer nada
  if (!giroAperturaActivo) {
    return false; // 
  }
  actualizarGiro();
  // Verificar si ha pasado el tiempo de apertura
  if (millis() - tiempoInicioApertura >= duracionAperturaActual) { // 
    unsigned long tiempoRealTranscurrido = millis() - tiempoInicioApertura;
    Serial.println(" Giro de Apertura completado en " + String(tiempoRealTranscurrido) + " ms. Iniciando giro principal PID.");
   
    // *** PASO 1: Marcar la apertura como finalizada ***
    giroAperturaActivo = false;
    giroAperturaCompletado = true; // Marcar como completado por si se necesita
    
    // Centrar servo momentáneamente para una transición limpia
    //direccion.write(SERVO_CENTER); // 
    delay(50); // Pausa breve

    // *** PASO 2: INICIAR EL GIRO MÁXIMO INMEDIATAMENTE ***
    // Usamos la variable guardada 'direccionGiroAperturaIzquierda' para saber la dirección
    ejecutarGiroPID(GIRO_MAXIMO, direccionGiroAperturaIzquierda);

    // *** PASO 3: Resetear flags de apertura para el próximo ciclo ***
    giroAperturaCompletado = false;
    esGiroApertura = false;
    
    return false; // Se completó la transición de apertura a giro máximo.
  }
  
  return true; // La apertura aún está en proceso. 
}

//EJECUTAR GIRO CON PID ====================
void ejecutarGiroPID(float anguloObjetivo, bool giroIzquierda) {
  enGiro = true;
  giroEnProceso = true;
  cruceEnProceso = true;
  interrupcionCruce = false; // 
  startTime = millis();

  // Resetear y configurar PID
  resetearPID(&pidGiro); // 
  actualizarGiro();
    // Mostrar el valor cada 300 ms
  if (millis() - lastYawPrint >= YAW_INTERVAL) {
        Serial.print("Yaw: ");
        //Serial.print(yawFiltrado);
        Serial.print(anguloZ);
        Serial.println("°");
        lastYawPrint = millis();
    }
  float anguloInicial = anguloZ;
  anguloObjetivoActual = anguloInicial + (giroIzquierda ? anguloObjetivo : -anguloObjetivo);

  // Normalizar ángulo objetivo
  while (anguloObjetivoActual >= 360) anguloObjetivoActual -= 360; // 
  while (anguloObjetivoActual < 0) anguloObjetivoActual += 360;

  // Activar motor para el giro
  digitalWrite(MOTOR_PIN, LOW); // 
  
  // Variables para control del bucle PID
  bool giroCompletado = false;
  int contadorEstable = 0;
  const int CONFIRMACIONES_ESTABLE = 5; // 
  bool giroInterrumpido = false;

  // Bucle principal de control PID
  while (millis() - startTime < timeoutGiro && !giroCompletado && !interrupcionCruce) {
    actualizarGiro(); // 
  // Mostrar el valor cada 300 ms
  if (millis() - lastYawPrint >= YAW_INTERVAL) {
        Serial.print("Yaw: ");
        //Serial.print(yawFiltrado);
        Serial.print(anguloZ);
        Serial.println("°");
        lastYawPrint = millis();
    }
    float salidaPID = calcularPID(&pidGiro, anguloObjetivoActual, anguloZ); // 
    int anguloServo = SERVO_CENTRADO; // 
    
    if (abs(salidaPID) > 0.5) {
      float factorServo = constrain(abs(salidaPID) / 10.0, 0.2, 1.0); // 
      if (salidaPID > 0) {
        anguloServo = SERVO_CENTRADO + (factorServo * 90); // 
      } else {
        anguloServo = SERVO_CENTRADO - (factorServo * 90); // 
      }
      anguloServo = constrain(anguloServo, 0, 180); // 
    }
    
    direccion.write(anguloServo);
    
    float errorActual = anguloObjetivoActual - anguloZ; // 
    if (errorActual > 180) errorActual -= 360;
    if (errorActual < -180) errorActual += 360; // 
    
    if (abs(errorActual) <= TOLERANCIA_PID) {
      contadorEstable++; // 
      if (contadorEstable >= CONFIRMACIONES_ESTABLE) {
        giroCompletado = true; // 
      }
    } else {
      contadorEstable = 0; // 
    }
    
    delay(20);
  }

  direccion.write(SERVO_CENTRADO); // 
  digitalWrite(MOTOR_PIN, LOW);

  enGiro = false;
  giroEnProceso = false;
  cruceEnProceso = false; // 
  tiempoFinGiro = millis();

  // ACTIVAR RETROCESO POST-GIRO
  direccionRetroceso = giroIzquierda ? SERVO_OrientadoObjeto_RIGH : SERVO_OrientadoObjeto_LEFT; // 
  retrocesoOrientadoActivo = true;
  tiempoInicioRetrocesoOrientado = millis();
}
void controlarDireccion(float anguloObjetivoParam) {
  // Determinar si el ángulo está en rango correcto respecto al ángulo objetivo
  bool enRangoCorrecto = estaEnRangoCorrecto(anguloZ, anguloObjetivoParam);
  
  // Debug: mostrar estado del rango cada 500ms
  static unsigned long lastRangeDebug = 0;
  if (millis() - lastRangeDebug > 500) {
    Serial.print("Ángulo actual: ");
    Serial.print(anguloZ);
    Serial.print("° - Objetivo: ");
    Serial.print(anguloObjetivoParam);
    Serial.print("° - En rango: ");
    Serial.print(enRangoCorrecto ? "SÍ" : "NO");
    Serial.print(" - Estado: ");
    
    switch (estadoActualCorreccion) {
      case RECTO: Serial.println("RECTO"); break;
      case CORRIGIENDO_IZQUIERDA: Serial.println("CORRIGIENDO IZQUIERDA"); break;
      case CORRIGIENDO_DERECHA: Serial.println("CORRIGIENDO DERECHA"); break;
      case ESTABILIZANDO: Serial.println("ESTABILIZANDO"); break;
    }
    lastRangeDebug = millis();
  }
  
  switch (estadoActualCorreccion) {
    case RECTO:
      if (!enRangoCorrecto) {
        iniciarCorreccion(anguloObjetivoParam);
      }
      break;
      
    case CORRIGIENDO_IZQUIERDA:
      direccion.write(SERVO_LEFT); 
      if (enRangoCorrecto) {
          finalizarCorreccion();
      } else if (millis() - tiempoInicioCorreccion > TIEMPO_MAX_CORRECCION) {
          // Timeout de corrección - detener y reintentar
          Serial.println("Timeout de corrección - reiniciando");
          finalizarCorreccion();
      } 
      break;

    case CORRIGIENDO_DERECHA:
      direccion.write(SERVO_RIGHT); 
      if (enRangoCorrecto) {
          finalizarCorreccion();
      } else if (millis() - tiempoInicioCorreccion > TIEMPO_MAX_CORRECCION) {
          // Timeout de corrección - detener y reintentar
          Serial.println("Timeout de corrección - reiniciando");
          finalizarCorreccion();
      }
      break;
      
    case ESTABILIZANDO:
      if (millis() - tiempoInicioCorreccion > TIEMPO_ESTABILIZACION) {
        estadoActualCorreccion = RECTO;
        correccionCompletada = true;
        Serial.println("Estabilización completa - modo recto");
      }
      break;
  }
}

void iniciarCorreccion(float anguloObjetivoParam) {
  Serial.print("Iniciando corrección - Ángulo actual: ");
  Serial.print(anguloZ);
  Serial.print("° - Objetivo: ");
  Serial.print(anguloObjetivoParam);
  Serial.println("°");
  
  // LÓGICA PULL-UP: HIGH = detenido, LOW = activado
  // Detener motor principal y activar retroceso
  digitalWrite(MOTOR_PIN, HIGH);  // DETENER motor principal (pull-up)
  digitalWrite(MOTOR_RETROCESO_PIN, LOW); // ACTIVAR retroceso si es necesario
  
  // Determinar dirección de corrección basada en la diferencia angular
  float diferenciaAngular = calcularDiferenciaAngular(anguloZ, anguloObjetivoParam);
  
  if (diferenciaAngular > 0) {
    // El ángulo actual está a la derecha del objetivo -> corregir hacia la izquierda
    direccion.write(SERVO_LEFT);
    estadoActualCorreccion = CORRIGIENDO_IZQUIERDA;
    Serial.print("Corrigiendo hacia la IZQUIERDA (diferencia: +");
    Serial.print(diferenciaAngular);
    Serial.println("°)");
    
  } else {
    // El ángulo actual está a la izquierda del objetivo -> corregir hacia la derecha
    direccion.write(SERVO_RIGHT);
    estadoActualCorreccion = CORRIGIENDO_DERECHA;
    Serial.print("Corrigiendo hacia la DERECHA (diferencia: ");
    Serial.print(diferenciaAngular);
    Serial.println("°)");
  }
  
  tiempoInicioCorreccion = millis();
}

void finalizarCorreccion() {
 /* Serial.print("Corrección completada - Ángulo final: ");
  Serial.print(anguloZ);
  Serial.println("°");
  */
  // Detener retroceso
  digitalWrite(MOTOR_RETROCESO_PIN, HIGH); // DETENER retroceso (pull-up)
  
  // Enderezar dirección
  direccion.write(SERVO_CENTER);
  
  // Período de estabilización
  estadoActualCorreccion = ESTABILIZANDO;
  tiempoInicioCorreccion = millis();
  
  // Pausa para estabilización
  delay(100);
  
  // Solo reactivar motor si la distancia objetivo ya está alcanzada
  float distanciaTrasera = medirDistancia(ULTRASONIDO_TRASERO_TRIG_PIN, ULTRASONIDO_TRASERO_ECHO_PIN);
  bool distanciaAlcanzada = (distanciaTrasera <= DISTANCIA_MINIMA_RETROCESO && distanciaTrasera > 0);
  
  if (distanciaAlcanzada) {
    // Si ya estamos en la distancia objetivo, podemos reactivar el motor de avance
    digitalWrite(MOTOR_PIN, LOW); // ACTIVAR motor principal (pull-up)
    Serial.println("Motor de avance reactivado - distancia objetivo ya alcanzada");
  } else {
    // Si no hemos alcanzado la distancia, mantener el motor de avance detenido
    // El retroceso orientado se encargará de activarlo cuando sea necesario
    digitalWrite(MOTOR_PIN, HIGH); // MANTENER motor principal detenido (pull-up)
    Serial.println("Motor de avance mantenido detenido - falta alcanzar distancia objetivo");
  }
}
// ===== FUNCIONES DE NAVEGACIÓN Y LÍNEAS =====
void calcularAnguloObjetivo() {
  if (sentidoVehiculo == IZQUIERDA) {
    // Giro a la izquierda: 1->0°, 2->90°, 3->180°, 4->270°, 5->0°, 6->90°...
    int ciclo = (contadorLineas - 1) % 4;  // Ajustar para que comience en 1
    switch (ciclo) {
      case 0: anguloObjetivo = 0.0; break;   // contadorLineas = 1, 5, 9...
      case 1: anguloObjetivo = 90.0; break;  // contadorLineas = 2, 6, 10...
      case 2: anguloObjetivo = 180.0; break; // contadorLineas = 3, 7, 11...
      case 3: anguloObjetivo = 270.0; break; // contadorLineas = 4, 8, 12...
    }
  } else {
    // Giro a la derecha: 1->0°, 2->270°, 3->180°, 4->90°, 5->0°, 6->270°...
    int ciclo = (contadorLineas - 1) % 4;  // Ajustar para que comience en 1
    switch (ciclo) {
      case 0: anguloObjetivo = 0.0; break;   // contadorLineas = 1, 5, 9...
      case 1: anguloObjetivo = 270.0; break; // contadorLineas = 2, 6, 10...
      case 2: anguloObjetivo = 180.0; break; // contadorLineas = 3, 7, 11...
      case 3: anguloObjetivo = 90.0; break;  // contadorLineas = 4, 8, 12...
    }
  }
}

void mostrarEstadoActual() {
  Serial.println("=== ESTADO ACTUAL ===");
  Serial.print("Contador de líneas: ");
  Serial.println(contadorLineas);
  Serial.print("Sentido de giro: ");
  Serial.println(sentidoVehiculo == IZQUIERDA ? "IZQUIERDA" : "DERECHA");
  Serial.print("Ángulo objetivo: ");
  Serial.print(anguloObjetivo);
  Serial.println("°");
  Serial.print("Ángulo actual: ");
  Serial.print(anguloZ);
  Serial.println("°");
  Serial.print("Tolerancia: ±");
  Serial.print(TOLERANCIA_ANGULO);
  Serial.println("°");
  Serial.println("====================");
}

bool estaEnRangoCorrecto(float anguloActual, float objetivo) {
  float diferencia = abs(calcularDiferenciaAngular(anguloActual, objetivo));
  return diferencia <= TOLERANCIA_ANGULO;
}

float calcularDiferenciaAngular(float actual, float objetivo) {
  // Calcula la diferencia angular considerando el cruce de 0°/360°
  float diferencia = actual - objetivo;
  
  // Normalizar la diferencia al rango [-180, 180]
  while (diferencia > 180.0) diferencia -= 360.0;
  while (diferencia < -180.0) diferencia += 360.0;
  
  return diferencia;
}
//funcion de detectar color reducida solo para reconfirmar que detecto una linea sin activar nada.
bool chequearLinea() {
  // No chequear si estamos en maniobras de alta prioridad como el retroceso final.
  if (retrocesoOrientadoActivo) {
    return false;
  }

  uint16_t r, g, b, c;
  double hue, saturation, value;

  tcs.getRawData(&r, &g, &b, &c);
  if (c > 0) {
    // Se realiza la misma conversión de color que en la función principal
    float r_norm = (float)r / c; 
    float g_norm = (float)g / c; 
    float b_norm = (float)b / c; 

    uint8_t r_byte = (uint8_t)(r_norm * 255.0); 
    uint8_t g_byte = (uint8_t)(g_norm * 255.0); 
    uint8_t b_byte = (uint8_t)(b_norm * 255.0); 

    ColorConverter::RgbToHsv(r_byte, g_byte, b_byte, hue, saturation, value); 

    bool esBlanco = (value > 0.85 && saturation < 0.15); 
    float proporcionAzul = b_norm / (r_norm + g_norm + 0.001); 
    // Detección azul (usando las mismas condiciones que en detectarColor)
    if (hue >= HUE_AZUL_MIN && hue <= HUE_AZUL_MAX &&
      saturation > SATURACION_AZUL_MIN &&
      value > VALOR_AZUL_MIN && value < VALOR_AZUL_MAX &&
      proporcionAzul > PROPORCION_AZUL_MIN &&
      !esBlanco) {
      if (confirmarColor(HUE_AZUL_MIN, HUE_AZUL_MAX, SATURACION_AZUL_MIN, VALOR_AZUL_MIN, VALOR_AZUL_MAX, CONFIRMAR_AZUL_TOLERANCIA, CONFIRMAR_AZUL_DELAY)) {
   
        return true; // ¡Línea azul encontrada!
      }
    }
      else if (((hue >= HUE_NARANJA_MIN && hue <= HUE_NARANJA_MAX)) &&
            saturation > SATURACION_NARANJA_MIN &&
            value > VALOR_NARANJA_MIN && value < VALOR_NARANJA_MAX &&
            !esBlanco) {
        if (confirmarColor(HUE_NARANJA_MIN, HUE_NARANJA_MAX, SATURACION_NARANJA_MIN, VALOR_NARANJA_MIN, VALOR_NARANJA_MAX, CONFIRMAR_NARANJA_TOLERANCIA, CONFIRMAR_NARANJA_DELAY)) {
        return true; // ¡Línea naranja encontrada!
      }
    }
  }
  // Si llegamos hasta aquí, no se detectó nada.
  return false;
}

// ===== FUNCIÓN PRINCIPAL DE INTEGRACIÓN =====
void realizarGiro(bool giroIzquierda) {
  // RESETEAR VARIABLES DE ESTADO ANTES DE INICIAR
  correccionCompletada = false;
  estadoActualCorreccion = RECTO;
  
  // RESETEAR variables de compensación
  tiempoTotalRetroceso = 0;
  huboRetrocesoEnCorreccion = false;

  // ===================== =====================
  // PASO 1: Calcular el ángulo de giro dinámico y el tiempo de retroceso necesario AL PRINCIPIO.
  // Esto nos permite saber de antemano si se requerirá un retroceso por giro cerrado.
  float anguloObjetivoGiro = calcularAnguloGiroDinamico(giroIzquierda);
//****************************
  // Forzar GIRO DE APERTURA usando la variable de memoria "ultimoCarrilActivo",
  // ya que "carriles.carrilObjetivo" se resetea antes de llamar a esta función.
  bool forzarApertura = (ultimoCarrilActivo == 1 && sentidoVehiculo == DERECHA) ||
                          (ultimoCarrilActivo == 2 && sentidoVehiculo == IZQUIERDA);

  if (forzarApertura) {
    esGiroApertura = true; // Anula la decisión basada en distancia.
    Serial.println(" FORZANDO GIRO DE APERTURA por condición de carril/sentido memorizada.");
  }
//****************************
  unsigned long tiempoRetrocesoAdicional = 0;
  bool retrocesoGestionado = false; // Flag para controlar que solo se ejecute una lógica de retroceso.

  if (anguloObjetivoGiro == GIRO_MAXIMO) {
    tiempoRetrocesoAdicional = TIEMPO_RETROCESO_MAXIMO;
  } else if (anguloObjetivoGiro == GIRO_ALTO) {
    tiempoRetrocesoAdicional = TIEMPO_RETROCESO_ALTO;
  } else if (anguloObjetivoGiro == GIRO_MEDIO_ALTO) {
    tiempoRetrocesoAdicional = TIEMPO_RETROCESO_MEDIO_ALTO;
  }
  // ==========================================

  // PASO 2: Actualizar ángulo objetivo de navegación basado en contadorLineas actual
  calcularAnguloObjetivo();

  // PASO 3: Verificar si estamos en el rango correcto
  bool enRangoCorrecto = estaEnRangoCorrecto(anguloZ, anguloObjetivo);

  if (!enRangoCorrecto) {
    Serial.println("Ángulo fuera de rango - iniciando corrección antes del giro");
    Serial.print(" > Actual: "); Serial.print(anguloZ);
    Serial.print(" | Objetivo: "); Serial.println(anguloObjetivo);
    
    tiempoInicioCorreccion = millis();
    unsigned long tiempoInicioRetrocesoLocal = 0;
    bool retrocediendoAhora = false;
    
    // Bucle de corrección hasta que el ángulo esté en rango
    while (!estaEnRangoCorrecto(anguloZ, anguloObjetivo)) {
      actualizarGiro();
      bool motorRetrocesoActivo = (digitalRead(MOTOR_RETROCESO_PIN) == LOW);
      if (motorRetrocesoActivo && !retrocediendoAhora) {
        tiempoInicioRetrocesoLocal = millis();
        retrocediendoAhora = true;
        huboRetrocesoEnCorreccion = true;
      } else if (!motorRetrocesoActivo && retrocediendoAhora) {
        tiempoTotalRetroceso += (millis() - tiempoInicioRetrocesoLocal);
        retrocediendoAhora = false;
      }
      
      controlarDireccion(anguloObjetivo);
      delay(10);
      
      if (millis() - tiempoInicioCorreccion > TIEMPO_MAX_CORRECCION) {
        Serial.println("Timeout de corrección - procediendo con giro");
        if (retrocediendoAhora) {
          tiempoTotalRetroceso += (millis() - tiempoInicioRetrocesoLocal);
        }
        break;
      }
    }
    
    if (retrocediendoAhora) {
      tiempoTotalRetroceso += (millis() - tiempoInicioRetrocesoLocal);
    }
    
    finalizarCorreccion();
    delay(100);

    // =====================  LÓGICA DE DECISIÓN =====================
    // Ahora decidimos qué hacer: un retroceso neto o un avance compensatorio. NUNCA ambos.
    
    if (tiempoRetrocesoAdicional > 0) {
      // CASO A: Se necesita un retroceso pre-giro.
      // Calculamos cuánto tiempo de retroceso nos falta después de la corrección.
      long tiempoNetoRetroceso = tiempoRetrocesoAdicional - tiempoTotalRetroceso;

      if (tiempoNetoRetroceso > 0) {
        Serial.print("GIRO CERRADO: Realizando retroceso neto adicional por ");
        Serial.print(tiempoNetoRetroceso);
        Serial.println(" ms.");
        
        digitalWrite(MOTOR_PIN, HIGH);
        direccion.write(SERVO_CENTER);
        digitalWrite(MOTOR_RETROCESO_PIN, LOW);
        delay(tiempoNetoRetroceso);
        digitalWrite(MOTOR_RETROCESO_PIN, HIGH);
        delay(100);
      } else {
        Serial.println("GIRO CERRADO: El retroceso de corrección fue suficiente. Omitiendo retroceso neto.");
      }
      retrocesoGestionado = true; // Marcamos que ya manejamos el retroceso.
    }
    else if (huboRetrocesoEnCorreccion && tiempoTotalRetroceso > 0) {
      // CASO B: NO se necesita retroceso pre-giro, PERO sí hubo retroceso por corrección.
      // Solo en este caso, hacemos el avance compensatorio.
      unsigned long tiempoAvanceCompensatorio = tiempoTotalRetroceso/2;
      Serial.print("Iniciando avance compensatorio por hasta ");
      Serial.print(tiempoAvanceCompensatorio);
      Serial.println(" ms, buscando línea activamente...");
      
      int servoDireccion = giroIzquierda ? SERVO_OrientadoObjeto_RIGH : SERVO_OrientadoObjeto_LEFT;
      
      digitalWrite(MOTOR_RETROCESO_PIN, HIGH);
      digitalWrite(MOTOR_PIN, LOW); 
      direccion.write(servoDireccion);
      
      unsigned long tiempoInicioAvance = millis();
      bool lineaReDetectada = false;
      while ((millis() - tiempoInicioAvance) < tiempoAvanceCompensatorio) {
        if (chequearLinea()) {
          lineaReDetectada = true;
          break;
        }
        delay(5);
      }
      
      digitalWrite(MOTOR_PIN, HIGH);
      direccion.write(SERVO_CENTER); 

      if (lineaReDetectada) {
        Serial.println("Avance compensatorio interrumpido por re-detección de línea.");
      } else {
        Serial.println("Avance compensatorio completado por tiempo.");
      }
      delay(50);
    }
    // ===================== FIN DE LA LÓGICA DE DECISIÓN =====================

  } else {
    Serial.println("PRE-GIRO: Ángulo ya en rango. Omitiendo corrección.");
    Serial.print(" > Actual: "); Serial.print(anguloZ);
    Serial.print(" | Objetivo: "); Serial.println(anguloObjetivo);
  }
  
  // PASO 4: Ejecutar la lógica de giro.

  // Si es un giro de apertura (suave), la lógica no cambia.
  if (esGiroApertura) {
     if (anguloObjetivoGiro == GIRO_MEDIO) {
      duracionAperturaActual = DURACION_APERTURA_MINIMO;
    } else if (anguloObjetivoGiro == GIRO_MINIMO) {
      duracionAperturaActual = DURACION_APERTURA_MINIMO;
    } else if (anguloObjetivoGiro == GIRO_BAJO) {
      duracionAperturaActual = DURACION_APERTURA_BAJO;
    } else if (anguloObjetivoGiro == GIRO_MEDIO_BAJO) {
      duracionAperturaActual = DURACION_APERTURA_MEDIO_BAJO;
    }
    
    if (!giroAperturaCompletado && !giroAperturaActivo) {
       realizarGiroApertura(giroIzquierda);
    }
    return;
  }

  // ===================== LÓGICA DE RETROCESO MODIFICADA =====================
  // Este bloque ahora solo se ejecuta si el ángulo ESTABA BIEN desde el principio
  // y, por lo tanto, la lógica de unificación no se activó.
  if (tiempoRetrocesoAdicional > 0 && !retrocesoGestionado) {
    Serial.print("GIRO CERRADO (sin corrección previa): Retrocediendo recto por ");
    Serial.print(tiempoRetrocesoAdicional);
    Serial.println(" ms.");

    digitalWrite(MOTOR_PIN, HIGH);
    direccion.write(SERVO_CENTER);
    digitalWrite(MOTOR_RETROCESO_PIN, LOW);
    delay(tiempoRetrocesoAdicional);
    digitalWrite(MOTOR_RETROCESO_PIN, HIGH);
    delay(100);
  }
  // ===================== FIN =====================

  // Finalmente, ejecutar el giro PID normal.
  ejecutarGiroPID(anguloObjetivoGiro, giroIzquierda);
}
float calcularAnguloRetroceso() {
  float anguloRetroceso;
  
  if (sentidoVehiculo == IZQUIERDA) {
    // Giro a la izquierda: 1->90°, 2->180°, 3->270°, 4->0°, 5->90°...
    int ciclo = (contadorLineas - 1) % 4;
    switch (ciclo) {
      case 0: anguloRetroceso = 90.0; break;   // contadorLineas = 1, 5, 9...
      case 1: anguloRetroceso = 180.0; break;  // contadorLineas = 2, 6, 10...
      case 2: anguloRetroceso = 270.0; break;  // contadorLineas = 3, 7, 11...
      case 3: anguloRetroceso = 0.0; break;    // contadorLineas = 4, 8, 12...
    }
  } else {
    // Giro a la derecha: 1->270°, 2->180°, 3->90°, 4->0°, 5->270°...
    int ciclo = (contadorLineas - 1) % 4;
    switch (ciclo) {
      case 0: anguloRetroceso = 270.0; break;  // contadorLineas = 1, 5, 9...
      case 1: anguloRetroceso = 180.0; break;  // contadorLineas = 2, 6, 10...
      case 2: anguloRetroceso = 90.0; break;   // contadorLineas = 3, 7, 11...
      case 3: anguloRetroceso = 0.0; break;    // contadorLineas = 4, 8, 12...
    }
  }
  
  return anguloRetroceso;
}
// Función auxiliar para llamar cuando se detecte una nueva línea
void incrementarContadorRecalibracion() {
  contadorLineasRecalibracion++;
  Serial.print("Líneas desde recalibración: ");
  Serial.println(contadorLineasRecalibracion);
}

// ============== FUNCIÓN DE RETROCESO POST-GIRO ORIENTADO ==============
bool procesarRetrocesoPostGiro() {
  actualizarGiro();

  // Si el retroceso no está activo, no hacer nada
  if (!retrocesoOrientadoActivo) {
    return false;
  }

  unsigned long tiempoTranscurrido = millis() - tiempoInicioRetrocesoOrientado;

  // PASO 1: Calcular el ángulo objetivo para el retroceso orientado
  float anguloObjetivoRetroceso = calcularAnguloRetroceso();

  // PASO 2: Verificar si estamos en el rango correcto
  bool enRangoCorrecto = estaEnRangoCorrecto(anguloZ, anguloObjetivoRetroceso);

  if (!enRangoCorrecto) {
    Serial.println("POST-GIRO: Ángulo fuera de rango. Iniciando corrección...");
    Serial.print(" > Actual: "); Serial.print(anguloZ);
    Serial.print(" | Objetivo: "); Serial.println(anguloObjetivoRetroceso);
     // RESETEAR tiempo de inicio para nueva corrección
    tiempoInicioCorreccion = millis();
    // Ejecutar corrección hasta que esté en rango
  
  unsigned long tiempoInicioCorreccionPostGiro = millis(); // Timer local para el timeout
  bool necesitaCorreccion = !estaEnRangoCorrecto(anguloZ, anguloObjetivoRetroceso);

  if (necesitaCorreccion) {
      Serial.println("POST-GIRO: Iniciando corrección de ángulo...");
  }

  while (!estaEnRangoCorrecto(anguloZ, anguloObjetivoRetroceso)) {
      actualizarGiro();

      // Lógica de corrección directa y simple
      float diferencia = calcularDiferenciaAngular(anguloZ, anguloObjetivoRetroceso);
      
      if (diferencia > 0) {
          // El ángulo actual es mayor que el objetivo, corregir a la izquierda
          // Usamos 170 en lugar de 180 para un giro un poco menos brusco
          direccion.write(170); 
      } else {
          // El ángulo actual es menor que el objetivo, corregir a la derecha
          // Usamos 10 en lugar de 0 para un giro un poco menos brusco
          direccion.write(10);
      }
      
      // Mantener motores en retroceso
      digitalWrite(MOTOR_RETROCESO_PIN, LOW);
      digitalWrite(MOTOR_PIN, HIGH);
      delay(20); // Delay más corto para una respuesta más rápida

      // Timeout de seguridad
      if (millis() - tiempoInicioCorreccionPostGiro > TIEMPO_MAX_CORRECCION) { // Usamos un timeout simple
          Serial.println("Timeout de corrección en POST-GIRO. Continuando...");
          break;
      }
  }

  if (necesitaCorreccion) {
      Serial.println("POST-GIRO: Corrección de ángulo finalizada.");
  }

  // Una vez que el bucle termina (porque el ángulo es correcto o por timeout),
  // estabilizamos y centramos las ruedas.
  digitalWrite(MOTOR_RETROCESO_PIN, HIGH); // Detener retroceso
  direccion.write(SERVO_CENTER);
  delay(250); // Pausa para estabilizar
  } else {
    Serial.println("POST-GIRO: Ángulo ya en rango. Omitiendo corrección y centrando ruedas.");
    Serial.print(" > Actual: "); Serial.print(anguloZ);
    Serial.print(" | Objetivo: "); Serial.println(anguloObjetivoRetroceso);
  }

  // PASO 4: Verificar distancia trasera después de la recalibración
  float distanciaTrasera = medirDistancia(ULTRASONIDO_TRASERO_TRIG_PIN, ULTRASONIDO_TRASERO_ECHO_PIN);

  // PASO 5: Verificar si ya estamos en la distancia objetivo después de la recalibración
  bool distanciaAlcanzada = (distanciaTrasera <= DISTANCIA_MINIMA_RETROCESO && distanciaTrasera > 0);

  if (distanciaAlcanzada) {
    Serial.println("Distancia objetivo alcanzada después de recalibración");
    // Recalibrar estableciendo el ángulo objetivo como nuevo ángulo base
  
    finalizarRetrocesoPostGiro();
    return false;
  }

  // PASO 6: Verificar timeouts de seguridad
  bool timeoutSeguridad = (tiempoTranscurrido >= TIMEOUT_RETROCESO_MAXIMO);
  bool emergenciaTimeout = (tiempoTranscurrido >= 7000);

  if (timeoutSeguridad || emergenciaTimeout) {
    Serial.println("Timeout de seguridad alcanzado");
    finalizarRetrocesoPostGiro();
    return false;
  }

  // PASO 7: Continuar con el retroceso orientado
  // Después de la recalibración, el ángulo base es ahora el ángulo objetivo
  // Por lo tanto, mantener dirección centrada para retroceso recto
  direccion.write(SERVO_CENTER);
  digitalWrite(MOTOR_RETROCESO_PIN, LOW);  // Activar retroceso
  digitalWrite(MOTOR_PIN, HIGH);           // Asegurar que avance esté desactivado

  // Debug para mostrar el progreso del retroceso
  static unsigned long lastRetrocesoDebug = 0;
  if (millis() - lastRetrocesoDebug > 1000) {
    Serial.print("Retroceso orientado activo - Distancia trasera: ");
    Serial.print(distanciaTrasera);
    Serial.print(" cm - Ángulo actual: ");
    Serial.print(anguloZ);
    Serial.print("° - Ángulo objetivo: ");
    Serial.print(anguloObjetivoRetroceso);
    Serial.println("°");
    lastRetrocesoDebug = millis();
  }

  delay(200);
  return true; // Indica que el retroceso sigue activo
}
// ============== FUNCIÓN PARA FINALIZAR RETROCESO ==============
void finalizarRetrocesoPostGiro() {
  // Paso 1: Detener motores inmediatamente
  digitalWrite(MOTOR_RETROCESO_PIN, HIGH);
  digitalWrite(MOTOR_PIN, HIGH);
  
  // Paso 2: Centrar dirección
  direccion.write(SERVO_CENTER);
  //delay(5000);
  
  // Paso 3: Pausa para estabilizar sistema
  delay(100);
  
  // Paso 4: Resetear flags de estado completamente
  retrocesoOrientadoActivo = false;
  enGiro = false;
  giroEnProceso = false;
  cruceEnProceso = false;
  
  // Paso 5: Resetear variables de tiempo
  tiempoInicioRetrocesoOrientado = 0;
  // Paso 6: Verificar si el giro era para el estacionamiento final
  if (giroParaEstacionamiento) {
    // Si era el último giro, proceder al estado de detención
    estadoActual = ESTADO_DETENCION_FINAL; 
    tiempoInicioDetencionFinal = millis(); 
    giroParaEstacionamiento = false; 
  } else {
    // CAMBIO CLAVE: Si no es el giro final, transicionar a ESPERANDO_COMANDO_CARRIL
    estadoActual = ESPERANDO_COMANDO_CARRIL;
  }

  // Resetear el PID de centrado
  resetearPIDCentrado(&pidCentrado);

  // El motor se activará cuando comience un nuevo estado de carril
  digitalWrite(MOTOR_PIN, HIGH); // Apagado por defecto en espera
  
  // Resetear variables de apertura
  giroAperturaActivo = false;
  giroAperturaCompletado = false;
  esGiroApertura = false;
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
// ==========  ==========

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
  // VERIFICACIÓN PERIÓDICA (cada 3 segundos)
  /*if (tiempoActual - ultimaVerificacion > 5000) {
    if (tiempoActual - ultimaLecturaValida > 2000) { // 1.5 segundos sin lecturas válidas
      Serial.println("Sensor sin respuesta - Reiniciando");
      reiniciarQMC5883L();
      lecturasFallidas = 0;
      valoresRepetidos = 0;
      anguloAnterior = -999;
      ultimaLecturaValida = tiempoActual;
    }
    ultimaVerificacion = tiempoActual;
  }*/

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
  
    Serial.print("angulo magne");
    Serial.println(anguloMagnetometro);
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
  const int muestras = 500;//500 // Número de muestras para promediar
  float sumaGz = 0.0;
  float sumaGx = 0.0;
  float sumaGy = 0.0;

  Serial.println("Calibrando giroscopio... Mantén el sensor inmóvil.");
  delay(500); // Dar un segundo para estabilizar el sensor

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
void mostrarInformacion() {
  if (millis() - lastYawPrint >= YAW_INTERVAL) {
    Serial.print("Yaw: ");
    Serial.print(anguloZ, 1);
    Serial.print(" | Objetivo: ");
    Serial.print(anguloObjetivo);
    Serial.print("° | Estado: ");
    
    switch (estadoActualCorreccion) {
      case RECTO: Serial.println("RECTO"); break;
      case CORRIGIENDO_IZQUIERDA: Serial.println("CORRIGIENDO IZQUIERDA"); break;
      case CORRIGIENDO_DERECHA: Serial.println("CORRIGIENDO DERECHA"); break;
      case ESTABILIZANDO: Serial.println("ESTABILIZANDO"); break;
    }
    
    lastYawPrint = millis();
  }
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

    // MODIFICACIÓN: Solo aplicar espera si ya se detectó al menos una línea
    bool puedeDetectar = (contadorLineas == 0) || (millis() - tiempoUltimaDeteccion > tiempoEsperaDeteccion);
    
    if (puedeDetectar) {
      bool esBlanco = (value > 0.85 && saturation < 0.15);
      float proporcionAzul = b_norm / (r_norm + g_norm + 0.001);
      
      bool azulDetectado = false;
      bool naranjaDetectado = false;
        
      // Detección azul
     // Detección azul
    if (hue >= HUE_AZUL_MIN && hue <= HUE_AZUL_MAX &&
        saturation > SATURACION_AZUL_MIN &&
        value > VALOR_AZUL_MIN && value < VALOR_AZUL_MAX &&
        proporcionAzul > PROPORCION_AZUL_MIN &&
        !esBlanco) {
        if (confirmarColor(HUE_AZUL_MIN, HUE_AZUL_MAX, SATURACION_AZUL_MIN, VALOR_AZUL_MIN, VALOR_AZUL_MAX, CONFIRMAR_AZUL_TOLERANCIA, CONFIRMAR_AZUL_DELAY)) {
             
          azulDetectado = true;
          
          pausarCentrado = true;
          tiempoInicioDeteccion = millis();
          
          if (primeraDeteccion) {
            sentidoVehiculo = IZQUIERDA;
            primeraDeteccion = false;
          }
        }
      }
      // Detección naranja
      else if (((hue >= HUE_NARANJA_MIN && hue <= HUE_NARANJA_MAX)) &&
          saturation > SATURACION_NARANJA_MIN &&
          value > VALOR_NARANJA_MIN && value < VALOR_NARANJA_MAX &&
          !esBlanco) {
        if (confirmarColor(HUE_NARANJA_MIN, HUE_NARANJA_MAX, SATURACION_NARANJA_MIN, VALOR_NARANJA_MIN, VALOR_NARANJA_MAX, CONFIRMAR_NARANJA_TOLERANCIA, CONFIRMAR_NARANJA_DELAY)) {
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
        
        // Ejecutar giro normal según el sentido del vehículo
        if (azulDetectado && sentidoVehiculo == IZQUIERDA) {
          //  LÓGICA DE INTERRUPCIÓN: Si estamos en una maniobra de carril, la reseteamos
          if (estadoActual == GIRANDO_HACIA_CARRIL || estadoActual == ENDEREZANDO_CARRIL || estadoActual == MANTENIENDO_CARRIL) {
            resetSistema(true); // Resetea la lógica de carriles conservando el ángulo
          }
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
            // Se activa la bandera para indicar que este es el último giro.
            giroParaEstacionamiento = true;
            resetearPIDCentrado(&pidCentrado);
            // Se realiza el giro a la izquierda (true) correspondiente a la línea azul.
            realizarGiro(true);
          } else {

            resetearPIDCentrado(&pidCentrado);
            realizarGiro(true); // Giro a la izquierda
          }
          
          digitalWrite(LED_PIN, LOW);
          tiempoUltimaDeteccion = millis(); // Actualizar tiempo después de procesar la detección
        }
        else if (naranjaDetectado && sentidoVehiculo == DERECHA) {
          //  LÓGICA DE INTERRUPCIÓN: Si estamos en una maniobra de carril, la reseteamos
          if (estadoActual == GIRANDO_HACIA_CARRIL || estadoActual == ENDEREZANDO_CARRIL || estadoActual == MANTENIENDO_CARRIL) {
            resetSistema(true); // Resetea la lógica de carriles conservando el ángulo
          }
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
            // Se activa la bandera para indicar que este es el último giro.
            giroParaEstacionamiento = true;
            resetearPIDCentrado(&pidCentrado);
            // Se realiza el giro a la derecha (false) correspondiente a la línea naranja.
            realizarGiro(false);
          }else {
            resetearPIDCentrado(&pidCentrado);
            realizarGiro(false); // Giro a la derecha
          }
          
          digitalWrite(LED_PIN, LOW);
          tiempoUltimaDeteccion = millis(); // Actualizar tiempo después de procesar la detección
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

// ==================== FUNCIÓN LEER SENSOR SHARP ====================
float leerSensorSharp(int pin) {
  // Leer valor analógico del sensor Sharp
  int valorAnalogico = analogRead(pin);
  
  // Convertir a voltaje (asumiendo 3.3V de referencia del ESP32)
  float voltaje = valorAnalogico * (3.3 / 4095.0);
  
  // Conversión aproximada para sensor Sharp GP2Y0A21YK0F (10–80 cm)
  // La fórmula puede requerir ajuste fino con pruebas reales
  if (voltaje < 0.3) return 999; // Fuera de rango útil
  
  float distancia = 27.728 / (voltaje - 0.1696);
  
  // Limitar rango válido
  if (distancia < 10 || distancia > 80) return 999;
  
  return distancia;
}
// ==================== FUNCIÓN VERIFICAR OBSTÁCULO DIAGONAL ====================

bool verificarObstaculoDiagonal() {
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
  //estadoActual = ESTADO_NORMAL;
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
    
    Serial.println("ACTIVANDO RETROCESO DE EMERGENCIA");
    
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
    Serial.println("TIMEOUT DE RETROCESO - FINALIZANDO FORZADAMENTE");
    
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
     Serial.println("Obstáculo aún presente - Reiniciando ciclo de retroceso");
      
      // RESETEAR PARA NUEVO CICLO
      motorRetrocesoConfigurado = false;  // PERMITIR NUEVA CONFIGURACIÓN
      tiempoInicioRetroceso = millis();   //  TIEMPO DE INICIO
      
      // Mantener retrocesoPorEmergencia = true para continuar
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
  //analogWrite(BUZZER_PIN, 160);
  //delay(100);
 // analogWrite(BUZZER_PIN, 0);
  direccion.write(SERVO_CENTRADO);
  Serial.begin(115200);
  Serial.setTimeout(1);
  Wire.begin(I2C_SDA, I2C_SCL);

  // Configurar pines de sensores originales
  pinMode(TRIG_DERECHO, OUTPUT);
  pinMode(ECHO_DERECHO, INPUT);
  pinMode(TRIG_IZQUIERDO, OUTPUT);
  pinMode(ECHO_IZQUIERDO, INPUT);
  
  // Configurar pines de sensor frontal de emergencia
  pinMode(TRIG_FRONTAL, OUTPUT);
  pinMode(ECHO_FRONTAL, INPUT);

  pinMode(ULTRASONIDO_TRASERO_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIDO_TRASERO_ECHO_PIN, INPUT);

  // Configurar pin del interruptor con pull-up interno
  pinMode(INTERRUPTOR_PIN, INPUT_PULLUP);
  
  // Inicializar sensor de color temprano para calibración
  if (!tcs.begin()) {
    while (1);
  }
  //*****************inicializacion magnetometro **************
   Serial.println("QMC5883L - Lectura de Ángulos para Vehículo ESP32");
  Serial.println("Iniciando sensor magnetometro...");
  
  // Configurar el magnetómetro QMC5883L
  Wire.beginTransmission(QMC5883L_ADDRESS);
  Wire.write(QMC5883L_SET_RESET_PERIOD);
  Wire.write(0x01); // Período de reinicio
  Wire.endTransmission();
  
  Wire.beginTransmission(QMC5883L_ADDRESS);
  Wire.write(QMC5883L_CONTROL_2);
  Wire.write(0x00); // Sin interrupciones
  Wire.endTransmission();
  
  Wire.beginTransmission(QMC5883L_ADDRESS);
  Wire.write(QMC5883L_CONTROL_1);
  Wire.write(0x1D); // Modo continuo, ODR 200Hz, RNG 8G, OSR 512
  Wire.endTransmission();
  
  delay(100);
  
  // Verificar si hay valores de calibración
  if(xOffset == 0.0 && yOffset == 0.0) {
    Serial.println("⚠️ ADVERTENCIA: Usando valores por defecto sin calibración");
    Serial.println("Para mejor precisión, ejecuta primero el código de calibración");
  } else {
    Serial.println("✅ Usando valores de calibración personalizados");
  }
  
  Serial.println("Formato: Ángulo | Dirección | [Datos internos]");
  Serial.println("----------------------------------------------");
  //*******************************

 mpu.initialize();
  while (!mpu.testConnection()) {
    Serial.println("Fallo la conexión con MPU6050. Reintentando...");
    delay(1000);
  }

  //sistemna con magne y giros. solo el buffer
  inicializarSistemaHibrido();

  direccion.attach(SERVO_PIN);
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
    direccion.write(SERVO_CENTRADO);
  } else {
    // Si está en HIGH, esperar a que se active
    esperarInterruptor();
  }
  
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
    // Procesar giro de apertura si está activo
  if (giroAperturaActivo) {
    procesarGiroApertura();
    return; // No ejecutar nada más mientras se procesa la apertura
  }
    // EJECUTAR SIEMPRE (independiente de comandos seriales)
  actualizarGiro();

    // Detección de color se ejecuta en estados de navegación, no durante maniobras de carril
  // (La interrupción ya se maneja dentro de los casos del switch)
  if (estadoActual == ESTADO_NORMAL) {
      detectarColor();
  }
 /*if (estadoActual != ESTADO_DETENCION_FINAL){
    detectarColor();
  }*/
   // MÁQUINA DE ESTADOS - Solo ejecutar si no hay procesos de alta prioridad
  switch(estadoActual) {
    case ESTADO_NORMAL:
      //centrarVehiculo();
       if (!enGiro && 
          prioridadActual == PRIORIDAD_NORMAL && 
          !esquiveDiagonalActivo && 
          !retrocesoPorEmergencia) {
        centrarVehiculo();
        //Serial.println("normal");
          }
      break;
         case ESPERANDO_COMANDO_CARRIL:
      digitalWrite(MOTOR_PIN, HIGH); // Motor apagado
      // Monitorear puerto serie para comandos de carril
      if (Serial.available() > 0) {
        String comando = Serial.readStringUntil('\n');
        comando.trim();
        comando.toUpperCase();
        if (comando.startsWith("C")) {
            procesarComandoCarril(comando);
        }
      }
      break;

     case EVALUANDO_POSICION:
    case GIRANDO_HACIA_CARRIL:
    case ENDEREZANDO_CARRIL:
    case MANTENIENDO_CARRIL:
      { 
        EstadoSistema estadoPrevio = estadoActual; // Guardar el estado actual

        // Durante todos los estados de carril, permitir interrupción por línea
        detectarColor(); // 

        // Si detectarColor cambió el estado, significa que encontró una línea
        // y comenzó un giro. Debemos salir del switch INMEDIATAMENTE.
        if (estadoActual != estadoPrevio) {
            break; 
        }

        // Si no hubo cambio de estado, continuar con la lógica de carriles
        if (millis() - ultimaActualizacionf >= INTERVALO_CONTROLf) {
            float anguloServo = calcularControlCarriles(); // 
            if (anguloServo != -1) {
                direccion.write((int)anguloServo); // 
                
                if (millis() - ultimoReportef > INTERVALO_REPORTEf) {
                    Serial.print("Servo: "); // 
                    Serial.print((int)anguloServo);
                    Serial.print(" | Carril: ");
                    Serial.print(carriles.carrilObjetivo);
                    Serial.print(" | Lado: ");
                    Serial.print(controlLateral.ladoControl);
                    Serial.print(" | Err Dist: ");
                    Serial.print(controlLateral.errorDistanciaAnterior, 2); // 
                    Serial.print(" | Err Ang: "); // 
                    float errorAngulo = controlLateral.anguloObjetivo - anguloZ;
                    if (errorAngulo > 180.0) errorAngulo -= 360.0; // 
                    else if (errorAngulo < -180.0) errorAngulo += 360.0;
                    Serial.print(errorAngulo, 1);
                    Serial.println();
                    ultimoReportef = millis(); // 
                }
            }
            digitalWrite(MOTOR_PIN, LOW); // 
            ultimaActualizacionf = millis();
        }
      } 
      break;
   /* float anguloServo = calcularControlCarriles();
    if (anguloServo != -1) {
        direccion.write((int)anguloServo);

        // ====== INICIO: NUEVO MONITOR DE DEPURACIÓN ======
        // Este bloque imprimirá solo cuando esté en el estado de mantenimiento de carril
        if (millis() - ultimoReportef > INTERVALO_REPORTEf) {
            Serial.print("Servo: ");
            Serial.print((int)anguloServo);
            Serial.print(" | Carril: ");
            Serial.print(carriles.carrilObjetivo);
            Serial.print(" | Lado: ");
            Serial.print(controlLateral.ladoControl);
            Serial.print(" | Err Dist: ");
            Serial.print(controlLateral.errorDistanciaAnterior, 2); // Muestra el último error de distancia calculado
            Serial.print(" | Err Ang: ");
            float errorAngulo = controlLateral.anguloObjetivo - anguloZ;
            if (errorAngulo > 180.0) errorAngulo -= 360.0;
            else if (errorAngulo < -180.0) errorAngulo += 360.0;
            Serial.print(errorAngulo, 1);
            Serial.println();
            ultimoReportef = millis();
        }
        // ====== FIN: NUEVO MONITOR DE DEPURACIÓN ======
    }
    digitalWrite(MOTOR_PIN, LOW);
    ultimaActualizacionf = millis();
    }
      break;
*/
    case ESTADO_DETENCION_FINAL:
     if (contadorEntradaEstadoFinal >= 1){
      /*  analogWrite(BUZZER_PIN, 255);
        delay(50);
        analogWrite(BUZZER_PIN, 0);
        delay(50);
        analogWrite(BUZZER_PIN, 255);
        delay(50);
        analogWrite(BUZZER_PIN, 0);*/
        contadorEntradaEstadoFinal += 1; 
      }
      // Avanzar en línea recta durante 2.5 segundos
      if (millis() - tiempoInicioDetencionFinal < 1500) {
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

    
  // MÁXIMA PRIORIDAD: Retroceso de emergencia (SOLO si NO está girando)
  if (prioridadActual == PRIORIDAD_RETROCESO && !giroEnProceso && !retrocesoOrientadoActivo && estadoActual !=ESTADO_DETENCION_FINAL){
    if (retrocesoPorEmergencia) {
      //procesarRetrocesoEmergencia();
    }
    return;
  }
  
  // ALTA PRIORIDAD: Retroceso post-giro
  if (procesarRetrocesoPostGiro()) {
    // El retroceso está activo, no ejecutar nada más
    return;
  }
    // Verificar obstáculos diagonales SOLO si NO está girando
    //if (!esquiveDiagonalActivo && !giroEnProceso && verificarObstaculoDiagonal()) {
      // Se activó esquive diagonal
    //}

    // Procesar esquive diagonal si está activo Y NO está girando
    if (esquiveDiagonalActivo && !giroEnProceso) {
      procesarEsquiveDiagonal();
    }
   
  // Mostrar el valor cada 300 ms
  if (millis() - lastYawPrint >= YAW_INTERVAL) {
        Serial.print("Yaw: ");
        //Serial.print(yawFiltrado);
        Serial.print(anguloZ);
        Serial.println(anguloZ);
        Serial.print("°  ");
        lastYawPrint = millis();
        Serial.println("Estado actual: " + obtenerNombreEstado(estadoActual));
        
    }

  delay(20);
}



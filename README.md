# Prometheus Team

Somos un equipo venezolano que representa con orgullo a la **Universidad Politécnica Territorial José Félix Ribas (UPTJFR)**. Nuestro compromiso es la innovación y el aprendizaje continuo en el campo de la robótica autónoma.

---

##  Índice

* [Contenido del Repositorio](#contenido-del-repositorio)
* [Introducción al Proyecto](#introducción-al-proyecto)
* [Fotos de Equipo](#fotos-de-equipo)
* [Fotos del Vehículo](#fotos-del-vehiculo)
* [Diseño de Hardware](#diseño-de-hardware)
   * [Componentes de Detección](#componentes-de-detección)
   * [Componentes de Procesamiento de Información](#componentes-de-procesamiento-de-información)
   * [Componentes de Alimentación](#componentes-de-alimentación)
* [Desarrollo de la Aplicación para la Detección de Objetos](#Desarrollo-de-la-Aplicación-para-la-Detección-de-Objetos)
* [Cálculo de Torque y Velocidad](#cálculo-de-torque-y-velocidad)
* [Sistema de Alimentación](#sistema-de-alimentación)
   * [Cálculo del Consumo Energético Total](#cálculo-del-consumo-energético-total)
   * [Autonomía Estimada](#autonomía-estimada)
* [Sistema de Detección de Objetos](#sistema-de-detección-de-objetos)
* [Paso a Paso de la Construcción](#paso-a-paso-de-la-construcción)
   * [Etapa de Diseño y Ensamblaje del Vehículo](#etapa-de-diseño-y-ensamblaje-del-vehículo)
   * [Montaje del Vehículo](#montaje-del-vehículo)
   * [Diseño de Base 3D](#diseño-de-base-3d)
   * [Cambio de Base](#cambio-de-base)
   * [Problemas Encontrados](#problemas-encontrados)
* [Pruebas Realizadas](#pruebas-realizadas)
   * [Prueba del Sensor de Color](#prueba-del-sensor-de-color)
   * [Fabricación de Pista a Escala](#fabricacion-de-pista-a-escala)
   * [Desarrollo de la Lógica de Vueltas a la Pista](#desarrollo-de-la-logica-de-vueltas-a-la-pista)
   * [Desarrollo de la Lógica de Evasión de Objetos](#desarrollo-de-la-logica-de-evasion-de-objetos)
   * [Desarrollo de Códigos Combinados](#desarrollo-de-códigos-combinados)
   * [Video Resumen de las Pruebas](#video-resumen-de-las-pruebas-de-vuelta-a-la-pista-y-esquive-de-objetos)
   * [Prueba de Vueltas a la Pista](#prueba-de-vueltas-a-la-pista)
   * [Prueba de Evasión de Obstáculos](#prueba-de-evacion-de-obstaculos)
---

##  Contenido del Repositorio

Este repositorio contiene los siguientes directorios para organizar nuestro proyecto:

* `t-photos`: Incluye 2 fotos del equipo (una oficial y una divertida con todos los miembros).
* `v-photos`: Contiene 6 fotos del vehículo (desde todos los ángulos, superior e inferior).
* `video`: Archivo `video.md` con el enlace a un video de demostración de conducción.
* `schemes`: Diagramas esquemáticos (JPEG, PNG o PDF) de los componentes electromecánicos, ilustrando la conexión de elementos electrónicos y motores.
* `src`: Código del software de control para todos los componentes programados para la competición.
* `models`: Archivos para modelos usados por impresoras 3D, cortadoras láser y máquinas CNC para producir elementos del vehículo.
* `other`: Archivos adicionales para entender cómo preparar el vehículo para la competición (documentación de conexión SBC/SBM, carga de archivos, especificaciones de hardware, etc.).

---

##  Introducción al Proyecto

Para esta competición, hemos desarrollado un diseño de vehículo **cómodo, fácil de modificar y sencillo**, donde todos los elementos interactúan a la perfección. Nos enorgullece presentar nuestro primer prototipo de vehículo autónomo con reconocimiento de objetos y colores.

El diseño y la construcción de nuestro vehículo son íntegramente propios. Su **estructura de dos niveles** permite que todos los componentes encajen armoniosamente. Incorpora **vigas de soporte extraíbles** para facilitar el desmontaje, un **sistema de transmisión** tipo LEGO que transfiere la potencia del motor a las ruedas traseras, y un **sistema de dirección** también con piezas LEGO, controlado por un servomotor para mayor precisión en los giros.

Este proyecto ha sido el resultado del esfuerzo y la dedicación de todo nuestro equipo. Cada cable, cada sensor, y cada línea de código es un testimonio de nuestro **esfuerzo técnico y financiero**. No solo construimos un vehículo; demostramos que con pasión y ganas de aprender, se pueden superar los límites.

---
##  Fotos de equipo
| ![ Foto Oficial](t-photos/foto_equipo.jpeg)  | ![ Foto Equipo](t-photos/foto_equipo2.jpeg) | 
| :----: | :---------- |


---
##  Fotos del vehiculo 
| ![Arriba del vehiculo](v-photos/arriba.JPG)  |  ![Debajo de vehiculo](v-photos/abajo.JPG)  |  ![Lateral derecho](v-photos/lateral-derecho.JPG) |
| :----: | :------------------- | :---------- |
| ![lateral izquierdo](v-photos/lateral-izquierdo.JPG)  |  ![frente](v-photos/frente.JPG)  |  ![detras](v-photos/detras.JPG) |

---

##  Diseño de Hardware

![Control Principal](other/primer_nivel.jpg)

| Imagen | Nombre de Componente | Descripción |
| :----: | :------------------- | :---------- |
| ![Base del Primer Piso](other/base.jpg) | **Base del Vehículo** | Con medidas de 11.5 cm x 2.5 cm, imita un coche de Fórmula 1, optimizando la distribución de espacio y la movilidad. La parte frontal permite alojar tres sensores (1 ultrasónico central y 2 infrarrojos diagonales). Aunque el cartón inicial presentó debilidades en la dirección, el uso de acrílico o un material más rígido es la solución ideal. |
| ![Transmisión](other/transmision.jpg) | **Sistema de Transmisión de Legos** | Distribuye la potencia del motor entre ambas ruedas traseras, permitiendo el giro continuo del motor incluso si una rueda se bloquea y evitando daños. Elegido por su facilidad de uso y tamaño adecuado, permite también la reversa. |
| ![Motor](other/encaje_motor.jpg) | **25GA370 Motor DC con Encoder** | Motor común en estos vehículos por su potencia y velocidad. El codificador integrado mide velocidad y dirección (aún no utilizado). **Especificaciones**: Potencia nominal: 4 W, Tensión nominal: 6V, Velocidad nominal: 220 RPM, Peso: 400g, Caja reductora: 21.3:1, Torque nominal: 0.35 kg·cm. |
| ![Dirección](other/direccion.jpg) | **Sistema de Dirección** | Construido con piezas LEGO, su tamaño es ideal y permite adaptar un servomotor de 180 grados para controlar el movimiento. El servomotor se centra a 90 grados, gira a la izquierda a 180 grados y a la derecha a 0 grados. |
| ![Separadores](other/separadores.jpg) | **Separadores de Acrílico** | Facilitan un montaje cómodo y modular. Girar los pilares permite desatornillar y desmontar el vehículo rápidamente. |



### Componentes de Detección

| Imagen | Nombre de Componente | Descripción |
| :----: | :------------------- | :---------- |
| ![Sensor Ultrasónico](other/sensor_ultrasonico.jpg) | **Sensor Ultrasónico HC-SR04** | Mide la distancia a obstáculos mediante ondas ultrasónicas. Lo utilizamos en la parte frontral, lateral izquierda, lateral derecha y en la parte trasera del vehiculo para detectar proximidad. |
| ![Sensor Infrarrojo](other/sensor_infrarojo.jpg) | **Sharp GP2Y0A21YK0F Sensor Infrarrojo** | Garantiza una detección de objetos rápida y precisa. Se ubican en las diagonales delanteras del vehículo. |
| ![Sensor Color](other/sensor_color.png) | **TCS34725 Sensor de Colores** | Permite una detección precisa del color a corta distancia, ubicado en la parte inferior para identificar los colores de las franjas de la pista. Alta sensibilidad y amplio rango dinámico (3.800.000:1), funcionando incluso tras cristales oscuros. |
| ![Giroscopio](other/sensor_giroscopio.jpg) | **MPU6050 Sensor Giroscopio y Acelerómetro** | Unidad de medición inercial (IMU) de 6 grados de libertad (DoF) que combina un acelerómetro y un giroscopio de 3 ejes. Ampliamente utilizado en navegación, radiogoniometría y estabilización. |
| ![Magnetometro](other/GY-273-1.png) | **Magnetómetro GY-273 HMC5883L** | Funciona como una brújula digital, midiendo la intensidad y la dirección del campo magnético de la Tierra en sus tres ejes (X, Y, Z). Al detectar este campo, puede determinar su orientación con respecto al norte magnético. |
| ![Cámara Celular](other/celular.png) | **Cámara del Teléfono Celular** | Utilizando una app desarrollada en android studio usando las librerias de OpenCV, el celular en general se usa como procesador de imagenes y la cámara del celular se usa como sensor para identificar el color y los objetos. |

### Componentes de Procesamiento de Información

| Imagen | Nombre de Componente | Descripción |
| :----: | :------------------- | :---------- |
| ![ESP32](other/esp32.png) | **Microcontrolador ESP32** | Actúa como el cerebro del vehículo, conectando y controlando todos los sensores y motores. |

### Componentes de Alimentación

| Imagen | Nombre de Componente | Descripción |
| :----: | :------------------- | :---------- |
| ![Batería](other/bateria_lipo.png) | **Batería LiPo 2200mAh 7.4V** | Fuente de alimentación recargable que alimenta todo el sistema, proporcionando movilidad y autonomía. Seleccionada por su capacidad para suministrar suficiente energía a los motores y componentes internos. |
| ![Regulador](other/Modulo_regulador_alimentacion_LM2596S.jpg) | **LM2596 Regulador de Voltaje** | Reduce el voltaje de manera eficiente. Voltaje de entrada: 4.5 V a 40 V CC. Voltaje de salida: 1.23 V a 37 V CC. Corriente de salida: Máx. 3 A, se recomiendan 2.5 A. |

---

# Desarrollo de la Aplicación para la Detección de Objetos

Para la detección de obstáculos, hemos desarrollado una aplicación móvil usando **Android Studio** e implementando la biblioteca **OpenCV**, la app es el cerebro de nuestro vehículo autónomo en cuanto a resolución de obstáculos se refiere. Permite que nuestro vehiculo identifique y reaccione de forma inteligente a los obstáculos en su entorno. El sistema procesa imágenes en tiempo real capturadas por la cámara del teléfono y, mediante una comunicación serial confiable, envía comandos específicos a una placa **Arduino/ESP32** para controlar el comportamiento del vehiculo en el campo de competición.

| ![Logo de OpenCV](other/openCV.png) | ![Logo de Android Studio](other/andriod_studio.PNG) |
| :-------------------------------: | :---------------------------------------------: |

---
A continuación, detallaremos algunas de las versiones más importantes de nuestra aplicación, mostrando la evolución del sistema de detección a lo largo del desarrollo y cómo cada etapa nos ayudó a superar los desafíos del proyecto.

## Versión 1: Detección Básica de Colores

Esta es la primera versión del sistema, enfocada en la detección simple de colores. Su objetivo principal era identificar objetos de color **rojo** y **verde** para enviar un comando básico (`R` para rojo, `G` para verde) a través de una conexión USB a la placa Arduino.

El mayor reto en esta etapa fue lograr la comunicación efectiva entre un dispositivo móvil Android y un microcontrolador como el Arduino. Después de numerosos intentos y ajustes, se consiguió establecer una conexión estable y que el Arduino recibiera comandos, al menos de forma básica. Además, fue un desafío significativo lograr aprovechar al máximo la librería **OpenCV** en un entorno móvil, dadas las complejidades de su instalación y configuración en dispositivos Android, pero este obstáculo también fue superado con éxito.

**Imágenes de la Versión 1:**

| <img src="other/app_prueba.jpg" width="400"> | <img src="other/app_prueba2.jpg" width="400"> |
| :-------------------------------: | :---------------------------------------------: |

---

---

---

## Versión 2: Mejora en Detección y Comunicación

Se introdujeron mejoras significativas para hacer el sistema más robusto y preciso. Uno de los principales retos fue **optimizar la aplicación** para que los comandos se enviaran y fueran recibidos por el Arduino lo más rápido posible, al mismo tiempo que se lograba reconocer con precisión la **distancia** y **orientación** del objeto.

Con respecto a la orientación, una complicación particular surgió debido a la **posición horizontal de la cámara** en el teléfono. Al dividir la pantalla en tres sectores para determinar la ubicación del objeto, la perspectiva de la cámara causaba distorsiones, haciendo que un objeto pareciera estar más hacia un lado del vehículo de lo que realmente estaba.

### Cálculo de Distancia por Área

Para estimar la distancia al objeto, se implementó un método basado en la relación inversa entre la distancia y el área aparente del objeto en la imagen. El principio es simple: **cuanto más lejos está un objeto, más pequeño se ve**. Este enfoque requiere una calibración inicial para funcionar.

La fórmula utilizada es la siguiente:

$$ \text{Distancia} = \text{DistanciaConocida} \times \sqrt{\frac{\text{AreaConocida}}{\text{AreaMedida}}} $$

Donde:
* **DistanciaConocida**: Es una distancia fija y conocida a la que se calibra el sistema (por ejemplo, 30 cm).
* **AreaConocida**: Es el área en píxeles que ocupa el objeto en la imagen cuando se encuentra a la `DistanciaConocida`. Este valor se guarda una vez durante la calibración.
* **AreaMedida**: Es el área actual en píxeles que el objeto ocupa en la imagen durante la detección en tiempo real.

Este método permite que, tras una única calibración, el sistema pueda estimar la distancia de forma continua simplemente midiendo el tamaño del objeto detectado.

A pesar de los desafíos, se implementaron las siguientes mejoras:

* **Filtrado de Formas:** La aplicación ahora filtra las imágenes para reconocer específicamente **formas rectangulares**, lo que reduce los falsos positivos y mejora la precisión.
* **Comunicación Detallada:** Se mejoró la comunicación con Arduino para transmitir información más completa. Ahora se envían la **distancia** y la **orientación** del objeto, además de su color.
* **Mejoras en la UI:** Se optimizó el comportamiento general de la aplicación y se realizaron cambios sutiles en la interfaz de usuario para una mejor experiencia.

**Imágenes de la Versión 2:**

| <img src="other/Deteccion_con_orientacion (2).jpeg" width="400"> | <img src="other/Deteccion_Magenta.jpeg" width="400"> |
| :-------------------------------: | :---------------------------------------------: |

---

## Versión 3: Sistema de Detección Avanzado y Profesional

Esta versión representa una **reestructuración completa** y un salto cualitativo en el sistema, adaptándose a las nuevas necesidades y a un cambio en la plataforma del vehículo. Tras evaluar los resultados obtenidos en los regionales del país, se hizo evidente la necesidad de un sistema más avanzado y robusto capaz de mejorar significativamente el **esquive de obstáculos**.

Para ello, se realizó un cambio en el vehículo para usar un ESP32 como microcontrolador principal, lo que requirió que la aplicación también se adaptara a esta nueva plataforma. La estrategia clave fue desarrollar un sistema capaz de reconocer dos objetos a la vez y establecer rutas de forma anticipada, en lugar de esquivar los obstáculos uno a uno en tiempo real. Esto llevó la automatización de nuestro vehículo a otro nivel.

Sin embargo, la implementación de la detección dual presentó varios desafíos. El más evidente fue que requirió rehacer gran parte del código de envío de comandos. Para resolverlo, decidimos crear un archivo independiente, el CommandManager, que se encarga exclusivamente de la lógica de comandos y de gestionar la información de cada combinación de obstáculos o "caso".

Además, surgió la necesidad de elevar el teléfono para que la cámara pudiera observar ambos objetos cuando uno estuviera detrás del otro. Esto nos obligó a realizar una modificación en la base del vehículo, un cambio que se detalla más a fondo en la sección correspondiente de nuestro repositorio. Finalmente, una vez resueltos estos retos, se llevaron a cabo una gran cantidad de pruebas para asegurar que el sistema fuera robusto y pudiera enfrentar de manera fiable las distintas combinaciones de obstáculos en la competición.
Se lograron importantes avances:

* **Solución al Problema de Orientación:** Se corrigió el problema anterior con la división de la pantalla y la orientación. En esta versión, no usamos una división en tres sectores; simplemente dividimos la pantalla en **izquierda y derecha** para identificar la posición lateral de los objetos.
* **Cálculo de Distancia Avanzado:** El cálculo de distancia se encarga de identificar la posición longitudinal de los objetos (adelante, medio o atrás). Mediante algunos "trucos" y algoritmos optimizados, logramos filtrar y reconocer de manera efectiva **cada caso de colisión o esquive**.
* **Interfaz Profesional:** La interfaz de usuario fue completamente **rediseñada** para ser más intuitiva, profesional y pulida, ofreciendo una experiencia de usuario superior.
* **Monitor Serial Integrado:** La incorporación de un **monitor serial** para recibir información directamente del microcontrolador ha sido fundamental para el **debug** y las pruebas en tiempo real, agilizando el desarrollo.
* **Diseño Único:** La nueva interfaz no solo es 100% más práctica, sino que también cuenta con un diseño único que representa la identidad de nuestro equipo: **Prometheus Team**.

**Imágenes de la Versión 3:**
| <img src="other/Deteccio_doble_app.jpeg" width="400"> | <img src="other/Monitor_serial.jpeg" width="400"> |
| :-------------------------------: | :---------------------------------------------: |

---

##  Flujo de Proceso de la Aplicación 

El sistema opera como un ciclo continuo de detección y envío de comandos, con tres componentes principales: la interfaz de usuario, el análisis de imágenes y la comunicación serial.

### Flujo Detallado

* **Inicio y Conexión Automática (`MainActivity`):** Al iniciar, la aplicación solicita los permisos de la cámara e inicializa las librerías de OpenCV. La aplicación no requiere que el usuario conecte manualmente el dispositivo, ya que es capaz de detectar automáticamente el microcontrolador (Arduino/ESP32) y establecer la comunicación serial de forma autónoma.
* **Captura y Análisis de la Imagen (`ColorAnalyzer`):** Un hilo de alta prioridad se encarga de capturar continuamente imágenes de la cámara. Cada fotograma se envía al `ColorAnalyzer` para su procesamiento.
* **Preprocesamiento:** El `ColorAnalyzer` convierte el fotograma y aplica técnicas avanzadas como la ecualización de histograma para normalizar la iluminación.
* **Detección de Colores y Formas:** La aplicación crea "máscaras" para aislar los colores de los obstáculos (rojo, verde y magenta) y busca contornos en esas máscaras, aplicando filtros geométricos para detectar pilares rectangulares.
* **Cálculo de Distancia:** Se calcula el área de cada objeto detectado en píxeles para estimar su distancia en centímetros.
* **Generación de Comandos (`CommandManager`):** La información del objeto detectado (color, distancia y posición en la imagen) se envía al `CommandManager`. Este módulo compara los datos del objeto con "casos" predefinidos para la detección de uno o dos objetos y determina el código de comando adecuado.
* **Envío del Comando (`MainActivity`):** El comando generado se añade a una cola de procesamiento. Un hilo dedicado a la comunicación USB lee la cola y envía el comando al microcontrolador a través del puerto serial, con una pausa mínima para evitar la saturación.
* **Monitoreo (`SerialMonitorActivity`):** Una actividad separada permite al usuario monitorear en tiempo real los datos que se envían y reciben, lo cual es útil para la depuración del sistema.

### Puntos Clave

* **Conexión Automática:** La aplicación se conecta automáticamente al microcontrolador al detectarlo, lo que agiliza el proceso de inicio.
* **Visión por Computadora Avanzada:** El uso de OpenCV con ecualización de histograma y filtros geométricos asegura una detección precisa de los pilares.
* **Doble Detección de Objetos:** La capacidad de diferenciar entre un objeto primario y uno secundario es fundamental para la navegación en el campo de la WRO.
* **Arquitectura de Hilos:** El uso de múltiples hilos separa las tareas de alto rendimiento (cámara, USB) del hilo principal, garantizando una operación fluida.
* **Sistema de Comandos Basado en Casos:** La lógica del `CommandManager` es clara y organizada, facilitando su mantenimiento y escalabilidad.
* **Comunicación USB Serial:** La conexión entre el teléfono y el robot se realiza de forma confiable a través de USB.

  
Diagrama de flujo, diseñado en Lucidchart, para una mejor visualización del funcionamiento de la aplicación WRO Prometheus.

![Diagrama de Flujo de la Aplicación](other/Diagrama_de_flujo_app.png)

---


##  Cálculo de Torque y Velocidad

### Cálculo de Torque Necesario para Mover el Vehículo:

El torque necesario ($T_{\text{necesario}}$) se calcula mediante la fórmula:
$T_{\text{necesario}} = m \cdot g \cdot r$

Donde:
* $m$ = masa del vehículo (0.943 kg)
* $g$ = gravedad (9.81 $m/s^2$)
* $r$ = radio de las ruedas (0.04 m)

$T_{\text{necesario}} = 0.943 \cdot 9.81 \cdot 0.04 = 0.370 \text{ N} \cdot \text{m}$

### Cálculo de Torque a la Salida (después de la reducción) según el motor DC 25GA370:

$T_{\text{salida}} = T_{\text{motor}} \cdot \text{Reducción}$
$T_{\text{salida}} = 0.0343 \cdot 21.3 = 0.7306 \text{ N} \cdot \text{m}$

### Cálculo de la Velocidad Lineal del Vehículo:

Convertimos la velocidad del motor a radianes por segundo ($\omega$):
$\omega = \frac{220 \cdot 2\pi}{60} = 23.04 \text{ rad/s}$

Velocidad lineal del vehículo ($v$):
$v = \omega \cdot r = 23.04 \cdot 0.04 \approx 0.92 \text{ m/s}$

El motor empleado tiene una velocidad sin carga de 4690 RPM y un torque nominal de 0.35 kg·cm (0.0343 N·m). Mediante una caja reductora de 21.3:1, la velocidad se reduce a 220.2 RPM en el eje de salida, lo que resulta en una velocidad lineal del vehículo de aproximadamente 0.92 m/s.

Gracias a esta reducción, el torque en las ruedas alcanza 0.7306 N·m, lo cual supera el torque mínimo necesario para mover el vehículo de 943 gramos (0.370 N·m). Por lo tanto, el sistema cumple adecuadamente los requerimientos de tracción y movilidad para condiciones normales.

---

##  Sistema de Alimentación

###  Versión 1:

![Diagrama de Alimentación 1](schemes/power_supply_diagram.jpeg)

Este diagrama representa la versión inicial del sistema de alimentación de nuestro proyecto, concebido con una configuración de componentes más simple. 

En esta iteración, el circuito se alimenta por completo a través de una **batería LiPo de 2200 mAh** (7.4V), cuya energía se distribuye a los distintos módulos después de pasar por un interruptor principal de encendido. Un **regulador de voltaje XL6009** eleva el voltaje para alimentar el puente H **L298N**, asegurando que nuestro motor DC reciba la tensión de funcionamiento adecuada. 

De forma paralela, un **módulo DSN-Mini 360** se encarga de reducir el voltaje de la batería para alimentar directamente el pin Vin de la placa **Arduino**, que a su vez controla el servomotor de dirección. Este diseño sienta las bases del sistema, aunque futuras versiones incorporarán más componentes para optimizar su rendimiento.

###  Versión 2:

![Diagrama de Alimentación 2](schemes/diagrma-alimentacion2.jpg)

 #### Cambios agregados a las conexiones en el sistema de alimentacion:
  1. Alimentamos ambos microcontroladores con el mismo reductor de voltaje a 7v.
  2. Agregamos un reductor de voltaje exclusivo para sensores ultrasonicos e infrarrojos y el servomotor de la direccion(que ahora se alimenta del reducctor y no del arduino).

###  Versión 3:

![Diagrama de Alimentación 3](schemes/Esquema-alimentacio-3.png)

Tras la primera iteración, este diagrama detalla una revisión del sistema de alimentación del vehículo autónomo, ahora con una arquitectura que soporta más componentes y funcionalidades. 

La **batería LiPo 2S de 2200mAh** (7.4V, con un máximo de 8.4V) sigue siendo la fuente principal de energía, conectada a un interruptor general. A partir de este punto, la alimentación se ramifica: una rama se dirige a un **step-up/down XL6009** ajustado a 7.5V, que a su vez alimenta el **puente H L298N**. Este entrega aproximadamente 6V al **Motor 25GA-370** tras su caída de voltaje interna. 

De forma paralela, la batería alimenta dos módulos **step-down DSN-Mini 360**. El primero, configurado a 6V, proporciona energía a ambos microcontroladores **ESP32 #1** y **ESP32 #2**. El segundo módulo DSN-Mini 360, con un ajuste de 5V, se encarga de la alimentación general para el **Servomotor SG90 180°** y otros posibles sensores, ofreciendo una línea de voltaje estable y adecuada para estos componentes. Esta configuración ampliada permite una gestión de energía más distribuida y específica para las necesidades de cada subsistema del vehículo.

### Cálculo del Consumo Energético Total

| Componente | Cantidad | Consumo Estimado (mA) | Total (mA) |
| :------------------------------- | :------: | :--------------------: | :--------: |
| Motor DC | 1 | 500 mA (en carga) | 500 mA |
| Servo 180° | 1 | 150 mA (típico) | 150 mA |
| Sensor ultrasónico HC-SR04 | 4 | 15 mA c/u | 60 mA |
| Sensor infrarrojo Sharp | 2 | 30 mA c/u | 60 mA |
| ESP32 (sin WiFi/BT y sin carga) | 2 | ~70 mA c/u | 140 mA |
| Sensor Magnetómetro GY-273 | 1 | ~0.1 mA | 0.1 mA |
| Sensor Giroscopio/Acelerómetro MPU6050 | 1 | ~3.6 mA | 3.6 mA |
| **TOTAL** | **—** | **—** | **~913.7 mA** |

---

### 🔋 Corriente total aproximada: **~913.7 mA**

---

### ⏱️ Autonomía Estimada

**Fórmula:**
`Autonomía (h) = Capacidad de la batería (mAh) / Consumo total (mA)`

**Ejemplo con batería de 2200 mAh:**
`Autonomía ≈ 2200 mAh / 913.7 mA ≈ 2.40 horas`

> ⚠️ *Nota:* Este valor es teórico y asume un consumo constante. En la práctica, el consumo del motor puede aumentar significativamente si se encuentra con un obstáculo, por lo que la autonomía real podría variar.
---

##  ¿Porque Seleccionamos esta Fuente de Alimentacíon?

Seleccionamos una bateria LiPo (Bateria de Polimero de Litio) por su alta capacidad de almacenamiento de energía y al mismo tiem su bajo peso, en especifico nuestra bateria marca (Ovonic 2S 2200mAh 50c) donde las siglas 2S significa que la bateria es de dos celdas, cada una de ellas tiene un voltaje nominal de 3,7V, en total nuestra bateria tiene 7,4V nominales; cuando se menciona 2200mAh (Miliamperio-hora) nos referimos a la  capacidad de almacenamiento de energía de la bateria, optamos por esa cantidad de miliamperios por su rendimiento en pruebas de larga duración; y para finalizar cuando nos referimos a 50c, hablamos de la tasa de descarga de la bateria, esto nos indica la velocidad a la que la bateria puede descargarse de forma segura, en nuestro caso 50c nos permite descargar la bateria cincuenta veces su capacidad nominal.

# Sistema de Detección de Objetos

## Versión 1:

![Diagrama de Sensores 1](schemes/diagrama-sensores.jpg)

Cada uno de los sensores instalados desempeña una función crítica para la navegación del vehículo y la prevención de colisiones:

* Los **sensores infrarrojos** se posicionan en las diagonales del vehículo, proporcionando una detección rápida y precisa para evitar obstáculos cercanos.
* Los **sensores ultrasónicos** están ubicados en la parte frontal y en los laterales, midiendo distancias para mantener un margen de seguridad con respecto a las paredes y otros objetos.
* El **sensor giroscopio** ayuda a mantener una trayectoria estable y recta, facilitando giros precisos y la orientación general del vehículo.
* El **sensor de color** se utiliza para identificar las líneas de la pista, permitiendo que el vehículo siga la ruta designada de manera autónoma.

## Versión 2: 

![Diagrama de Sensores](schemes/diagrama-sensores2.jpg)

#### Mejoras implementadas en el diagrama:

1.  Se incorporó un segundo microprocesador Arduino para gestionar exclusivamente el módulo de puente H. Esta separación de responsabilidades minimiza las interferencias eléctricas y mejora el rendimiento general del sistema.
2.  Se añadió un botón de inicio para controlar la ejecución del código. Esto permite que el vehículo se encienda y realice las calibraciones iniciales de los sensores, permaneciendo en un estado de espera hasta que se active el recorrido.

## Versión 3: 

![Diagrama de Sensores](schemes/Esquema-alimentacion-4.png)

Esta versión representa una mejora significativa en la capacidad de procesamiento y la precisión del sistema de navegación y evasión. Los cambios más importantes incluyen:

1.  **Migración de microcontroladores:** Ambos microcontroladores Arduino fueron reemplazados por dos **ESP32**. Este cambio se hizo necesario para obtener mayor potencia de procesamiento y memoria, lo que permite ejecutar algoritmos más complejos y eficientes para la evasión de obstáculos.
2.  **Ajuste de compatibilidad de voltaje:** Se agregaron **cuatro divisores de voltaje** para los sensores ultrasónicos HC-SR04. Los pines de entrada de los ESP32 operan a 3.3V, por lo que era necesario reducir el voltaje de 5V que emiten estos sensores para evitar daños. Este ajuste de compatibilidad se detallará mas adelante.
3.  **Expansión del sistema de sensores:** Se integró un **sensor magnetómetro (HMC5883L)** para mejorar la orientación del vehículo. Los datos de este sensor, combinados con los del IMU y el resto de los sensores, ofrecen una referencia más precisa y robusta para la navegación.
4.  **Detección de obstáculos ampliada:** Se añadió un **sensor ultrasónico trasero** para mejorar la capacidad de detección y maniobrabilidad en situaciones de marcha atrás, proporcionando datos adicionales para un sistema de evasión más completo.
5.  **Sistema de notificaciones:** Se agregaron un **LED** y un **buzzer** para proporcionar indicaciones visuales y sonoras sobre el estado operativo del vehículo, como el inicio del recorrido, la detección de obstáculos o cualquier error del sistema.

Estos cambios consolidan la capacidad del vehículo para operar de manera más autónoma y segura en su entorno, basándose en un flujo de datos más rico y un procesamiento más potente.

---

### Explicación del Divisor de Voltaje para Sensores HC-SR04

Un **divisor de voltaje** es un circuito simple que permite obtener un voltaje de salida (Vout) menor a partir de un voltaje de entrada (Vin), utilizando una configuración de dos resistencias en serie. Esta solución fue esencial para adaptar la salida de 5V del pin `Echo` de los sensores HC-SR04 a la entrada de 3.3V de los pines GPIO del microcontrolador ESP32, protegiéndolo de posibles daños.

La fórmula utilizada para calcular el voltaje de salida es:

$$ V_{out} = V_{in} * (R2 / (R1 + R2)) $$

Para nuestro circuito, utilizamos los siguientes valores de resistencias:

* **R1:** 1 kΩ
* **R2:** 2 kΩ

Al aplicar la fórmula con estos valores y la entrada de 5V de los sensores, obtenemos el siguiente resultado:

$$ V_{out} = 5V * (2kΩ / (1kΩ + 2kΩ)) $$
$$ V_{out} = 5V * (2kΩ / 3kΩ) $$
$$ V_{out} \approx 3.33 V $$

Este voltaje de salida de aproximadamente 3.3V es compatible con las entradas de 3.3V del ESP32, permitiendo la comunicación segura entre el sensor y el microcontrolador.

A continuación, se muestra un esquema de conexion del divisor de voltaje simulado en tinkercad:

![Divisor de voltaje](schemes/Divisor-de-voltaje.png)

---

### Implementación Física del Divisor de Voltaje

**Vista Superior:**

Esta imagen muestra la disposición de los componentes en la parte superior de la baquelita, incluyendo las resistencias de los divisores de voltaje conectadas a los pines del ESP32 y a los cables del sensor ultrasónico.

![Divisores de Voltaje Vista Superior](other/Divisor-de-voltaje-arriba.jpg)

**Vista Inferior:**

Esta imagen ofrece una perspectiva de las conexiones soldadas en la parte inferior de la baquelita, mostrando cómo se unen las resistencias para formar los divisores de voltaje y cómo se conectan los cables.

![Divisores de Voltaje Vista Inferior](other/divisor-de-voltaje-abajo.jpg)

### Timelapse de Diseño de Circuitos

Aquí puedes observar un timelapse del proceso de diseño de los diagramas de alimentación y de las conexiones de entrada/salida de nuestros microcontroladores.

Haz clic en la imagen para ver el vídeo:

<a href="https://youtube.com/shorts/UtEvRMK07D0?si=VvvrS2JtITUjtCgx" target="_blank">
  <img src="https://github.com/erchivox/Prometheus-Team/blob/f2510ccf25a47e93904d0bfb8a12c54cac5ecb01/other/Dise%C3%B1ando%20Diagramas-Cover.jpg" alt="Miniatura del Video Timelapse de Diseño de Circuitos" width="480px">
</a>

---

##  Paso a Paso de la Construcción

### Etapa de Diseño y Ensamblaje del Vehículo

El diseño de nuestro vehículo comenzó con un **modelo 3D en Tinkercad** para definir las dimensiones y el tamaño de cada componente. Este modelo se concibió para un vehículo de dos plantas:

* **Planta Baja:** Aloja el sistema de transmisión, sistema de dirección, motor, sistema de potencia y sensores (frontales, laterales y diagonales).
* **Segundo Piso:** Contendrá el Arduino y la cámara (originalmente no se pensó en un teléfono celular, pero se consideró la idea en esta etapa). La imagen muestra cómo un teléfono celular interactuaría con nuestro vehículo.

| Vista Frontal | Vista Base | Vista Lateral |
| :-----------: | :--------: | :-----------: |
| ![Modelo 3D Frontal](other/Frontal3d.png) | ![Modelo 3D Base](other/base_3d.png) | ![Modelo 3D Lateral](other/lateral3d.png) |

### Montaje del Vehículo

| Imagen | Descripción |
| :----: | :---------- |
| ![Molde Base](other/base1.jpg) | Aquí se muestran algunas medidas ya probadas para las primeras pruebas del coche. Utilizamos cartón como material de prueba para realizar cambios sin incurrir en costos adicionales. La base se cubrió con una doble capa de cartón para mayor rigidez. |
| ![Instalación de Transmisión](other/montaje.jpg) | Colocamos la transmisión LEGO en la base y realizamos los cortes necesarios para que el motor, la dirección y la transmisión encajaran correctamente. Luego, instalamos el servomotor en el diferencial y el motor DC en la transmisión. |
| ![Instalación de Servo](other/instalacion_servo_direccion.jpg) | Encaje del servomotor con el sistema de direccion. |
|  ![Instalación de Motor DC](other/instalacion_motor_transmision.jpg) | Encaje de motor dc con el sistema de transmision. |
| ![Primera Estructura](other/primera_estructura.jpg) | Con todos los elementos esenciales ensamblados, conectamos el motor a la alimentación con el regulador para una prueba de funcionamiento rápida, verificando el correcto desempeño del motor y la transmisión. |

### Diseño de base 3d
Teniendo en cuenta el modelo diseñado en carton empezamos con la modelacion en 3d de nuestro vehiculo para construirlo con un material mas resistente que no sufra los defectos de nuestra primera base de carton.

| Imagen | Descripción |
| :----: | :---------- | 
| ![diseño de la base](other/base3d2.png) | Empezamos realizando el primer piso en base a las medidas que teniamos previamente, en el software de Tinkercad. |
| ![diseño de la base](other/modelo3dgris.jpeg) | En nuestro primer piso realizamos la base donde encajara nuestro motor, para ello realizamos un agujero con el diametro del motor, y un pequeño espacio para la union de los ejes. |
| ![Montaje de la Nueva Base 2](other/base3d3.png)| Luego de ya tener fijado nuestro motor empezamos a diseñar y probar el espacio en donde encajaria el soporte para nuestro servomotor. Ademas agregamos el soporte para nuestros soportes hexagonales. |
| ![diseño de la base](other/modelointermediogris.jpeg) | Por debajo del vehiculo realizamos el espacio para nuestro sensor de color (posteriormente se cambio de posicion para optimizar el rendimiento del vehiculo).  |
| ![Montaje de la Nueva Base 4](other/protectorUltrasonico3d.jpeg) | Empezamos a probar el segundo piso con las mismas medidas del piso de abajo y la altura que habria entre los dos pisos. |
| ![diseño de la base](other/modelo3drojo.jpeg) | En el segundo piso realizamos los espacios para los tornilos que fijarian los separadores hexagonales al segundo piso. Ademas se hicieron espacios para el interruptor de encendido y espacio para fijar la bateria y el dispositivo movil mediante precintos de seguridad. |
| ![diseño de la base](other/modelo3drojo2.jpeg) | A nuestro segundo piso le agregamos una pequeña base para nuestro medidor de voltaje. |
| ![diseño de la base](other/modelointermediofull.jpeg) | Volviendo a nuestro primer piso realizamos medidas con nuestros componentes y terminamos de arreglar algunos detalles. |
| ![diseño de la base](other/modelo3drojo3.jpeg) | Primer y segundo piso ya armados con su respectiva separación. |
| ![diseño de la base](other/modelo3drojo4.jpeg) | Agregamos un aleron al diseño como toque estetico. |
| ![diseño de la base](other/modelo3drojo5.jpeg) | Vista final de como quedo todo el diseño 3d. |
| ![diseño de la base](other/modelo3dnegro2.jpeg) | Exportamos nuestro diseño a Fusion 360, para trabajar con mas precision y agregar las tolerancias para realizar la impresion 3d. |
| ![diseño de la base](other/modelo3dnegro3.jpeg) | Terminamos de trabajar con las tolerancias respectivas y haber confirmado todas las medidas. | 
| ![diseño de la base](other/modelo3dnegro.jpeg) | Debajo del vehiculo redondeamos el sobresaliente de la base de nuestro motor para darle un toque mas estetico y funcional. |
| ![diseño de base](other/modelo3dazul.jpeg) | Luego de terminar todos estos ajustes procedimos a exportar cada parte del diseño de forma independiente en un formato (STL). |
| ![Montaje de la Nueva Base 4](other/impresion3dterminada.jpeg) | Y aca podemos observar los resultados finales de la impresión 3d. |


---

## Cambio de Base

| Imagen | Descripción |
| :----: | :---------- |
| ![Montaje de la Nueva Base](other/proceso.jpg) | Inicio del proceso de cambio de base del prototipo hecho en carton a nuestra impresion 3d y pequeña modificacion en el primer piso de nuestro vehiculo. |
| ![Montaje de la Nueva Base 2](other/proceso-2.jpg) | Aca podemos observar la colocacion del sistema de dirección y tracción trasera. |
| ![Montaje de la Nueva Base 3](other/proceso-3.jpg) | Empezamos a realizar todo el cableado de algunos componentes. |
| ![Montaje de la Nueva Base 4](other/proceso-4.jpg) | Se continúa con el ensamblaje de la nueva base. |

### Mejoras en el orden de los cables y componentes (Segundo Prototipo - Base 3D)

Tras el cambio a la base impresa en 3D, realizamos una mejora significativa en el cableado y la organización interna de nuestro segundo prototipo para optimizar su rendimiento y estética.

Haz clic en la imagen para ver el vídeo:

<a href="https://www.youtube.com/shorts/rl1eokkc18E" target="_blank">
  <img src="other/TIMELAPSE.SIMPLIFICANDO-Cover.jpg" alt="Miniatura del Video de Mejora de Cableado" width="480px">
</a>

### Reubicación, mejoras y modificaciones (Tercer Prototipo)

A continuación, se detallan las modificaciones y mejoras implementadas en la tercera versión del prototipo.

#### 1. Reubicación de sensores y mejoras en el cableado

Para esta nueva versión, se realizó un reajuste estratégico en la posición de varios sensores clave, buscando mayor comodidad y eficiencia. Esto incluyó el reposicionamiento de los sensores de ultrasonido frontales, los sensores Sharp y el sensor de color, que fue desplazado hacia la parte delantera. Adicionalmente, se optimizó el cableado superior para adaptarlo a la nueva configuración de dos microcontroladores ESP32 y la integración de un magnetómetro. Finalmente, se reforzó la base del primer piso mediante el uso de tornillos inferiores para una mayor estabilidad.

![Descripción de la imagen](other/modificacion_base_v3.jpeg)

#### 2. Modificaciones en la base para el nuevo soporte de teléfono

Inicialmente, para validar el concepto y realizar las primeras pruebas de campo, se diseñó una adaptación provisional utilizando un material de goma espuma plástica. Este prototipo inicial nos permitió evaluar la funcionalidad del soporte de manera rápida y económica. Una vez comprobada su viabilidad, se procedió a diseñar e implementar la solución definitiva, que consistió en la integración de un trípode adaptado a la base del robot.

Como se mencionó anteriormente al mejorar nuestra app y su sistema de detección de objetos ahora debíamos mejorar el soporte de nuestro teléfono, para ello se diseñó e implementó una solución para el posicionamiento elevado del teléfono encargado del análisis de imágenes en tiempo real. La base de esta solución fue un trípode de teléfono convencional, el cual fue sometido a un proceso de modificación estratégica para su integración en el chasis del vehículo.

El procedimiento inicial consistió en un desmontaje selectivo, eliminando componentes innecesarios como las patas extensibles, para conservar únicamente los elementos funcionales: el soporte de sujeción del dispositivo y su vástago o columna central. El objetivo principal era posicionar el teléfono en una orientación horizontal y a una altura estratégica, garantizando así que la cámara tuviera un campo de visión amplio. Esta configuración es crucial, ya que permite al sistema capturar simultáneamente información tanto de los objetos que se encuentran delante como de aquellos que se encuentra por detrás, lo que representa una mejora significativa y un requisito indispensable para la fiabilidad de nuestro nuevo sistema de navegación.

Una vez finalizada la fase de planificación, procedimos con la operación de montaje. Para adaptar e integrar el soporte al vehículo, se realizó una perforación de precisión en la base de la plataforma superior. Dicho orificio fue dimensionado específicamente para alojar el diámetro del vástago del trípode, asegurando un encaje firme. Posteriormente, se fijó sólidamente la estructura y se realizaron diversas pruebas de ajuste y resistencia para validar la solidez del montaje. Se llevaron a cabo ensayos de vibración y estabilidad para comprobar que el soporte permanecería estático durante el desplazamiento del vehículo, garantizando así una navegacion segura y un funcionamiento correcto del sistema de análisis de imagenes que implementa nuestra app.

| Imagen | Descripción |
| :----: | :---------- |
| ![Modificación al tripode](other/desarmando-tripode.jfif) | Se modifico el tripode antes de ser instalado en la base del vehículo, de maneras que se descartaron las partes innecesarias para la adaptacion como lo son el mango de agarre y las patas de apoyo. |
| ![Marca para la Perforación](other/marca-x.jfif) | Aquí se puede observar la marca en las medidas necesarias para la perforación de la base del vehículo, para así poder adaptar este tripode modificado. |
| ![Perforación hecha](other/perforacion-hecha.jfif) | Se puede observar el resultado luego de que se hiciera la perforación que permitira la instalación del tripode. |
| ![Finalizacion de la instalacion del tripode](other/fijacion-del-soporte.jfif) | Se busco una posición optima para posicionar el tripode, de manera que pudiera tener un mejor angulo de visión, lo que permitira mejorar la visibilidad de los obstaculos en pista. Luego se procedio a fijar dicho tripode a la base superior del vehículo con una combinación de quimicos que permitira una fijación optima. |

---

## Problemas encontrados. 

### interferencia con el puente H, solución: Un sistema maestro y esclavo.

Al momento de realizar las pruebas de vueltas a la pista nos percatamos que el vehículo no respondía correctamente y al momento de realizar a el giro nunca terminaba de realizarlo o simplemente no detectaba la línea y por esto no lo realizaba. Entre mucha investigación encontramos que la falla era una interferencia con los pines que mandaban las señales al modulo de puente H. Por lo que buscamos aislar estos cables resultando sin éxitos. 
La solución probada fue conectado un segundo microcontrolador Arduino esclavo, encargado exclusivamente de enviar los comandos de avanzar, retroceder, detenerse y algún cambio de velocidad al módulo puente H, mientras que el Arduino principal en este caso el maestro, llevara todos los sensores y cables conectados al esclavo para indicarle que acción debe ejecutar el puente H. 
De esta manera logramos distanciar esta interferencia y el código fluía continuamente sin problemas

#### Diagrama del codigo del microcontrolador arduino esclavo.

![Diagrama del codigo del arduino esclavo](other/diagrama_esclavo.png)

---

## Pruebas Realizadas

### Prueba del Sensor de Color

Haz clic en la imagen para ver el vídeo:

[![Mira el video del proyecto](other/portada_pruebas.png)](https://youtu.be/7GS6mCXmGks)

### Fabricacion de pista a escala

Para las pruebas, creamos una imitación de la pista de competición utilizando láminas recicladas, pegadas para cumplir con los estándares de 3x3 metros. Nuestra pista tiene un ligero error de aproximadamente 10 cm. Las esquinas de la pista deben tener dos líneas, una azul y otra naranja, separadas por 30 grados cada una, tomando el mismo origen que la esquina interior del cuadrado.

![Pista de Pruebas](other/pista.jpg)

## Desarrollo de la Logica del codigo Maestro

### Funcion actualizarGiro():

  Para hacer que el vehiculo estuviera siempre guiado usamos la funcion actualizarGiro(), con ella simulamos un brujula digitar.

Esta función es un sistema de fusión de sensores que combina dos fuentes de datos para obtener el ángulo final del vehículo: el MPU6050 (giroscopio) y el magnetómetro (a través de obtenerAngulo()).

  El giroscopio mide la velocidad de rotación del vehículo (qué tan rápido está girando). La función toma esta velocidad y la suma a un ángulo acumulado (anguloZ) en cada ciclo de lectura. Piensa en esto como un cálculo continuo del ángulo de giro. Es muy preciso a corto plazo, pero tiene un problema: la deriva (pequeños errores que se acumulan con el tiempo).

 Para solucionar la deriva del giroscopio, la función llama a obtenerAngulo(), Esta función es el corazón del sistema, actúa como un compás digital usando el sensor QMC5883L (magnetómetro).

### Funcion obtenerAngulo():

 Se conecta con el magnetómetro y lee los valores magnéticos en los ejes X, Y y Z. Estos valores representan la intensidad del campo magnético de la Tierra. Utiliza los valores X e Y para calcular el ángulo en un plano 2D, de forma similar a cómo lo haría una brújula. Este ángulo es la dirección del vehículo con respecto al Norte magnético. Se aplican correcciones (xOffset, yOffset, declinacionMagnetica) para mejorar la precisión y compensar la ubicación geográfica.

  En la primera lectura válida, la función toma el ángulo absoluto actual y lo define como el punto de referencia (0°). Esto significa que la dirección en la que mira el vehículo al encenderse se considera su punto de partida.

 Todas las lecturas subsiguientes se comparan con ese ángulo de referencia inicial. El resultado es el ángulo relativo del vehículo con respecto a su dirección inicial, que es el valor que finalmente devuelve la función.

 El ángulo del magnetómetro es absoluto y no tiene deriva, pero es más susceptible a interferencias externas (como metales o campos magnéticos).

### Fusión de los datos

Aquí es donde ocurre la magia. La función compara el ángulo calculado por el giroscopio con el ángulo de referencia del magnetómetro. La diferencia entre ambos valores se usa para corregir suavemente el ángulo del giroscopio, ajustándolo hacia el valor más confiable del magnetómetro. El ajuste es gradual (* 0.1) para evitar saltos bruscos en la lectura.

Esta combinación aprovecha lo mejor de ambos mundos: la velocidad del giroscopio para detectar movimientos instantáneos y la precisión del magnetómetro para mantener el ángulo sin errores a largo plazo.
## Logica de vueltas a la pista

Para este momento ya teníamos idea de la lógica de nuestro primer código del primer desafío que seria la vuelta libre a la pista. En esta lógica usaremos las líneas de la pista para marca el momento exacto para cruzar y el conteo de vueltas siendo 4 líneas igual a una vuelta y así cumplir las 3 vueltas que serían 12 líneas, con el sensor giroscopio(mpu6050) mediremos los grados que va girando el vehículo para calcular exactamente los 90grados del cruce de la esquina. Un planteamiento básico que nos sirvió como comienzo. A partir de aquí presentamos los siguientes problemas. 
•	Falta de detección de líneas: Con el sistema de detección de color tuvimos problemas para detectar las líneas ya que había veces que no la detectaba por la velocidad del vehículo por lo que aumentamos la tasa de refresco a :
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
INTEGRATIONTIME _2_4MS es el tiempo de integración, que es el tiempo que la carama recoge luz para hacer una lectura y el Gain_1X es la ganancia que es un factor de amplificación que se aplica a la señal de los fotodiodos antes de la conversión analógica a digital.

•	Sobre detección y detección errónea: Detectaba la línea más de una vez y en algunas zonas por la diferencia de luz podía tomar un color distinto como azul o naranja, por lo que desarrollamos una lógica que saca el promedio de las ultimas 3 mediciones para garantizarnos que si era la línea azul y que no tomara el comando mas de una vez.

•	Desviación: Luego se utilizó un sistema de centrado con 2 sensores ultrasónicos laterales para corregir el problema de que el vehículo no quedaba exactamente con 90 grados al momento después de terminar el cruce. Este sistema compara ambas distancias e intenta mover el servomotor para que cruce en dirección al lado con mayor distancia. Esto luego fue robustecido con un sistema PID que hace correcciones diferentes según el nivel de desviación para una corrección leve si la desviación es poca o fuerte si es grande. 
El ángulo de cruce que el mpu suma al ángulo actual fue modificado de 90 a 70 ya que el sensor giroscopio cuando llegaba su medición a 90 grados el vehículo cruzaba de más por lo que fuimos calibrando manualmente este valor hasta 70 grados con el que el vehículo se logró desempeñar mejor en las pruebas. Este valor puede ser distinto dependiendo de la calibración o posición de montaje del sensor por eso es mejor calibrarlo cada quien en sus montajes mediante las pruebas.

Para culminar con este código realzamos la lógica necesaria para cumplir con la condición de que el vehículo girara en ambos independientemente (horario y antihorario). Por lo que pensamos en una lógica para que el vehículo dependiendo de la línea que se encontrase primero sería el sentido de la vuelta si es azul es horario y si es naranja es antihorario. Para la lógica de antihorario seria restar 70 grados al ángulo actual y girar a la izquierda en lugar de la derecha como en la lógica pasada. 

### Diagrama de logica de vueltas a la pista.
 ![Diagrama del codigo de vueltas a la pista](other/Diagrama_codigo_vueltas.png)

### Diagrama de logica PID del centrado y giro.
 ![Diagrama del codigo de vueltas a la pista](other/diagrama_PID_giro_y_centrado.png)

## Desarrollo de la logica de evasion de objetos.

### Primer esquive:
Ya con app lista desarrollamos una lógica para el microcontrolador Arduino y de esta manera recibir los comandos de color del objeto y su distancia. Realizando una prueba controlada tuvimos unos errores con la iluminación que nos afectaba el reconocimiento de los colores por que tuvimos que ajustar los valores de los colores para mejorar la detección del objeto que termino siendo mejor pero todavía le faltan mejoras, pero cumplió con el objetivo de identificar el objeto y darnos su distancia aproximada, que a 30 cm el Arduino deberá realizar el giro a la derecha si es rojo(R) y a la izquierda si es verde(G), con un retorno al lado contrario luego de realizar el giro para reincorporarse al centro del carril de la pista. 

   ### Orientacion hacia el objeto para posterior esquive.
Problema: El vehículo cuando no está perfectamente en dirección al objeto este ejecutara el esquive pero lo derribara o simplemente perderá de vista el objeto ya que su dirección no va hacia el, por tal razón pensamos en una lógica para que dependiendo de la posición del objeto en la cámara de la del teléfono mandara comandos como C(centro), D(derecha) y I(izquierda), de esta manera ir moviendo de forma controlada la dirección según el comando recibido apenas se capte el objeto en la visión de la cámara. Con este cambio realizado el vehículo se posiciona de manera adecuada frente al objeto a esquivar y realiza su movimiento preciso para esquivar durante 1 segundo y al retornar al centro durante 1 segundos.

Problema: al momento de retornar al centro puede que haya un objeto fuera de ranga de la cámara por que al momento de esquivar si se veía pero este modo no es capaz de comenzar el seguimiento así que se le agregó que si detecta otro objeto mientras este en modo de retorno cambie a modo de seguimiento nuevamente. Ademas se le aumento el modo de retorno para que haga un mapeo al area por si no llega a identificar un objeto.

Aquí dejo el paso a paso de la realización de la app:

### Diagrama de logica de evasion de objetos
 ![Diagrama del codigo de evasion de objetos](other/diagrama_esquive.png)

# Desarrollo de Códigos Combinados

---

## Para este momento necesitamos una lógica que nos permita:

1.  **Un botón de inicio:** Un botón que energice el vehículo y programe el inicio del código al enviar una señal.

2.  **Detección y conteo de líneas:** Girar en la dirección correcta al detectar una línea y llevar el conteo de las mismas para saber las vueltas.

3.  **Recepción de comandos de la app:** Recibir comandos de la aplicación cuando se detecta un objeto en la pista, incluyendo su color y posición.
    * **Orientación:** Orientarse para quedar al frente del objeto.
    * **Esquive:** Esquivarlo dependiendo de su color (Rojo por derecha, Verde por izquierda).

4.  **Centrado del vehículo:** Centrar el vehículo según las paredes exteriores cuando no esté recibiendo comandos de objetos detectados.

5.  **Estacionamiento:** Estacionarse en paralelo en el área indicada al final del recorrido.

---

## Con nuestros objetivos claros, desarrollamos una máquina de estados capaz de abarcar todo lo requerido y cambiando de estado según las condiciones impuestas:

### ESTADO_NORMAL $\leftrightarrow$ SIGUIENDO_OBJETO $\leftrightarrow$ ESQUIVANDO

---

### ESTADO_NORMAL:

En este modo abarcamos todos los objetivos del código 1 de vueltas a la pista, donde detectamos las líneas que nos indican el cruce. Mientras no estemos evadiendo objetos, estamos centrando el vehículo dependiendo de la distancia con las paredes laterales. Además, añadimos la recepción de comandos de la aplicación para indicarnos que hay un objeto, lo cual nos hace cambiar al estado "SIGUIENDO_OBJETO".

* **Navegación básica siguiendo líneas:**
    * Detección de líneas de colores (azul/naranja).
    * Centrado automático en el pasillo usando sensores ultrasónicos.
* Recepción de comandos de la aplicación.

#### Transiciones:

* **A SIGUIENDO_OBJETO:** Cuando recibe el comando 'R' o 'G' por la aplicación.
* Permanece en estado hasta nueva detección.

---

### SIGUIENDO_OBJETO:

En este estado, orientamos la dirección del vehículo con el servomotor en función de los comandos recibidos por la aplicación, ya sea a su orientación izquierda (I), derecha (D) o centro (C). Cuando se detecta que el comando de distancia del objeto es menor o igual a 30 cm, se realizará el cambio al estado "ESQUIVANDO". Si el objeto se pierde de vista por alguna razón, volveremos al estado "NORMAL".

* Orienta el servo según la posición del objeto detectado.
* Mantiene el motor activo para seguir el objetivo.
* Procesa la orientación del objeto (Izquierda/Derecha/Centro).

#### Transiciones:

* **A ESQUIVANDO:** Cuando el objeto está muy cerca ($\le 30 \text{cm}$).
* **A ESTADO_NORMAL:** Cuando pierde el objeto (comando 'N').

---

### ESQUIVANDO:

* Ejecuta la secuencia de esquive en 6 etapas:
    * **Case 0: Orientación inicial (300ms):** Servo hacia el lado de esquive.
    * **Case 1: Preparación (tiempo variable):** Activación del motor.
    * **Case 2: Esquive activo (1200ms):** Avance lateral evitando el obstáculo.
    * **Case 3: Reorientación interrumpible (300ms):** Giro hacia el lado contrario para retorno.
    * **Case 4: Retorno interrumpible (3000ms):** Avance de vuelta al curso original.
    * **Case 5: Finalización (100ms):** Centrado y retorno al estado normal.
* Los "Case 3" y "Case 4" son interrumpibles si se detecta un objeto mientras se está retornando para cambiar al estado "SIGUIENDO_OBJETO".

#### Transiciones:

* **A SIGUIENDO_OBJETO:** Durante las etapas 3-4 si detecta un nuevo objeto.
* **A ESTADO_NORMAL:** Al completar la secuencia de esquive.


## Cambio de logica de esquive version 2.0 sistema de carriles.

  ### Funcion realizarGiro():
  Para este momento nos imaginamos un caso mejor donde el esquive fuera mas preciso, anteriormente esabamos implementardo un sistema para girar que consistia en que el vehiculo dependiendo de la distancia lateral giraba mas o menos para luego aplicar un retroceso post giro a ciegas. Esto era ineficiente por que a ciencia cierta nunca sabremos donde terminara el vehiculo por lo que ahora ese retroceso post giro sera controlado por el angulo llegando a un angulo recto dependiendo del conteo de lineas que lleve y el sentido del vehiculo todo gracias a la funcion actualizarGiro que nos proporciona el angulo, un ejemplo de ello con fragmento del codigo:

<img width="728" height="522" alt="calculo_angulo_retroceso" src="https://github.com/user-attachments/assets/b56db8b0-9f01-4cac-bd8f-de8faac6f162" />

Ahora que nos posicionamos siempre en un angulo correcto el vehiculo puede retroceder y con el sensor ultrasonico trasero determinar una distancia suficiente para proceder con los esquives.

### Explicacion de la nueva logica:
Ahora en lugar de hacer un mismo metodo de esquive para cualquier objeto que identifiquemos haremos un nuevo metodo, gracias a la mejora de la aplicacion podemos identificar 2 objetos a la vez y sus posiciones por lo que dependiendo se como sea la configuracion de los mismos podremos hacer un esquive programado indepediente y unico, de esta manera nos aseguramos de tomar cada caso por separado y con mayor eficacia. 

### Máquina de Estados para Navegación por Carriles
La lógica del vehículo se gestiona mediante una máquina de estados que transita entre diferentes modos operativos. Este sistema reemplaza el antiguo modelo de "Seguir/Esquivar" por uno más adecuado para la navegación en pista.

ESTADO_NORMAL ↔ ESPERANDO_COMANDO_CARRIL → EVALUANDO_POSICION → GIRANDO_HACIA_CARRIL → MANTENIENDO_CARRIL


#### ESTADO_NORMAL:
Este es el estado inicial y el modo por defecto después de completar un giro en una intersección.

Avanza en línea recta mientras utiliza la función centrarVehiculo() para mantenerse estable en el pasillo con una vigilancia activa, su objetivo principal es buscar la siguiente línea de color con la función detectarColor(). Al encontrar una, inicia la secuencia de giro (realizarGiro) y el ciclo de navegación se repite.

##### Transición: Si no detecta una línea, puede pasar a ESPERANDO_COMANDO_CARRIL después de la maniobra post-giro.

### ESPERANDO_COMANDO_CARRIL:
En este estado, el vehículo se detiene y espera instrucciones externas.

El motor se apaga y el sistema monitorea activamente el puerto serie para recibir un comando de carril.

#### Transición: Al recibir un comando válido, cambia a EVALUANDO_POSICION.

### EVALUANDO_POSICION:
Estado de transición muy breve para planificar el cambio de carril.

Lee los sensores ultrasónicos laterales para determinar su posición actual en relación con el carril objetivo. Decide la dirección del giro necesario para alcanzar el nuevo carril.

#### Transición: Inmediatamente cambia a GIRANDO_HACIA_CARRIL.

### GIRANDO_HACIA_CARRIL:

 Aqui aplicaremos una nueva logica donde dependiendo de las distancias laterales nos dameremos cuenta en que parte de la pista nos encontramos, dividiendo la pista en 2 carriles principales y 2 secundarios. Los principales son:
 1. Carril 1: A la derecha del vehiculo. Usado para los objetos rojos.
 2. Carril 2: A la izquierda del vehiculo. Usado para los objetos verdes.
 3. Carril 1 pero a mas distancia de la pared
 4. Carril 2 a mas distancia de la pared.
Dependiendo de que objeto tengamos al frente tomaremos un carril. Hay doble objeto se ejecuta un cambio de carril apenas se consiga entrar en el estado donde mantiene el carril, esto se hace de la misma manera que se dirige al carril en un inicio solo que un poco mas de tiempo. este tiempo depende del comando en especifico.

#### Posicionamiento en un carril:
Dependiendo de a que carril deseemos dirigirnos usaremos diferentes sensores:
  Carril 1: Sensores derechos Ultrasonico e infrarrojo.
  Carril 2: Sensores izquierdos Ultrasonico e infrarrojo.
  
Para realizar el posicionamiento recurrimos al estado GIRANDO_HACIA_CARRIL, donde mediantes algunas funciones tendremos un giro hacia el carril correspondiendo con cierta inclinacion de la direccion y un tiempo establecido que dependen de la distancia lateral que alla con la pared externa de la pista. Al finalizar pasaremos a un giro de retorno en sentido contrario que terminara cuando el angulo sea recto. 

Durante el primer giro estaremos esperando de igual manera una retroalimentacion del sensor sharp correspondiente para que nos confirme si estamos a una buena distancia de la pared lateral, si estamos muy cerca interrumpira el primer giro dara un breve retroceso y procedera con el giro de retorno. 

#### Transición: Una vez que el vehículo está orientado en el nuevo carril, cambia a MANTENIENDO_CARRIL.

### MANTENIENDO_CARRIL:
Este es el estado de navegación principal entre intersecciones.

Utiliza un controlador PID para mantener de forma activa y precisa la distancia con la pared lateral (izquierda o derecha, según el carril). Esto le permite seguir el contorno de la pista con gran exactitud.

Control Fino: El sistema prioriza mantener el ángulo correcto; si el vehículo se desvía demasiado, corrige primero su orientación y luego ajusta la distancia a la pared.

#### Transición: Permanece en este estado hasta que detectarColor() encuentra una línea, lo que interrumpe el mantenimiento del carril e inicia un giro de intersección, devolviendo el sistema al ESTADO_NORMAL.

### ESTADO_DETENCION_FINAL:
El estado final del recorrido. Se activa después de que se cuenta la última línea y se completa el último giro. El vehículo avanza en línea recta durante un tiempo predefinido (1.5 segundos) para posicionarse en la zona de estacionamiento.

#### Transición: Al finalizar el avance, llama a detenerRobot() para apagar todos los motores y finalizar la operación.
 
 ### Diagrama logica de esquive 2.0 sistema de carriles
 ![Diagrama de logica carriles](other/diagramaV2.png)
## Video resumen de las pruebas de vuelta a la pista y esquive de objetos.

Haz clic en la imagen para ver el vídeo:

[![Mira el video del proyecto](other/video_promocional.png)](https://youtu.be/E-xsGcDSaJY?si=9i87w-iJ8TijE_ca)


## Prueba de vueltas a la pista

Haz clic en la imagen para ver el vídeo:

[![Mira el video del proyecto](other/prueba-vuelta.png)](https://youtube.com/shorts/wI_OLS1nWhc?si=868zlHss_CnnXe10)

## Prueba de evacion de obstaculos

Haz clic en la imagen para ver el vídeo:

[![Mira el video del proyecto](other/prueba-evacion-obstaculos.png)](https://www.youtube.com/watch?v=WQM6CjVKSwE)

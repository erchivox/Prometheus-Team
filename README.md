# Prometheus Team

Somos un equipo venezolano que representa con orgullo a la **Universidad Politécnica Territorial José Félix Ribas (UPTJFR)**. Nuestro compromiso es la innovación y el aprendizaje continuo en el campo de la robótica autónoma.

---

##  Índice

* [Contenido del Repositorio](#-contenido-del-repositorio)
* [Introducción al Proyecto](#-introducción-al-proyecto)
* [Diseño de Hardware](#-diseño-de-hardware)
    * [Componentes de Detección](#-componentes-de-detección)
    * [Componentes de Procesamiento de Información](#-componentes-de-procesamiento-de-información)
    * [Componentes de Alimentación](#-componentes-de-alimentación)
* [Programas para Detección de Objetos](#-programas-para-detección-de-objetos)
* [Cálculo de Torque y Velocidad](#-cálculo-de-torque-y-velocidad)
* [Sistema de Alimentación](#-sistema-de-alimentación)
    * [Cálculo del Consumo Energético Total](#-cálculo-del-consumo-energético-total)
    * [Autonomía Estimada](#-autonomía-estimada)
* [Sistema de Detección de Objetos](#-sistema-de-detección-de-objetos)
* [Paso a Paso de la Construcción](#-paso-a-paso-de-la-construcción)
    * [Etapa de Diseño y Ensamblaje del Vehículo](#-etapa-de-diseño-y-ensamblaje-del-vehículo)
    * [Montaje del Vehículo](#-montaje-del-vehículo)
    * [Cambio de Base](#-cambio-de-base)
* [Pruebas Realizadas](#-pruebas-realizadas)
    * [Prueba del Sensor de Color](#-prueba-del-sensor-de-color)

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
| ![Sensor Ultrasónico](other/sensor_ultrasonico.jpg) | **Sensor Ultrasónico HC-SR04** | Mide la distancia a obstáculos mediante ondas ultrasónicas. Se utiliza frontal y lateralmente para detectar proximidad. |
| ![Sensor Infrarrojo](other/sensor_infrarojo.jpg) | **Sharp GP2Y0A21YK0F Sensor Infrarrojo** | Garantiza una detección de objetos rápida y precisa. Se ubican en las diagonales delanteras del vehículo. |
| ![Sensor Color](other/sensor_color.png) | **TCS34725 Sensor de Colores** | Permite una detección precisa del color a corta distancia, ubicado en la parte inferior para identificar los colores de las franjas de la pista. Alta sensibilidad y amplio rango dinámico (3.800.000:1), funcionando incluso tras cristales oscuros. |
| ![Giroscopio](other/sensor_giroscopio.jpg) | **MPU6050 Sensor Giroscopio y Acelerómetro** | Unidad de medición inercial (IMU) de 6 grados de libertad (DoF) que combina un acelerómetro y un giroscopio de 3 ejes. Ampliamente utilizado en navegación, radiogoniometría y estabilización. |
| ![Cámara Celular](other/celular.png) | **Cámara del Teléfono Celular** | Utilizando un programa con OpenCV y Android Studio, la cámara del celular se usa para identificar el color de obstáculos a gran distancia. |

### Componentes de Procesamiento de Información

| Imagen | Nombre de Componente | Descripción |
| :----: | :------------------- | :---------- |
| ![Arduino](other/arduino.jpeg) | **Microprocesador Arduino Uno** | Actúa como el cerebro del vehículo, conectando y controlando todos los sensores y motores. |

### Componentes de Alimentación

| Imagen | Nombre de Componente | Descripción |
| :----: | :------------------- | :---------- |
| ![Batería](other/bateria_lipo.png) | **Batería LiPo 2200mAh 7.4V** | Fuente de alimentación recargable que alimenta todo el sistema, proporcionando movilidad y autonomía. Seleccionada por su capacidad para suministrar suficiente energía a los motores y componentes internos. |
| ![Regulador](other/Modulo_regulador_alimentacion_LM2596S.jpg) | **LM2596 Regulador de Voltaje** | Reduce el voltaje de manera eficiente. Voltaje de entrada: 4.5 V a 40 V CC. Voltaje de salida: 1.23 V a 37 V CC. Corriente de salida: Máx. 3 A, se recomiendan 2.5 A. |

---

##  Programas para Detección de Objetos

![OpenCV Logo](other/openCV.png)

Para la detección de obstáculos, hemos desarrollado una **aplicación móvil con Android Studio** que implementa la biblioteca **OpenCV**. Esta aplicación mejora la capacidad de detección del entorno del robot, identificando objetos rojos y verdes en tiempo real mediante la cámara del dispositivo móvil. Una vez que la aplicación identifica el color predominante, envía esta información a la placa Arduino a través de una conexión USB.

![Android Studio Logo](other/andriod_studio.PNG)

### Flujo del Proceso:

1.  **Captura de Imagen:** La aplicación utiliza la cámara del teléfono para capturar vídeo en tiempo real.
2.  **Procesamiento con OpenCV:** La imagen se filtra por color para identificar la presencia de objetos rojos o verdes en la escena.
3.  **Comunicación con Arduino:** Una vez identificado el color, la aplicación envía un código o carácter específico a la placa Arduino (ej., "R" para rojo, "G" para verde).
4.  **Toma de Decisiones:** Al recibir esta información, Arduino ejecuta una acción preprogramada.

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

![Diagrama Fuente de Poder](schemes/power_supply_diagram.jpeg)

En este esquema, toda la energía proviene de nuestra **batería LiPo de 2200 mAh** con un voltaje de 7.4 voltios. La batería se interconecta con un interruptor principal que indica el encendido del vehículo.

Luego, una rama se dirige al **regulador de voltaje XL6009**, que proporciona una salida directa de 7.5 voltios al **módulo L298N** (un puente H con control de rotación). Este puente H tiene una caída de voltaje de 1.5 voltios, por lo que la salida conectada a nuestro motor DC le suministrará aproximadamente 6 voltios, su voltaje de funcionamiento normal.

Adicionalmente, derivamos la batería LiPo para alimentar la placa **Arduino**. Este voltaje se regula mediante un **módulo DSN-Mini 360** configurado a 6 voltios. Esta salida se conecta directamente al pin Vin de la placa Arduino, donde también conectamos el servomotor que controla la dirección del vehículo.

### Cálculo del Consumo Energético Total

| Componente | Cantidad | Consumo estimado (mA) | Total (mA) |
| :------------------------- | :------- | :-------------------- | :--------- |
| Motor DC | 1 | 500 mA (en carga) | 500 mA |
| Servo 180° | 1 | 150 mA (típico) | 150 mA |
| Sensor ultrasónico HC-SR04 | 3 | 15 mA c/u | 45 mA |
| Sensor infrarrojo Sharp | 2 | 30 mA c/u | 60 mA |
| Sensor TCS34725 (color) | 1 | 3 mA | 3 mA |
| Arduino Uno | 2 | 50 mA c/u (sin carga) | 100 mA |
| **TOTAL** | — | — | **858 mA** |

---

### 🔋 Corriente total aproximada: **~858 mA**

---

### ⏱️ Autonomía Estimada

**Fórmula:**
`Autonomía (h) = Capacidad de la batería (mAh) / Consumo total (mA)`

**Ejemplo con batería de 2200 mAh:**
`Autonomía ≈ 2200 mAh / 858 mA ≈ 2.56 horas`

> ⚠️ *Nota:* Este valor es teórico y asume un consumo constante. En la práctica, el consumo del motor puede aumentar significativamente si se encuentra con un obstáculo, por lo que la autonomía real podría variar.

---

##  Sistema de Detección de Objetos

![Diagrama de Sensores](schemes/diagrama-sensores.jpg)

Cada uno de nuestros sensores desempeña funciones específicas para guiar el vehículo y asegurar que no impacte con ningún obstáculo:

* Los **sensores infrarrojos** están ubicados en las diagonales del vehículo, proporcionando una detección rápida y segura para evitar colisiones sorpresa.
* Los **sensores ultrasónicos** se encuentran al frente y en los laterales del vehículo, midiendo distancias largas para mantener nuestros márgenes de movimiento con respecto a las paredes laterales y la frontal.
* El **sensor giroscopio** nos ayuda a mantener una trayectoria recta a través de la pista y a realizar giros precisos, guiándonos según nuestros grados iniciales.
* Finalmente, nuestro **sensor de color** identifica las líneas en el mapa, permitiéndonos recorrer la pista de manera eficiente.

---

##  Paso a Paso de la Construcción

### Etapa de Diseño y Ensamblaje del Vehículo

El diseño de nuestro vehículo comenzó con un **modelo 3D en Tinkercad** para definir las dimensiones y el tamaño de cada componente. Este modelo se concibió para un vehículo de dos plantas:

* **Planta Baja:** Aloja el sistema de transmisión, sistema de dirección, motor, sistema de potencia y sensores (frontales, laterales y diagonales).
* **Segundo Piso:** Contendrá el Arduino y la cámara (originalmente no se pensó en un teléfono celular, pero se consideró la idea en esta etapa). La imagen muestra cómo un teléfono celular interactuaría con nuestro vehículo.

| Vista Frontal | Vista Base | Vista Lateral |
| :-----------: | :--------: | :-----------: |
| ![Modelo 3D Frontal](schemes/front_side_3d_design.png) | ![Modelo 3D Base](schemes/side_3d_design_4.png) | ![Modelo 3D Lateral](schemes/side_3d_design_2.png) |

### Montaje del Vehículo

| Imagen | Descripción |
| :----: | :---------- |
| ![Molde Base](other/base1.jpg) | Aquí se muestran algunas medidas ya probadas para las primeras pruebas del coche. Utilizamos cartón como material de prueba para realizar cambios sin incurrir en costos adicionales. La base se cubrió con una doble capa de cartón para mayor rigidez. |
| ![Instalación de Transmisión](other/montaje.jpg) | Colocamos la transmisión LEGO en la base y realizamos los cortes necesarios para que el motor, la dirección y la transmisión encajaran correctamente. Luego, instalamos el servomotor en el diferencial y el motor DC en la transmisión. |
| ![Instalación de Servo](other/instalacion_servo_direccion.jpg) | ![Instalación de Motor DC](other/instalacion_motor_transmision.jpg) |
| ![Primera Estructura](other/primera_estructura.jpg) | Con todos los elementos esenciales ensamblados, conectamos el motor a la alimentación con el regulador para una prueba de funcionamiento rápida, verificando el correcto desempeño del motor y la transmisión. |

### Cambio de Base

| Imagen | Descripción |
| :----: | :---------- |
| ![Segunda Base Impresión 3D](other/1.jpg) | Realizamos un cambio del material de la base. El cartón fue el material de prueba para modificar el diseño de todas las maneras posibles. Con el molde final de nuestro vehículo, procedimos a imprimir la base en 3D. |
| ![Montaje de la Nueva Base](other/proceso.jpg) | Inicio del proceso de cambio de base, montando la transmisión y el diferencial del vehículo. |
| ![Montaje de la Nueva Base 2](other/proceso-2.jpg) | ![Montaje de la Nueva Base 3](other/proceso-3.jpg) |
| ![Montaje de la Nueva Base 4](other/proceso-4.jpg) | Se continúa con el ensamblaje de la nueva base. |

---

## Pruebas Realizadas

Para las pruebas, creamos una imitación de la pista de competición utilizando láminas recicladas, pegadas para cumplir con los estándares de 3x3 metros. Nuestra pista tiene un ligero error de aproximadamente 10 cm. Las esquinas de la pista deben tener dos líneas, una azul y otra naranja, separadas por 30 grados cada una, tomando el mismo origen que la esquina interior del cuadrado.

![Pista de Pruebas](other/pista.jpg)

### Prueba del Sensor de Color

Haz clic en la imagen para ver el vídeo:

[![Mira el video del proyecto](other/portada_pruebas.png)](https://youtu.be/7GS6mCXmGks)

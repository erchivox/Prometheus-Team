Prometheus Team
====

Equipo venezolano representando a la Universidad Politécnica Territorial José Félix Ribas (UPTJFR).

## Contenido

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## Introducción
Para esta competición, trabajamos en un diseño cómodo, fácil de modificar y a la vez sencillo, donde todos los elementos interactúan a la perfección. Presentamos nuestro primer prototipo de vehículo autónomo con reconocimiento de objetos y colores.

El diseño y la construcción de nuestro vehículo son íntegramente propios, con una estructura de dos niveles diseñada para que todos los componentes encajen perfectamente, vigas de soporte extraíbles para facilitar su desmontaje si es necesario, un sistema de transmisión para transmitir la potencia del motor conectado al eje a las dos ruedas traseras del vehículo, fabricado con piezas tipo LEGO, y un sistema de dirección con el mismo tipo de piezas, controlado por un servomotor para una mayor precisión al cruzar.

Este proyecto fue llevado a cabo por nuestro equipo; cada cable, cada sensor, cada línea de código fue posible gracias a su esfuerzo técnico y financiero. No solo construimos un vehículo; demostramos que con pasión y ganas de aprender, se pueden superar los límites.

  
## Diseño de harware.
![control principal](other/primer_nivel.jpg)

Para esta primera fase de nuestra primera montura 
| Imagen | Nombre de componente | Descripcion |
|:------:|:----------------------|:------------|
| ![base del primer piso](other/base.jpg) | **Base del Vehiculo** | La base de este vehículo con unas medidas de 11,5 y 2,5 cm está pensada en imitar un auto de carreras de Fórmula 1, con eso en mente ajustamos las dimensiones de nuestra base para una mejor distribución del espacio y movilidad del vehículo dando espacio suficiente tanto al sistema de dirección del vehículo como al de transmisión y de los componentes y módulos electrónicos seleccionados para el vehículo. Parte delantera: esta se realizó de manera que se pudieran colocar los 3 sensores que teníamos pensado colocar para que el vehículo confirme que tiene objetos al frente del él (1 ultrasónico frontal y 2 infrarrojos en las diagonales). Su parte más débil es donde se coloca la dirección, ya que necesita el espacio suficiente para mover correctamente el ángulo de sus ruedas delanteras al momento de ejecutar el cruce, pero esto ya depende del tipo de material, ya que como se empezó con cartón si nos dio algunos problemas, pero si trabajas directamente con una lámina de acrílico o un material más rígido sería lo mejor. |
| ![transmision](other/transmision.jpg) | **Sistema de transmision de Legos** | Permite distribuir la potencia del motor entre ambas ruedas traseras para que el motor pueda seguir girando aunque alguna de las ruedas se bloquee, evitando que el motor sufra algún daño. Escogimos este sistema por su facilidad de uso y su tamaño adecuado a lo que teniamos pensando para nuestro vehiculo, permitiendo tambien la reversa de ser necesario. |
| ![Motor](other/encaje_motor.jpg) | **25GA370 Motor dc con encoder** | Este es el tipo de motor normalmente usado para estos vehiculos, por su potencia y velocidad de giros por segundo. El codificador integrado mide la velocidad y la dirección de rotación para un mejor control del movimiento pero hasta ahora no a sido utilizado. Especificaciones:( Potencia nominal: 4 W, Tensión nominal: 6V, Velocidad nominal: 220 RPM, Peso: 400g, caja reductora: 21.3:1, torque nominal: 0.35 kg·cm) |
| ![Direccion](other/direccion.jpg) | **Sistema de direccion** | Este sistema contruido mediante piezas legos fue escogido por su tamaño adecuado y su capicidad sencilla de adaptarle un servomotor de 180 grados para controlar el movimiento del vehiculo, Sacandole su maximo giro con el servomotor en 180 y 0 grados. El servo esta mirando hacia el vehiculo por lo que seria sus 90 grados para centrarse, 180 para girar a la izquierda y 0 grados para girar a la derecha.|
| ![separedores](other/separadores.jpg) | **Separadores de acrilico** | Para un montaje cómodo y fácil de modificar, se utilizaron estos espaciadores, donde solo hay que girar los pilares para desatornillarlos y desmontar el vehículo. |

## Componentes de decteccion.
| Imagen | Nombre de componente | Descripcion |
|:------:|:----------------------|:------------|
| ![Sensor Ultrasonico](other/sensor_ultrasonico.jpg) | **Sensor ultrasonico HC-SR04** | Mide la distancia a obstáculos mediante ondas ultrasónicas. Se utiliza frontal y lateralmente para detectar la proximidad. |
| ![Sensor Infrarrojo](other/sensor_infrarojo.jpg) | **Sharp GP2Y0A21YK0F Sensor infrarrojo** | Garantiza una detección de objetos rápida y precisa. Se ubican en las diagonales delanteras del vehículo. |
| ![Sensor Color](other/sensor_color.png) | **TCS34725 Sensor de colores** |Permite una detección precisa del color a corta distancia, por lo que se coloca en la parte inferior del vehículo e identifica los colores de las franjas de la pista. Tiene una alta sensibilidad y un amplio rango dinámico de 3.800.000:1, y puede funcionar incluso tras cristales oscuros. |
| ![Cámara](other/sensor_giroscopio.jpg) | **MPU6050 Sensor giroscopio y acelerometro** | El MPU6050 es una unidad de medición inercial (IMU) de 6 grados de libertad (DoF) que combina un acelerómetro y un giroscopio de 3 ejes. Este sensor se utiliza ampliamente en navegación, radiogoniometría, estabilización y más. |
| ![Cámara](other/celular.png) | **Camara del telefono celular** | Utilizando un programa que utiliza OpenCV y Android Studio, utilizaremos la cámara del celular para identificar el color de obstáculos a gran distancia. |

## Componentes de procesamiento de informacion.
| Imagen | Nombre de componente | Descripcion |
|:------:|:----------------------|:------------|
| ![Arduino](other/arduino.jpeg) | **Microprocesador Arduino Uno** | Actúa como el cerebro del vehículo, conectando y controlando todos los sensores y motores. |

## Componentes de alimentacion. 
| Imagen | Nombre de componente | Descripcion |
|:------:|:----------------------|:------------|
| ![Batería](other/bateria_lipo.png) | **Bateria de lippo 2200mah 7.4v** | Fuente de alimentación recargable que alimenta todo el sistema, proporcionando movilidad y autonomía. Seleccionada por su capacidad para suministrar suficiente energía a los motores y componentes internos.|
| ![regulador](other/Modulo_regulador_alimentacion_LM2596S.jpg) | **LM2596 Regulador de voltaje** | Reduce el voltaje de la manera más eficiente. Voltaje de entrada: 4,5 V a 40 V CC. Voltaje de salida: 1,23 V a 37 V CC. Corriente de salida: Máx. 3 A, se recomiendan 2,5 A. |


## Programas para deteccion de objetos.
 ![openCV](other/openCV.png) 
 Para la detección de obstáculos, diseñamos una aplicación móvil con Android Studio e implementamos la biblioteca OpenCV, con el objetivo de mejorar la capacidad de detección del entorno del robot. La función principal de esta aplicación es detectar objetos rojos y verdes en tiempo real mediante la cámara del dispositivo móvil. Una vez que la aplicación identifica el color predominante en el campo de visión, envía esta información a la placa Arduino mediante una conexión USB.
 
![Android studio](other/andriod_studio.PNG)

Flujo del proceso:
1. Captura de imagen: La aplicación utiliza la cámara del teléfono para capturar vídeo en tiempo real.

2. Procesamiento con OpenCV: La imagen se filtra por color para identificar si hay un objeto rojo o verde en la escena.

3. Comunicación con Arduino: Una vez identificado el color, la aplicación envía un código o carácter específico a la placa Arduino (p. ej., «R» para rojo, «G» para verde).

4. Toma de decisiones: Al recibir esta información, Arduino ejecuta una acción preprogramada.


## Cálculo de Torque y Velocidad.

  #### Calculo de torque necesario para mover el vehiculo:
  Tnecesario = m.g⋅r=0.943⋅9.81⋅0.04= 0.370N\cdotpm
  m= masa
  g= gravedad
  r= radio de las ruedas.

#### Calculo de torque a la salida (después de la reducción) segun el motor dc 25GA370:
```
 𝑇salida = 𝑇motor⋅Reduccion =0.0343 . 21.3= 0.7306 N\cdotpm 
```
 
#### Calculo de la velocidad lineal del vehiculo:

  Convertimos a radianes por segundo:
 ```
    ω = (220 . 2π)/60=23,04𝑟𝑎𝑑/𝑠
  ```
  Velocidad lineal del vehículo:
  
  ```
   v=ω⋅r=23.04⋅0.04 ≈ 0.92m/s
  ```



  El motor empleado tiene una velocidad sin carga de 4690 RPM y un torque nominal de 0.35 kg·cm (0.0343 N·m). Mediante una caja reductora de 21.3:1, la velocidad       se reduce a 220.2 RPM en el eje de salida, resultando en una velocidad lineal del vehículo de aproximadamente 0.92 m/s.

  Gracias a esta reducción, el torque en las ruedas alcanza 0.7306 N·m, lo cual supera el torque mínimo necesario para mover el vehículo de 943 gramos (0.370 N·m).     Por tanto, el sistema cumple adecuadamente los requerimientos de tracción y movilidad para condiciones normales.


## Sistema de alimentacion.

![openCV](schemes/power_supply_diagram.jpeg) 
En este esquema, tomamos toda la energía de nuestra batería LiPo de 2200 mAh con un voltaje de 7.4 voltios. La interconectamos con un interruptor principal que indica el encendido del vehículo. Luego, pasamos una rama al regulador de voltaje XL6009 con una salida directa de 7.5 voltios al módulo L298N, un puente H con control de rotación, que tiene una caída de voltaje de 1.5 voltios. La salida de este puente H está conectada directamente a nuestro motor de CC y le llegarán aproximadamente 6 voltios, que sería su voltaje de funcionamiento normal.
Luego, derivamos la batería LiPo para alimentar la placa Arduino. Este voltaje se regula mediante un módulo DSN-Mini 360 configurado a 6 voltios. Esta salida se conecta directamente al pin Vin de la placa Arduino, donde también conectamos el servomotor que controla la dirección del vehículo.

###  Cálculo del Consumo Energético Total

| Componente                   | Cantidad | Consumo estimado (mA) | Total (mA) |
|-----------------------------|----------|------------------------|------------|
| Motor DC                      | 1        | 500 mA (en carga)      | 500 mA     |
| Servo 180°                  | 1        | 150 mA (típico)        | 150 mA     |
| Sensor ultrasónico HC-SR04 | 3        | 15 mA c/u              | 45 mA      |
| Sensor infrarrojo Sharp     | 2        | 30 mA c/u              | 60 mA      |
| Sensor TCS34725 (color)     | 1        | 3 mA                   | 3 mA       |
| Arduino Uno                 | 2        | 50 mA c/u (sin carga)    | 100 mA      |
| **TOTAL**                   | —        | —                      | **858 mA** |

---

### 🔋 Corriente total aproximada: **~858 mA**

---

### ⏱️ Autonomía estimada

**Fórmula:**
```
Autonomía (h) = Capacidad de la batería (mAh) / Consumo total (mA)
```

**Ejemplo con batería de 2200 mAh:**
```
Autonomía ≈ 2200 mAh / 810 mA ≈ 2.56 horas
```

> ⚠️ *Nota:* Este valor es teorico y si el consumo se mantiene igual sin irregularidades, pero eso nunca pasa el motor puede llegar a consumir mas si te traba en un obstaculo asi que es apoximado.


## Sistema de deteccion de objetos.

![diagrama sensores](schemes/diagrama-sensores.jpg) 
Cada uno de estos sensores tiene funciones específicas. Con las cuales guiaremos nuestro vehículo e garantizaremos que no impacte contra cualquier obstáculo. Los sensores infrarrojos estarán ubicados en las diagonales del vehículo con una detección rápida y segura para evitar colecciones colisiones sorpresas, los sensores ultrasonicos están ubicados al frente y en los laterales del vehículo midiendo a largas distancias e ir manteniendo nuestros márgenes en movimiento en función de las paredes laterales y la del frente. El sensor giroscopio nos ayudará a guiarnos según nuestro grados iniciales para avanzar de una manera recta a través de la pista y poder hacer los giros de manera correcta. Y finalmente nuestro sensor de color que nos identificará las líneas que están en el mapa para así recorrer la pista de una manera eficiente.

## Paso a paso de la construcción.
   ## Vehicle design and assembly stage
   
   En cuanto al diseño de nuestro vehículo, comenzamos a desarrollar un modelo 3D en la plataforma Tinkercad para determinar las dimensiones y el tamaño de cada uno de sus componentes. Este modelo se diseñó para un vehículo de dos plantas, con la planta baja albergando el sistema de transmisión, el sistema de dirección, el motor, el sistema de potencia y los sensores (frontales, laterales y diagonales).
| Front | Base | Side |
|:------:|:----------------------|:------------|
| ![modelo3dVehiculo](schemes/front_side_3d_design.png) | ![modelo3dVehiculo](schemes/side_3d_design_4.png) | ![modelo3dVehiculo](schemes/side_3d_design_2.png) |

  El segundo piso albergará el Arduino y la cámara. En aquel momento, la cámara no estaba pensada para usarse con un teléfono celular, pero se estaba considerando la idea. Esto muestra cómo interactuaría un teléfono celular con nuestro vehículo.
   
## Montaje de vehículos
  | Imagen | Descripción |
|:------:|:----------------------|
| ![mold](other/base1.jpg) | Aquí hay algunas medidas ya probadas para sus respectivas pruebas de coche. Usamos cartón para poder hacer varios cambios si es necesario sin incurrir en costos adicionales. Cubriremos la base con una doble capa de cartón para que no quede demasiado blanda y se dañe con un poco de peso. | 
| ![transmision](other/montaje.jpg) || ![transmision](other/montaje.jpg) |Colocamos la transmisión Lego en la base e hicimos algunos cortes para que encajaran correctamente el motor, la dirección y la transmisión. Luego, instalamos el servomotor en el diferencial y el motor de CC en la transmisión. | 
| ![instalacion de servo](other/instalacion_servo_direccion.jpg) | ![instalacion de motor dc](other/instalacion_motor_transmision.jpg)| 
| ![primera estructura](other/primera_estructura.jpg) |** Ya con todo lo elemental esamblado conectamos el motor a la alimentación con el regulador para hacer una prueba rápida de funcionamiento, para observar si el motor y la transmisión funcionan adecuadamente.**| 

Cambio de base
  | Imagen | Descripción |
|:------:|:----------------------|
| ![segunda base impresion 3d](other/1.jpg) |**  Realizamos un cambio del material de la base ya que el carton es fue el material de prueba para modificarlo de todas las maneras posibles, pero ya con el molde de nuestro vehiculo terminado procedemos a mandar a imprimir en 3d nuestro molde **| 
| ![montaje de la nueva](other/proceso.jpg) |**  Comiendo del cambio de base montando la transmision y diferencial del vehiculo **| 
| ![montaje de la nueva](other/proceso-2.jpg) | ![montaje de la nueva](other/proceso-3.jpg) | 
| ![montaje de la nueva](other/proceso-4.jpg) | ** ** | 

## Pruebas realizadas
  Para hacer las pruebas realizamos una imitacion de la pista para practicar. Construida con laminas recicladas pegadas unas a otras para cumplir con los estandares de la competencia los cuales son que la pista debe tener unas medidas de 3*3metros. Son 8 cuadrados de 100cm. Pero para nuestra pista tenemos un error de mas o menos 10cm. Una pista que en las esquinas debe de tener 2 lineas una azul y otra naranja separadas por 30 grados cada una tomando el mismo origen que es la esquina del cuadrado de adentro.

  ![montaje de la nueva](other/pista.jpg)
  

  
 ## Prueba del sensor de color
  Haga clic en la imagen para ver el vídeo
  [![Mira el video del proyecto](other/portada_pruebas.png)](https://youtu.be/7GS6mCXmGks)



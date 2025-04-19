Prometheus Team
====

Equipo venezolano que esta representando a la Universidad Politecnica Territorial Jose Felix Ribas(UPTJFR).

## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## Introduction
Para esta competencia estamos trabajando en un diseño comodo facil de modificar y a la vez simple donde todos los elementos se relacionen entre si de manera uniforme, con nuestro primer prototipo de vehiculo autonomo con reconocimiento de objetos y colores. 
 
El diseño y contruccion de nuestro vehiculo es totalmente de nuestra autoria contando con una estructura de 2 niveles diseñada para que todos los componentes encajaran adecuadamente, vigas de soporte removibles para un desmontaje facil de ser necesario, sistema de transmisión para transmitir la potencia del motor conectado al eje hacia las dos ruedas traseras del vehículo realizado con piezas de tipo LEGO y un sistema de dirección con el mismo tipo de piezas controlado por un servomotor para una mejor precisión al momento de cruzar.

El proyecto realizado no cuenta con patrocinio externo. Todo está financiado por nuestro equipo de trabajo. Cada cable, cada sensor, cada línea de código ha sido posible gracias al esfuerzo tanto técnico como económico de nuestro propio bolsillo. No solo estamos construyendo un vehículo, estamos demostrando que con pasión y ganas de aprender se pueden romper los límites.


## Diseño del harware
![primer piso](other/motor_encoder.jpg)

| Imagen | Nombre del Componente | Descripción |
|:------:|:----------------------|:------------|
| ![Motor](other/motor_encoder.jpg) | **Motor 25GA370 con Encoder** | Proporciona la tracción del vehículo. El encoder permite medir la velocidad y dirección del giro para un mejor control de movimiento. |
| ![Sensor Ultrasonico](other/ultrasonico.jpg) | **Sensor Ultrasónico HC-SR04** | Mide la distancia a obstáculos usando ondas ultrasónicas. Se usa en el frente y laterales para detectar cercanía. |
| ![Sensor Ultrasonico](other/ultrasonico.jpg) | **Sensor Ultrasónico HC-SR04** | Mide la distancia a obstáculos usando ondas ultrasónicas. Se usa en el frente y laterales para detectar cercanía. |



| ![Sensor Ultrasonico](other/ultrasonico.jpg) | **Sensor Ultrasónico HC-SR04** | Mide la distancia a obstáculos usando ondas ultrasónicas. Se usa en el frente y laterales para detectar cercanía. |
| ![Sensor Infrarrojo](other/infrarrojo.jpg) | **Sensor Infrarrojo** | Detecta líneas o bordes. Ideal para mantenerse dentro de los límites de la pista o seguir trayectorias. |
| ![Sensor Color](other/color_sensor.jpg) | **Sensor de Color TCS34725** | Identifica colores (rojo, verde, etc.). Permite que el vehículo tome decisiones basadas en el color de los obstáculos. |
| ![Cámara](other/camara.jpg) | **Cámara de Visión** | Reconoce colores a distancia, permitiendo anticiparse a obstáculos o señales. Se utiliza para visión avanzada. |
| ![Arduino](other/arduino.jpg) | **Placa Arduino UNO** | Actúa como el cerebro del vehículo, conectando y controlando todos los sensores y motores. |
| ![Batería](other/bateria.jpg) | **Batería de Carrito Infantil** | Fuente de energía recargable que alimenta todo el sistema, brindando movilidad y autonomía. |





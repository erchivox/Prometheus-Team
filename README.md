Prometheus Team
====

Venezuelan team representing the José Felix Ribas Territorial Polytechnic University (UPTJFR).

## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## Introduction
For this competition, we're working on a comfortable, easy-to-modify, yet simple design where all the elements interact seamlessly with each other, featuring our first prototype of an autonomous vehicle with object and color recognition.
 
The design and construction of our vehicle is entirely our own, with a two-level structure designed so that all components fit together properly, removable support beams for easy disassembly if necessary, a transmission system to transmit power from the engine connected to the axle to the two rear wheels of the vehicle made with LEGO-type pieces, and a steering system with the same type of pieces controlled by a servomotor for greater precision when crossing.

This project was carried out by our team; every cable, every sensor, every line of code was made possible thanks to the technical and financial efforts of our team. We're not just building a vehicle; we're demonstrating that with passion and a desire to learn, you can push the boundaries.


## Hardware design
![control principal](other/primer_nivel.jpg)

| Image | Component Name | Description |
|:------:|:----------------------|:------------|
| ![transmision](other/transmision.jpg) | **LEGOS transmission system** | It allows the engine's power to be distributed between both rear wheels so that the engine can continue turning even if some of the wheels lock up, preventing the engine from suffering any damage. |
| ![Motor](other/encaje_motor.jpg) | **25GA370 Motor with Encoder** |It provides the vehicle's traction. The encoder measures the speed and direction of rotation for better motion control. |
| ![Direccion](other/direccion.jpg) | **180° servo-controlled steering system** | It allows control over the vehicle's movement, moving both front wheels with a 180-degree servomotor, which allows us to achieve good precision when avoiding obstacles. |



| ![Sensor Ultrasonico](other/ultrasonico.jpg) | **Sensor Ultrasónico HC-SR04** | Mide la distancia a obstáculos usando ondas ultrasónicas. Se usa en el frente y laterales para detectar cercanía. |
| ![Sensor Infrarrojo](other/infrarrojo.jpg) | **Sensor Infrarrojo** | Detecta líneas o bordes. Ideal para mantenerse dentro de los límites de la pista o seguir trayectorias. |
| ![Sensor Color](other/color_sensor.jpg) | **Sensor de Color TCS34725** | Identifica colores (rojo, verde, etc.). Permite que el vehículo tome decisiones basadas en el color de los obstáculos. |
| ![Cámara](other/camara.jpg) | **Cámara de Visión** | Reconoce colores a distancia, permitiendo anticiparse a obstáculos o señales. Se utiliza para visión avanzada. |
| ![Arduino](other/arduino.jpg) | **Placa Arduino UNO** | Actúa como el cerebro del vehículo, conectando y controlando todos los sensores y motores. |
| ![Batería](other/bateria.jpg) | **Batería de Carrito Infantil** | Fuente de energía recargable que alimenta todo el sistema, brindando movilidad y autonomía. |





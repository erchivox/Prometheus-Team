# Software del Vehículo Autónomo WRO Prometheus

## Código del Microcontrolador (Arduino)

El software del microcontrolador para nuestro vehículo autónomo ha sido desarrollado utilizando el IDE de Arduino. Este código gestiona la interacción directa con los sensores y actuadores del vehículo.

### Librerías del Proyecto Arduino

El proyecto de Arduino utiliza las siguientes librerías. Para asegurar una compilación exitosa, es fundamental que estas librerías estén instaladas en tu IDE de Arduino.

* **`Wire.h`**:
    * **Tipo**: Librería estándar de Arduino.
    * **Descripción**: Permite la comunicación a través del bus I2C (Inter-Integrated Circuit), fundamental para interactuar con sensores como el TCS34725 y el MPU6050.
    * **Instalación**: Viene preinstalada con el IDE de Arduino. No se requiere acción adicional.
    * **Documentación**: [Referencia de la Librería Wire (Arduino)](https://www.arduino.cc/reference/en/libraries/wire/)

* **`Adafruit_TCS34725.h`**:
    * **Tipo**: Librería externa (Adafruit).
    * **Descripción**: Controla el sensor de color TCS34725, permitiendo la lectura de valores RGB para la detección de colores en el entorno.
    * **Instalación**: Se instala a través del **Gestor de Librerías del IDE de Arduino** buscando "Adafruit TCS34725".
    * **Documentación**: [Repositorio de GitHub de Adafruit_TCS34725](https://github.com/adafruit/Adafruit_TCS34725) | [Tutorial de Adafruit sobre TCS34725](https://learn.adafruit.com/adafruit-tcs34725-rgb-color-sensor?view=all)

* **`MPU6050.h`**:
    * **Tipo**: Librería externa (Electronic Cats).
    * **Descripción**: Facilita la interacción con el giroscopio y acelerómetro MPU-6050, proporcionando datos de movimiento y orientación para la navegación del vehículo.
    * **Instalación**: Se instala a través del **Gestor de Librerías del IDE de Arduino** buscando "MPU6050" (by Electronic Cats).
    * **Documentación**: [Repositorio de GitHub de MPU6050 (Electronic Cats)]([https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050](https://github.com/ElectronicCats/mpu6050))

* **`Servo.h`**:
    * **Tipo**: Librería estándar de Arduino.
    * **Descripción**: Permite controlar motores servo, utilizados para movimientos precisos en el vehículo (por ejemplo, dirección o manipulación).
    * **Instalación**: Viene preinstalada con el IDE de Arduino. No se requiere acción adicional.
    * **Documentación**: [Referencia de la Librería Servo (Arduino)](https://www.arduino.cc/reference/en/libraries/servo/)

* **`ColorConverterLib.h`**:
    * **Tipo**: Librería externa.
    * **Descripción**: Utilizada para la conversión entre diferentes espacios de color (ej. RGB a HSV/HSL), optimizando el procesamiento y la interpretación de los datos del sensor de color.
    * **Instalación**: Esta librería puede no estar disponible directamente en el Gestor de Librerías de Arduino. Se recomienda descargarla desde su repositorio de GitHub. Para instalarla, descargue el archivo ZIP y añádalo a su IDE de Arduino (`Sketch > Incluir Librería > Añadir Librería .ZIP`).
    * **Documentación/Repositorio**: Puedes encontrar el código fuente y más detalles en el repositorio oficial de GitHub: **[Arduino-ColorConverter](https://github.com/luisllamasbinaburo/Arduino-ColorConverter/tree/master)**

---

### Preparación del Proyecto Arduino

Para cargar el código en el microcontrolador de su vehículo:

1.  **Requisitos Previos**:
    * Tenga instalado el **IDE de Arduino** en su computadora.
    * Asegúrese de haber instalado todas las librerías externas mencionadas (Adafruit_TCS34725, MPU6050, ColorConverterLib) a través del Gestor de Librerías de Arduino o manualmente según sea necesario.

2.  **Abrir el Proyecto**:
    * Abra el archivo `.ino` principal de su proyecto de Arduino en el IDE de Arduino.

3.  **Seleccionar Placa y Puerto**:
    * En el IDE de Arduino, vaya a `Herramientas > Placa` y seleccione el modelo de Arduino que está utilizando (ej., Arduino Uno, ESP32 Dev Module, etc.).
    * Luego, vaya a `Herramientas > Puerto` y seleccione el puerto serie al que está conectado su Arduino.

4.  **Verificar y Subir**:
    * Haga clic en el botón "Verificar" (el checkmark) para compilar el código y comprobar errores.
    * Una vez verificado, haga clic en el botón "Subir" (la flecha) para cargar el código a su microcontrolador Arduino.

---

## APP WRO Prometheus Desarrollada en Android Studio

Este directorio contiene el código fuente y los artefactos necesarios para la aplicación **WRO Prometheus**, que sirve para procesar datos de imágenes para nuestro Vehículo Autónomo en la competencia. Aquí encontrarás toda la información relevante sobre las tecnologías utilizadas y cómo preparar el proyecto.

---

### Tecnologías y Dependencias Principales

Nuestra aplicación Android ha sido desarrollada utilizando **Android Studio** y se basa en las siguientes librerías y marcos de trabajo:

#### Librerías de AndroidX y Material Design

Componentes fundamentales de la plataforma Android para la interfaz de usuario y funcionalidades básicas, asegurando compatibilidad y una experiencia de usuario moderna.

* `androidx.core:core-ktx`: [Documentación oficial de AndroidX Core KTX](https://developer.android.com/kotlin/ktx)
* `androidx.appcompat:appcompat`: [Notas de lanzamiento de AndroidX Appcompat](https://developer.android.com/jetpack/androidx/releases/appcompat)
* `com.google.android.material:material`: [Guía de inicio de Material Design Components para Android](https://material.io/develop/android/docs/getting-started)
* `androidx.constraintlayout:constraintlayout`: [Notas de lanzamiento de AndroidX ConstraintLayout](https://developer.android.com/jetpack/androidx/releases/constraintlayout)

#### CameraX

Un *toolkit* de Android Jetpack que simplifica el uso de la cámara del dispositivo, proporcionando una API consistente y fácil de usar para diversas funcionalidades de cámara.

* `androidx.camera:camera-core`
* `androidx.camera:camera-camera2`
* `androidx.camera:camera-lifecycle`
* `androidx.camera:camera-view`
* Puedes encontrar la documentación y ejemplos en la **[Guía general de CameraX](https://developer.android.com/media/camera/camerax)**.

#### USB Serial for Android

Una librería esencial para la comunicación de nuestra aplicación con el microcontrolador del vehículo a través de una conexión serial USB.

* `com.github.mik3y:usb-serial-for-android`: Puedes consultar el repositorio oficial en **[GitHub - usb-serial-for-android](https://github.com/mik3y/usb-serial-for-android)**.

#### OpenCV (Open Source Computer Vision Library)

Una potente librería para el procesamiento de imágenes y visión por computador, utilizada para tareas de análisis visual y toma de decisiones dentro de la aplicación.

* **Integración Local**: Es importante destacar que **OpenCV está integrado como un módulo local dentro de este proyecto de Android Studio**. Esto significa que los archivos binarios y la configuración necesaria de OpenCV ya están incluidos en este repositorio.
* `implementation(project(":OpenCV"))`: Esta línea en el `build.gradle` indica que el módulo `:OpenCV` es una parte intrínseca de este proyecto.
* Para documentación general sobre la librería, puedes visitar el sitio oficial de **[OpenCV](https://opencv.org/)**.

#### Librerías de Testing

Componentes estándar para realizar pruebas unitarias y de instrumentación en la aplicación.

* `junit:junit`: [Sitio oficial de JUnit 4](https://junit.org/junit4/faq.html)
* `androidx.test.ext:junit`: [Notas de lanzamiento de AndroidX Test Ext JUnit](https://developer.android.com/jetpack/androidx/releases/test)
* `androidx.test.espresso:espresso-core`: [Notas de lanzamiento de AndroidX Test Espresso](https://developer.android.com/jetpack/androidx/releases/espresso)

---

### Preparación del Proyecto Android

Para compilar y ejecutar esta aplicación en su entorno, siga estos pasos:

1.  **Requisitos Previos**:
    * Asegúrese de tener **Android Studio** instalado y configurado, incluyendo el SDK de Android necesario y las herramientas de Gradle.

2.  **Abrir el Proyecto**:
    * Abra este directorio directamente en Android Studio.

3.  **Sincronización de Gradle**:
    * Android Studio detectará automáticamente el proyecto y sus archivos `build.gradle`. Se le pedirá que sincronice el proyecto. Este paso es crucial, ya que Gradle descargará automáticamente todas las dependencias externas (AndroidX, CameraX, USB Serial, etc.) desde los repositorios de Maven configurados.
    * **No se requiere instalación de OpenCV**: Dado que OpenCV se incluye como un módulo local, no necesita realizar ninguna instalación o configuración adicional de OpenCV en su sistema; Gradle lo gestionará directamente desde el código fuente del proyecto. Simplemente asegúrese de que la estructura de carpetas incluya el módulo `:OpenCV` y que esté referenciado en el `settings.gradle`.

4.  **Compilar y Ejecutar**:
    * Una vez que la sincronización de Gradle finalice sin errores, podrá compilar el proyecto y ejecutar la aplicación en su dispositivo Android.

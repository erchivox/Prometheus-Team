Control software
====

This directory must contain code for control software which is used by the vehicle to participate in the competition and which was developed by the participants.

All artifacts required to resolve dependencies and build the project must be included in this directory as well.

# APP WRO Prometheus Desarrollada en Android Studio

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

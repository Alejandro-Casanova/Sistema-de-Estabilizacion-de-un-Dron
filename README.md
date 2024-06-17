# Sistema de Estabilización de un Dron

[![license: AGPL](https://img.shields.io/badge/license-AGPL-lightgrey.svg)](https://raw.githubusercontent.com/Alejandro-Casanova/Sistema-de-Estabilizacion-de-un-Dron/main/LICENSE)

Este proyecto desarrolla el software necesario para mantener la horizontalidad y altitud de un dron en vuelo estático. Utiliza un giróscopo para medir rotación en los ejes X e Y, un acelerómetro para detectar aceleraciones en tres ejes, y un altímetro para medir la altura. El sistema controla los motores de las hélices para ajustar la posición y altura del dron. Además, detecta vibraciones bruscas para prevenir inestabilidad y envía datos a una base. 

El sistema de estabilización se implementa en una tarjeta STM32F407G-DISC1 utilizando FreeRTOS y C. Cada funcionalidad del sistema será gestionada por una tarea específica para cumplir con los requisitos de tiempo real, asegurando una operación eficiente y coordinada.

## Puesta en marcha
Debe utilizarse la IDE Keil uVision5 para importar y compilar cada proyecto.


## Enlaces de interés

- [La tarjeta empleada (STM32F407G-DISC1)](https://www.st.com/en/evaluation-tools/stm32f4discovery.html)
- [FreeRTOS Docs](https://www.freertos.org/Documentation/code/index.html)
- [Android Native Application (Java)](https://github.com/RefugeRestrooms/refugerestrooms-android)
- [iOS Native Application](https://github.com/RefugeRestrooms/refuge-ios)
- [Yo Application](https://github.com/raptortech-js/YoRestrooms)

## Implementaciones

### 1. Sistema de Estabilización Básico
El sistema de estabilización básico cuenta con cinco tareas y tres recursos compartidos protegidos por mutex. De las cinco tareas, cuatro son periódicas y la quinta es esporádica, siendo ésta activada por una interrupción hardware disparada por uno de los GPIOs del microcontrolador. La activación de la tarea esporádica se realiza mediante un semáforo, que será liberado desde la rutina de servicio de interrupción para desbloquear dicha tarea.

A continuación se muestra el diagrama de bloques simplificado del sistema, con todos los recursos compartidos, tareas y dispositivos. Las flechas negras indican los accesos a los recursos compartidos, indicándose con el sentido de éstas si se trata de una operación de lectura o escritura, y señalándose en verde el nombre de la variable particular a la que se accede. En el caso de los dispositivos, en verde se indican las operaciones que se realizan sobre estos a través de sus interfaces.

<img src="app/src/main/res/drawable-xxhdpi/Screenshots/screen1.png" alt="drawing" width="200"/>

## [Video Demo](https://youtu.be/-619bC9_QJU)

## Screenshots
<p float="left">
    <img src="app/src/main/res/drawable-xxhdpi/Screenshots/screen1.png" alt="drawing" width="200"/>
    <img src="app/src/main/res/drawable-xxhdpi/Screenshots/screen2.png" alt="drawing" width="200"/>
    <img src="app/src/main/res/drawable-xxhdpi/Screenshots/screen3.png" alt="drawing" width="197"/>
</p>

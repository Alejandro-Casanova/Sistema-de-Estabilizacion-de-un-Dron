# Sistema de Estabilización de un Dron

[![license: AGPL](https://img.shields.io/badge/license-AGPL-lightgrey.svg)](https://raw.githubusercontent.com/Alejandro-Casanova/Sistema-de-Estabilizacion-de-un-Dron/main/LICENSE)

Este proyecto desarrolla el software necesario para mantener la horizontalidad y altitud de un dron en vuelo estático. Utiliza un giróscopo para medir rotación en los ejes X e Y, un acelerómetro para detectar aceleraciones en tres ejes, y un altímetro para medir la altura. El sistema controla los motores de las hélices para ajustar la posición y altura del dron. Además, detecta vibraciones bruscas para prevenir inestabilidad y envía datos a una base. 

El sistema de estabilización se implementa en una tarjeta STM32F407G-DISC1 utilizando FreeRTOS y C. Cada funcionalidad del sistema será gestionada por una tarea específica para cumplir con los requisitos de tiempo real, asegurando una operación eficiente y coordinada.

## Video Demostrativo

<p align="center">
    <a href="https://youtu.be/37sEJJjmmXA" target="_blank" title="Go to video demo"><img alt="video demo" src="https://img.youtube.com/vi/37sEJJjmmXA/hqdefault.jpg">
    </a>
</p>

## Puesta en marcha
Debe utilizarse la IDE Keil uVision5 para importar y compilar cada proyecto.


## Enlaces de interés

- [La tarjeta empleada (STM32F407G-DISC1)](https://www.st.com/en/evaluation-tools/stm32f4discovery.html)
- [FreeRTOS Docs](https://www.freertos.org/Documentation/code/index.html)

## Diagrama de Bloques del Sistema
<img src="Documentación/Diagramas/2 Diagrama de Bloques del Sistema.png" alt="diagrama de bloques del sistema"/>

## Dispositivos empleados

1. Acelerómetro: realiza medidas de aceleración en los ángulos x, y, z. Su lectura se realiza a través de un bus SPI. Desde el micro se hace un cálculo adicional para derivar de dichas medidas la orientación del dispositivo (de esta manera se simula el giroscopio del que no dispone la tarjeta de desarrollo utilizada). Dado que tanto las tareas de inclinación como de vibraciones hacen uso de él, se trata de un recurso compartido, y su acceso ha sido protegido mediante un mutex. De esta manera, también se asegura que los cálculos intermedios realizados sobre sus lecturas se realizan de manera atómica.

2. Altímetro: simulado mediante un potenciómetro. Se realiza su lectura mediante el ADC del micro, obteniéndose una lectura entre 0 y 255, dado que el ADC se ha configurado con una resolución de 8 bits.

3. Motores: simulados mediante cuatro LEDs de colores de la tarjeta de desarrollo, que son encendidos y apagados mediante los GPIOs del microcontrolador.

4. LEDs de estado: uno indica si el sistema de estabilización se encuentra encendido o apagado, y otro indica si el estado de emergencia por vibraciones se encuentra activado.

## Implementaciones

### 1. Sistema de Estabilización Básico
El sistema de estabilización básico cuenta con cinco tareas y tres recursos compartidos protegidos por mutex. De las cinco tareas, cuatro son periódicas y la quinta es esporádica, siendo ésta activada por una interrupción hardware disparada por uno de los GPIOs del microcontrolador. La activación de la tarea esporádica se realiza mediante un semáforo, que será liberado desde la rutina de servicio de interrupción para desbloquear dicha tarea.

A continuación se muestra el diagrama de bloques simplificado del sistema, con todos los recursos compartidos, tareas y dispositivos. Las flechas negras indican los accesos a los recursos compartidos, indicándose con el sentido de éstas si se trata de una operación de lectura o escritura, y señalándose en verde el nombre de la variable particular a la que se accede. En el caso de los dispositivos, en verde se indican las operaciones que se realizan sobre estos a través de sus interfaces.

<img src="Documentación/Diagramas/3.1 Diagrama de Bloques Simplificado (original).png" alt="diagrama simplificado del sistema original"/>

### 2. Implementación con máquina de estados
En esta parte se sustituyó la tarea de control de los motores y dos de los recursos compartidos por una máquina de estados o "Statechart". De esta manera se realiza un mejor modelado del comportamiento del sistema, se reduce la carga de trabajo sobre el procesador (dado que se ha eliminado una tarea), se reduce el número de recursos compartidos, y se facilita la depuración del comportamiento del sistema. La propia máquina de estados será un recurso compartido, cuyo valor será su estado actual, y cuyo acceso será protegido mediante un mutex.

A continuación se muestran el diagrama de bloques simplificado, y el diagrama de estados, que definen esta implementación.

<img src="Documentación/Diagramas/3.2 Diagrama de Bloques Simplificado (statechart).png" alt="diagrama de bloques simplificado del sistema con statechart" />

<img src="Documentación/Diagramas/4 Diagrama de Estados.png" alt="diagrama de estados del sistema con statechart" />

### 3. Implementación Distribuida con Bus CAN
Partiendo del sistema original, se implementó una versión distribuida, separando en un nodo aparte la tarea de control de los motores. El resto de las tareas permanecen en el sistema original, y en lugar de escribirse los comandos de los motores en las variables compartidas, estos son enviados al segundo nodo a través de un bus CAN. Por lo tanto, los recursos compartidos Motor-Commands e Instability-Emergency han sido sustituidos por el módulo de comunicaciones del bus CAN. En las tareas de ambos nodos, las lecturas/escrituras de las variables compartidas han sido sustituidos por el envío y la recepción de los mensajes por el bus CAN.

<img src="Documentación/Diagramas/3.3 Diagrama de Bloques Simplificado (distribuido).png" alt="diagrama de bloques simplificado del sistema distribuido con can bus" />

## Montaje
### Centralizado (izquierda) / Distribuido (derecha)
<p align="center">
    <img src="Documentación/Montaje/1 Montaje Centralizado.png" alt="montaje centralizado" height=250/>
    <img src="Documentación/Montaje/2 Montaje Distribuido.jpg" alt="montaje distribuido" height=250/>
</p>
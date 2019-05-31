# Curso de RTOS - ESE - FIUNER
## Estimación de estabilidad basado en IMU y EDU-CIAA

Implementación de un **monitor de estabilidad para un móvil**. Para ello se utiliza:
-EDU-CIAA
-IMU9250
-Display TFT basado en ILI9341
-Lector de tarjetas SD

EDU-CIAA corre bajo FreeRTOS. Se generan lecturas periódicas del IMU, luego se las filtra, y posteriormente se realiza una evaluación de las últimas lecturas, para confeccionar un indicador. Este indicador es mostrado en el display TFT. 

Existe además una comunicación con UART para realizar cambios en la configuración. La misma permite cambiar umbrales de aceleración lineal (opc. 1),  umbrales de aceleración angular (opc. 2) parámetros de los filtros (opc. 3), y el periodo de muestreo (opc. 4). Estas opciones se muestran al presionar **TEC2**.

Al presionar **TEC1** se genera una interrupción y se graban los últimos datos generados en la tarjeta SD.


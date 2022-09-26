# Sensor_GM_LoRA
### Sensor Geomagnético LoRa

El código funciona y está probado con los siguientes componentes:

- Placa RAK-5005-O [Datasheet](https://docs.rakwireless.com/Product-Categories/WisBlock/RAK5005-O/Datasheet/).
- Módulo central RAK4631 [Datasheet](https://docs.rakwireless.com/Product-Categories/WisBlock/RAK4631/Datasheet/).
- Módulo IO RAK13002 [Datasheet](https://docs.rakwireless.com/Product-Categories/WisBlock/RAK13002/Overview/).
- Sensor Geomagnético RM3100 [Imagen](https://raw.githubusercontent.com/UrbOSDev/Sensor_GM_LoRA/main/Img/rm3100.png).

Se publica junto al código los archivos (Gerber) de un shield que se fabricó para unir los componentes Wisblock al sensor RM3100.

La idea de este sensor es la de enviar una señal de detección o no-detección de masa vehicular a un concentrador mediante la red LoRa (AU915 para el caso de Chile).
El sensor va empotrado en el pavimneto y es capaz de detectar las variaciones electromagnéticas que ocurren por encima.
Envía un sólo dato (1 o 0). Y el concentrador analiza esos datos para determinar el estado de una pista de circulación.

Este repositorio y su contenido está bajo licencia AGPL-3.0 [Licencia](https://github.com/UrbOSDev/Sensor_GM_LoRA/blob/main/LICENSE)

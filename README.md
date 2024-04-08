Estación Meteorológica Pape: Sistema de Adquisición y Transmisión de Datos Meteorológicos Basado en ESP32
---------------------------------------------------------------------------------------------------------

Descripción
-----------

Este proyecto implementa una estación meteorológica utilizando un microcontrolador ESP32. El sistema recopila datos de diversos sensores y los transmite a un servidor remoto para su almacenamiento y análisis.

Hardware
--------

Componentes:

-   ESP32 DevKitC
-   Sensor de temperatura MCP9808
-   Sensor de humedad y temperatura DHT11
-   Sensor de presión barométrica (compatible con SparkFun MicroPressure)
-   Anemómetro
-   Veleta
-   Resistores
-   Breadboard
-   Cables de puente

Diagrama de conexiones:

Diagrama de Conexiones: [se quitó una URL no válida]

Software
--------

-   IDE de Arduino
-   Librería Adafruit MCP9808
-   Librería SparkFun MicroPressure
-   Librería DHTesp

Pinout
------

| Sensor | Pin | Función |
| ------ | --- | ------ |
| MCP9808 | D2 | SDA (I2C) |
| MCP9808 | D1 | SCL (I2C) |
| DHT11 | 15 | Data |
| Barometer | No especificado | Se requiere configuración específica según el sensor (ver biblioteca SparkFun_MicroPressure) |
| Anemometer | 37 | ADC |
| Vane | 36 | ADC |

drive_spreadsheetExportar a Hojas de cálculo

Librerías
---------

-   Adafruit MCP9808: [se quitó una URL no válida]
-   SparkFun MicroPressure: [se quitó una URL no válida]
-   DHTesp: [se quitó una URL no válida]

Recursos Adicionales
--------------------

-   Datasheet del MCP9808: <https://www.microchip.com/en-us/product/MCP9808>
-   Datasheet del DHT11: [se quitó una URL no válida]
-   Guía de SparkFun MicroPressure: [se quitó una URL no válida]
-   Guía de DHTesp: [se quitó una URL no válida]

Funcionamiento
--------------

1.  Los sensores recopilan datos de temperatura, humedad, presión, velocidad del viento y dirección del viento.
2.  Los datos se procesan y filtran para mejorar la precisión.
3.  Los datos se transmiten a un servidor remoto mediante una solicitud HTTP POST.
4.  El servidor puede almacenar, analizar y visualizar los datos.

Formato de Datos
----------------

Los datos se transmiten al servidor como una cadena separada por comas (CSV) que contiene los siguientes valores:

-   Temperatura (MCP9808)
-   Presión (barómetro)
-   Temperatura (DHT11)
-   Humedad (DHT11)
-   Velocidad del viento (anemómetro)
-   Dirección del viento (grados) (veleta)
-   Velocidad del viento filtrada (anemómetro)
-   Dirección del viento filtrada (grados) (veleta)

Integración con el Servidor
---------------------------

Se requiere un script del lado del servidor (p. ej., Flask) para recibir la solicitud HTTP POST que contiene la cadena de datos meteorológicos. El script puede:

-   Almacenar los datos en una base de datos.
-   Mostrar los datos en un panel.
-   Integrar los datos en un flujo de trabajo de análisis de datos.

Notas
-----

-   El pinout del barómetro no se ha especificado ya que depende del modelo utilizado. Consulte la documentación del sensor o la biblioteca SparkFun_MicroPressure para obtener la configuración específica.
-   Asegúrese de verificar las conexiones de pinout para su placa ESP32 específica, ya que pueden variar.
-   Se recomienda utilizar técnicas de manejo de errores y depuración para un sistema robusto.

Conclusión
----------

La estación meteorológica Pape es un sistema versátil y fácil de usar para la adquisición y transmisión de datos meteorológicos. El sistema se puede personalizar para satisfacer diferentes necesidades y aplicaciones.

Créditos
--------

-   Adafruit Industries por la biblioteca Adafruit_MCP9808
-   SparkFun Electronics por la biblioteca SparkFun_MicroPressure
-   Espressif Systems por el microcontrolador ESP32

Licencia
--------

Este proyecto está licenciado bajo la Licencia MIT.

Información de contacto
-----------------------

-   Correo electrónico: [su dirección de correo electrónico]

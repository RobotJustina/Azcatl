Descripción de los archivos del fichero "Smart Things Develop"

Ultima revisión: 10 de Diciembre dle 2018
Encargado: Daniel Garcés Marín

 Los archivos que se encuentran en este fichero son aquellos desarrollados previamente para hacer funciónar la modalidad de Smart Things; las caracterśiticas del proyecto consistían en comandar un robot por medio de una aplicación móvil haciendo uso de una cruceta de direcciónes y enviando la instrucción por medio de una varaiable de tipo string. La cual era enviada por la aplicación móvil, pasando por los servidores y el modem de proposito específico (lo cual generaba un pequeño retrazo de 2-3 seg) y era recibido por el módulo instalado en el shiel del arduino UNO, trasnmitiendo este la intrucción a la raspberry y recibida e interpretada por un nodo en python.

 >>>Archivos disponibles:

 	-[ smart_things ] :: Este fichero es el el paquete que debe intalarse dentro de la arquitectura del robot, el cual contiene el nodo en python que debe ejecutarse para recibir y transmitir correctamente la orden enviada.

 	-[ Shield Libray] :: Este fichero contiene los archivos que deben instalarse en el IDE del Arduino para poder hacer uso del módulo Smart Things.

 	-[ Arduino_ThingShield ] :: Este fichero contiene el archivo .ino que debe programamrse en el arduino UNO para poder hacer un correcto uso del módulo Smart Things.


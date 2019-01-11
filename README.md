# Azcatl
LABORATORIO DE BIOROBÓTICA, Facultad de Ingeniería UNAM.

Responsable: Daniel Garcés Marín

Integrantes:
            
            Victor Hugo Sanchez Correa --- Hugo (2)
            
            Omar 
            
Breviario cultural: Azcatl = Hormiga en Nahuatl

El proposito de este repositorio es almacenar el trabajo que se ha desarrollado en el proyecto Azcatl, el cual tiene que objetivo desarrollar un robot móvil autónomo el cual cuente con características similares a los desarrolados en el proyecto justina, en términos de navegación autónoma y planeación de acciones, además de que cuente con una compatibilidad para trabajar con dispositivos Smart Things. 

Actualmente el proyecto cuenta con un robot móvil de seis ruedas cuyas características se encuentran explicadas en la documentación incluida en este repositorio. Pero en pocas palabras consta de un robot de tipo diferencial con 6 ruedas, una torre de fotoresistores, el módulo Smart Things, arduinos y una raspberry sobre la cual corre la arquitectura del proyecto.

=> DESCRIPCIÓN DE LAS RAMAS DE DESARROLLO:

- [SAFE_MODE]:: En esta rama solo se mantendrá el el workspace con la arquitectura y los nodos mas sencillos para la prueba del hardware con el que cuenta el robot; el principal objetivo es el revisar cada uno de los componentes disponibles en caso de probar con un nuevo roboto o el que se encuentre en uso presente algunos problemas de desempeño o funcioabilidad.

- [MASTER]:: En esta rama se guardarán la arquitectura y nodos que se han desarrollado y que han demostrado tener la eficiencia adecuada para cumplir con el comportamiento que se desea implementar el robot. En otras palabras, es el código que consideramos el más adecuado para implementar en el robot y que consiga los objetivos deseado.

- [DEVELOP]:: En esta rama se guardarán los archivos que se están desarrollando y probando, los cuales pueden contener algunos errores ya sea de programación y lógica, los cuales serán revisados e implmentados para poder ser aprobados y guardados en la rama MASTER.

=> CONTENIDO DEL REPOSITORIO

- Workspace [catkin_ws] :: En esta carpeta se encuentra almacenada todo el software desarrollado hasta la fecha, el cual debe ser implementado en la tarjeta Raspberry Pi que se encuentra integrada al robot.

- Smart things [SmartThings_Develop] :: En esta carpeta se encuentra todo el software necesario para el uso del módulo Smart Things para la tarjeta arduino UNO. Así como el código original del nodo para ROS con el que se comenzó a implementar este módulo.

- Archivos Arduino [Arduino_files] :: En esta carpeta se encuentran los archivo .iso que debes ser grabados en las tarjetas Arduino UNO paa el uso del módulo SMART THINGS y Arduino Mega para la lectura de los sensores y el control de los motores. Esta carpeta se encuentra dentro del workspace [catkin_ws] para mayor comodidad

- Documentacion [Manual_Usuario_Azcatl] :: Esta documentación describe el desarrolloo del robot, así como una rápida introducción a cada uno de los componentes disponibles y una serie de instrucciones para la instalación del software necesario. Este documento tamibén se encuentra dentro del workspace [ccatkin_ws] para un accceso más rápido y cómodo.

=> INFORMACIÓN EXTRA
- Cuenta de correo para el uso de la tablet, user: "lab.biorobotica@gmail.com"    pwrd: "JustinaTimeDebug" 
- Contraseña tablet: "pumas"
- Cuenta de correo para el uso de la app Smart Things, user:"Robotscasa@gmail.com"  pwrd: "p0u7mjaqsw"
- Usuario para la computadora Dell asignada al proyecto, user: Biorobotica, pwrd: pumaspumas


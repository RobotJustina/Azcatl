*****************************************************
*	Laboratorio de Birobótica 2019
*	Proyecto << ÁZCATL>>
*
*	Ultima revisión: 24 de Enero del 2019
*****************************************************

-->> 22 de Enero del 2019

	-Los cambios que se han realizado en del proyecto usando como punto de partida lo relacionado con el proyecto 
	denominado como "Xochitonatl" son:

		<> Modificación del circuito eléctrio del robot, se agrego una serie de clemas al robot para un mejor manejo 
		de ladistribución.
		<> Modificación de la alimentación de varios componentes, ahora con las clemas extras se fijó una específica 
		para la alimentación 5V
		<> Se cambió la alimentación de los enconders a 5 V (revisar datasheet)
		<> Se añadierón sensores infrarrojos de distancia para la detección de obstaculos, y para así ser 
		implementadas las SM de evasión.
		<> Se realizó exitosamente el control PID de los motores
		<> Modificación del nombre de varios archivos en pos de cubrir el nuevo nombre del proyecto.

	-[*] Pendientes:

			+ Documentación del proceso de modificación del robot
			+ Modificación y adecuación de la documentación del ahora nuevo proyecto
			+ Documentación del proceso del control PID
			+ Documentación del proceso de modificación de la App Móvil
			+ Modifiación del nodo de sensores (Rasp) para aceptar los nuevos datos de los sensores infrarrojos
			+ Modificación del pkg al nombre actual del proyecto
			+ Instalación de git en raspbery(?)

	-[#]{D} Por el momento salvo algunos problemas con el hardware del robot no se ha producido gran cambio del concepto
	en general, además de que el proceso se lleva a un ritmo adecuado, se debe documentar todo de inmediato para evitar 
	cualquier otro problema en el futuro con respecto a los componentes o implementación del software. Posiblemente 
	considere una completa reorganización para el manual del robot y tener las consideraciones actuales de las 
	características del robot, mas allá de complementar la documentación acutalmente existente. 
___________________________________________________________________________________________________________________________________________________________________

-->> 24 de Enero del 2019

	-Se solucionaron los problemas de la aplicación Móvil.
		[#]{D} Básicamente lo que realice fue actualizar el arduino uno con el mismo archivo que se tenía 
		disponible, la cuestión recaía en que el comando si era recibido en el arduino UNO pero no era procesado 
por el nodo de SmartThings.py debido a que no correspondía con el formato propuesto para cortar la cadena.


	-Se modifico el archivo para las comunicación vía ssh con la raspberry.

	-[!] No hay archivos en la carpeta Smart_Things Library

	-Se realizó un respaldo del código de la app
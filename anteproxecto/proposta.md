# Títulos:

- Mellora do comportamento a baixo nivel da patada dun robot cuadrúpedo
- Mejora del comportamiento a bajo nivel del pateo de un robot cuadrúpedo
- Improved low-level kicking behavior of a quadriped robot

# Directores 

- Carlos Vázquez Regueiro

# Mención

- Enxeñaría de computadores

# Tipo de proxecto

- Desenvolvemento en investigación

# Breve descripción

El interés por este proyecto surge del contexto de competiciones como la RoboCup, donde se plantean desafíos avanzados de percepción, decisión y acción en entornos dinámicos. Se trata de un comportamiento complejo que requiere la integración de módulos de percepción con comportamientos ya establecidos, así como el desarrollo de nuevos movimientos a bajo nivel para lograr una acción específica como chutar un balón.

Este proyecto trará de mejorar el comportamiento de un robot cuadrúpedo Go2 definido en un TFG anterior. El TFG mencionado, "Un robot cuadrúpedo que juega al fútbol: detectar y patear una pelota" realizado por Samuel Pérez Fente, realiza un gran trabajo según lo que propone. El robot detecta un objeto de pateo (p.e. una pelota o botella) y otro de destino (p.e. una pelota o botella), se acerca al de pateo y lo golpea para que colisione con el de destino. Aunque el comportamiento del robot acaba por ser el planificado, la dificultad del proceso produce escenarios donde el robot realiza acciones repetitivas sin acercarse necesariamente al objetivo esperado, de esta forma ciertos procesos aceptan mejoras. 

Contemplamos dos lineas de mejora respecto al estado actual:

Primero, se buscará mejorar el proceso de pateo. Actualmente este proceso emplea una acción de alto nivel del robot proporcionada por el software de control de unitree, un saludo. El saludo produce que el robot levante la pata frontal derecha y la mueva. Este levantamiento es el que se empleaba en el TFG anteriror para golpear la pelota. Esta metodología produce una limitación respecto al control del pateo de la pelota, pues es imposible para nosotros ajustar o controlar de forma consistente el proceso de pateo. Estudiaremos entonces la posibilidad de intercambiar este comportamento por uno de bajo nivel que permita un mayor control respecto a los movimientos del pateo. De esta forma cada pateo tendrá en cuenta la posición relativa de la pelota respecto a la pata, pudiendo calcular y modificar la trayectoria y seleccionar el punto de impacto sobre la pelota. Para poder llevar a cabo esto necesitamos ser capaces de: modificar el plano en el que se mueve la pata y modificar el ángulo de flexión de la pata. Con estas modificaciones podrémos seleccionar el punto de impacto sobre la pelota

En segundo lugar se estudiará la posibilidad de añadir funciones en bajo nivel que permitan realizar desplazamientos precisos de la orden de centímetros. Estas funciones suplirán el problema de desplazamiento actual, donde el robot pasa mucho tiempo corrigiendo una y otra vez su posición respecto a la pelota debido a que estos ajustes no son precisos ni controlados. Los comportamientos a añadir se basarán en implementaciones a bajo nivel de desplazamientos controlando los grados de cada articulación. Esto permitirá el movimiento del robot de forma controlada y se añadirán a los casos adecuados para mejorar el proceso de alineación.

Debido a la complejidad y peligrosidad (para la integridad del propio robot) de los movimientos y procesos de desplazamiento a realizar, se llevará a cabo el desarrollo y prueba de este comportamiento en un entorno simulado. Para ello se empleará ROS2 en un contenedor Docker sobre Ubuntu junto un software de simulación (p.e. Mujoco Sim). 

Se utilizará ROS2 (Robot Operating System 2) como plataforma para la gestión de nodos, sensores, actuadores y lógica de control. El desarrollo se centrará en emplear los datos de las cámaras RGB del robot, la cámara LIDAR 3D y los modelos de inteligencia artificial ya integrados. Estos datos se usarán en funciones de bajo nivel que permitan un realizar un control adecuado y preciso de las diferentes partes del robot. Podremos entonces golpear la pelota eligiendo el punto de impacto sobre ella.

Inicialmente emplearemos un software de simulación donde probar los diferentes comportamientos anteriormente mencionados, evitaremos y corregiremos así fallos que afecten a la integridad del robot. El software de simulación nos permitirá realizar pruebas en un entorno con físicas realistas para validar el comportamiento antes de realizar el despliegue en el robot real. En este entorno de simulación podrémos modelar el robot, los objetos (de pateo y destino) y el terreno. Si tras la simulación y corrección de errores el comportamiento teórico es satisfactorio, se realizarán pruebas sobre el robot físico para comprobar el funcionamiento en condiciones reales.

La implementación de comportamientos a bajo nivel se realizará empleando la interfaz de unitree_ros2, implementando el código en C++ y usando mensajes del unitree sdk2 (p.e. LowCmd). El comportamiento se declarará mediante los valores de las articulaciones del robot, indicando así posturas concretas que este tomará con tal de completar el comportamiento. Los conjuntos de datos de las articulaciones serán susceptibles a la información de los diferentes sensores de las cámaras, permitiendo así mejorar la precisión y adaptabilidad de los movimientos.

Este proyecto nos presenta un reto técnico centrado en el control a bajo nivel de los componentes del robot, el mantener la integridad y equilibrio del mismo sin el apoyo de las funciones de alto nivel y el cálculo de valores a aplicar a los componentes del robot con tal de realizar un golpeo preciso que siga una trayectoria concreta. El planteamiento teórico y cálculos a realizar junto con las aplicaciones posibles de este comportamiento nos permite comprobar el interés académico y aplicado del proyecto.

# Obxectivos concretos

1. Creación de un entorno de simulación en Docker con Ubuntu, ROS2 y Mujoco.
2. Estudiar el diseño de funciones a bajo nivel que permitan realizar el pateo de una pelota de forma precisa y el cálculo de la trayectoria de la misma evitando la caída del robot.
   1. Usar el entorno simulado para comprobar el correcto funcionamiento del pateo y verificar los cálculos computados.
   2. Validar el comportamiento desarrollado en el robot real, comprobando que los pateos son efectivos, el robot mantiene el equilibrio y la pelota sigue la trayectoria planeada según el punto de impacto.
3. Desarrollo de funciones a bajo nivel que permitan realizar desplazamientos del robot precisos y en distancias cortas sin comprometer su integridad, realizando pruebas en el simulador y posteriormente en el robot real. 
4. Comprobar el correcto comportamiento completo en el robot real, pudiendo precisar el punto de impacto del pateo según convenga y empleando los movimientos precisos empleados para una correcta alineación.

# Método de traballo

Se empleará una metodología ágil, con una planificación iterativa y flexible del trabajo. Esto permitirá estructurar el desarrollo en ciclos cortos facilitando la identificación temprana de problemas y la evolución gradual del sistema mediante la implementación y ajuste de los distintos módulos.

# Fases principais

1. Preparación entorno desarrollo:
   - Instalación de una imagen en Docker con Ubuntu, ROS y Mujoco.
   - Configuración del entorno con paquetes ROS2 para el Go2 (simulación y control).
   - Conexión con el robot real.
2.  Estudio preliminar del pateo:
   - Verificación de estabilidad del robot con una pata levantada y libertad de movimiento de la misma.
   - Implementación sencilla de un pateo que no comprometa la integridad del robot.
   - Verificación del funcionamiento en Mujoco y ajustes necesarios.  
   - Verificación en robot real y ajustes necesarios.
3. Mejora del pateo: [uso IA? Cálculo ángulos de los flexores y actuadores]
   - Uso posiciones relativas de las pelotas y patas.
   - Cálculo trayectorias, potencia de pateo y punto de golpeo sobre la pelota.
   - Integración empleando datos de los cálculos realizados en la función de pateo a bajo nivel.
   - Verificación del funcionamiento en Mujoco y ajustes necesarios.  
   - Verificación en robot real y ajustes necesarios.
4. Desarrollo a bajo nivel de movimientos precisos:
   - Comprobación de control a bajo nivel de las 4 patas de forma simultánea.
   - Establecer esquema de movimiento a seguir.
   - Verificación del funcionamiento en Mujoco y ajustes necesarios.  
   - Verificación en robot real y ajustes necesarios.
5. Integración movimientos precisos en proceso de alineación:
   - Integrar nueva función precisa de desplazamiento en proceso de alineación.
   - Uso de sensores y IA integrada para determinar distancia a desplazarse.
   - Verificación del funcionamiento en Mujoco y ajustes necesarios.  
   - Verificación en robot real y ajustes necesarios.
6. Validación del comportamiento completo en el robot real:
   - Montado del sistema en el robot real.
   - Pruebas y ajustes finales en el entorno real.
7. Memoria y documentación.

# Material e medios necesarios

- Hardware:
  - Robot cuadrúpedo Go2 de Unitree (con cámaras RGB y LIDAR 3D)
  - PC para desarrollo del software y control del robot

- Software:
- ROS2
- Mujoco
- Docker
- Herramientas de visión por computador
- SDK del Go2


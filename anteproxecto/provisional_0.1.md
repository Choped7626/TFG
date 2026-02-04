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

[necesario?-modificar/eliminar]

El interés por este proyecto surge del contexto de competiciones como la RoboCup, donde se plantean desafíos avanzados de percepción, decisión y acción en entornos dinámicos. Se trata de un comportamiento complejo que requiere la integración de módulos de percepción con comportamientos ya establecidos, así como el desarrollo de nuevos movimientos a bajo nivel para lograr una acción específica como chutar un balón.

[necesario?-modificar/eliminar]

Este proyecto tratará de mejorar el comportamiento de un robot cuadrúpedo Go2. La finalidad de dicho robot es el de detectar una pelota determinada (objeto de pateo) y otro objeto de destino (una pelota o botella), acercarse al objeto de pateo y chutarlo con tal de que colisione con el objeto de destino. 

El objetivo del proyecto consistirá en mejorar los comportamientos de bajo nivel empleados en este proceso. Se implementará la acción final de chutar, teniendo en cuenta la posición relativa entre la pelota respecto a la pata. En primer lugar se analizará la secuencia de órdenes a bajo nivel que mueven la pata que chuta. A continuación, se estudiará cómo alterar dicha secuencia para conseguir dos objetivos: primero, modificar el plano en el que se mueve la pata, segundo, modificar el ángulo de flexión de la pata para poder seleccionar (hasta cierto punto) el punto de impacto de la pata sobre la pelota. 

El propósito de estas modificaciones es poder seleccionar la dirección del chute, es decir, en qué dirección saldrá la pelota que se empuja. Debido a la complejidad y peligrosidad (para la integridad del propio robot) de los movimientos a realizar se llevará a cabo la implementación y prueba de este comportamiento en un entorno simulado. Para ello se empleará el software Gazebo Sim y ROS2 en un contenedor Docker sobre Ubuntu. La herramienta Gazebo Sim nos permitirá realizar pruebas en un entorno de simulación física realista para validar el comportamiento antes de realizar el despliegue en el robot real. En este entorno de simulación podrémos modelar el robot, los objetos (de pateo y destino) y el terreno. 

[Mencionar sustitución ROS1 por EOL?]

Se utilizará ROS2 (Robot Operating System 2) como plataforma para la gestión de nodos, sensores, actuadores y lógica de control. El desarrollo se centrará en la implementación de funciones de bajo nivel que, a partir de los datos de las cámaras RGB del robot y los modelos de inteligencia artificial ya integrados, permitirán realizar un golpeo preciso de la pelota pudiendo elegir el punto de impacto al golpearla. El robot chutará entonces con una de sus patas, calculando la trayectoria adecuada y el punto de impacto sobre la pelota, para que esta a su vez golpeé al objeto de destino. Las funciones implementadas a bajo nivel entonces calcularán y ejecutarán la acción de golpeo actuando directamente sobre los motores y flexores del robot, permitiendo así un control preciso del movimiento de la pata robótica.

Además, se estudiará la posibilidad de mejorar el movimiento del robot mediante una aproximación a bajo nivel. Esta implementación nos permitiría entonces realizar movimientos precisos del robot, es decir, desplazarlo tan solo unos centímetros o milímetros en la dirección deseada. Esta funcionalidad, de ser integrada, mejoraría el comportamiento del robot pues actualmente el proceso de alineación para realizar el chutado de la pelota no es preciso y requiere de varios intentos y movimientos extra hasta alcanzar una posición adecuada.

Estos comportamientos, chute y alineación precisa, se desarrollarán e implementarán inicialmente en un entorno simulado con Gazebo para evitar y corregir fallos que afecten a la integridad del robot. Si tras la simulación y corrección de errores el comportamiento teórico es satisfactorio, se realizarán pruebas sobre el robot físico para comprobar el funcionamiento en condiciones reales. 

Este proyecto nos presenta un reto técnico centrado en el control a bajo nivel de los componentes del robot, el mantener la integridad y equilibrio del mismo sin el apoyo de las funciones de alto nivel y el cálculo de valores a aplicar a los componentes del robot con tal de realizar un golpeo preciso que siga una trayectoria concreta. El planteamiento teórico y cálculos a realizar junto con las aplicaciones posibles de este comportamiento nos permite comprobar el interés académico y aplicado del proyecto.

# Obxectivos concretos

[revisar/modificar]

 	1. Creación de un entorno de simulación en Docker con Ubuntu, ROS2 y Gazebo.
 	2. Implementar funciones a bajo nivel que permitan realizar el pateo de una pelota de forma precisa y el cálculo de la trayectoria de la misma evitando la caída del robot.
 	3. Usar el entorno simulado para comprobar el correcto funcionamiento del pateo y verificar los cálculos computados.
 	4. Validar el comportamiento desarrollado en el robot real, comprobando que los pateos son efectivos, el robot mantiene el equilibrio y la pelota sigue la trayectoria planeada según el punto de impacto.
 	5. Implementar funciones a bajo nivel que permitan realizar desplazamientos del robot precisos y en distancias cortas sin comprometer su integridad, realizando pruebas en el simulador y posteriormente en el robot real. 
 	6. Comprobar el correcto comportamiento completo en el robot real, pudiendo precisar el punto de impacto del pateo según convenga y empleando los movimientos precisos empleados para una correcta alineación.

# Método de traballo

[modificar?]

Se empleará una metodología ágil, con una planificación iterativa y flexible del trabajo. Esto permitirá estructurar el desarrollo en ciclos cortos facilitando la identificación temprana de problemas y la evolución gradual del sistema mediante la implementación y ajuste de los distintos módulos.

[modificar?]

# Fases principais

[revisar/modificar]

1. Preparación entorno desarrollo:
   - Instalación de una imagen en Docker con Ubuntu, ROS y Gazebo.
   - Configuración del entorno con paquetes ROS2 para el Go2 (simulación y control).
   - Conexión con el robot real.
2.  Desarrollo implementación a bajo nivel del pateo:
   - Verificación de estabilidad del robot con una pata levantada y libertad de movimiento de la misma.
   - Implementación sencilla de pateo contra la pelota con valores por defecto.
   - Verificación del funcionamiento en Gazebo y ajustes necesarios.  
   - Verificación en robot real y ajustes necesarios.
3. Mejora implementación pateo:
   - Uso posiciones relativas de las pelotas y patas.
   - Cálculo trayectorias, potencia de pateo y punto de golpeo sobre la pelota.
   - Integración empleando datos de los cálculos realizados en la función de pateo a bajo nivel.
   - Verificación del funcionamiento en Gazebo y ajustes necesarios.  
   - Verificación en robot real y ajustes necesarios.
4. Desarrollo implementación a bajo nivel de movimientos precisos:
   - Comprobación de control a bajo nivel de las 4 patas de forma simultánea.
   - Establecer esquema de movimiento a seguir.
   - Verificación del funcionamiento en Gazebo y ajustes necesarios.  
   - Verificación en robot real y ajustes necesarios.
5. Integración movimientos precisos en proceso de alineación:
   - Integrar nueva función precisa de desplazamiento en proceso de alineación.
   - Uso de sensores y IA integrada para determinar distancia a desplazarse.
   - Verificación del funcionamiento en Gazebo y ajustes necesarios.  
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
- Gazebo
- Docker
- Herramientas de visión por computador
- SDK del Go2


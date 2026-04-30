# Títulos:
- Mellora do comportamento a baixo nivel da patada dun robot cuadrúpedo
- Mejora del comportamiento a bajo nivel del pateo de un robot cuadrúpedo
- Improved low-level kicking behavior of a quadriped robot

# Directores 
- Carlos Vázquez Regueiro
- Martín Naya Varela

# Mención
- Enxeñaría de computadores

# Tipo de proxecto
- Desenvolvemento en investigación

# Breve descripción



El interés por este proyecto surge del contexto de competiciones robóticas como la RoboCup, donde se plantean desafíos avanzados de percepción, decisión y acción en entornos dinámicos. Este proyecto trata de mejorar el comportamiento de un robot cuadrúpedo Go2 definido en un TFG anterior, "Un robot cuadrúpedo que juega al fútbol: detectar y patear una pelota" de Samuel Pérez Fente, en el que el robot detecta un objeto de pateo y otro de destino, se aproxima al primero y lo golpea para que colisione con el segundo.
El acercamiento se divide en dos fases: una basada en datos de la cámara RGBD y una fase final apoyada en datos del LIDAR. Aunque el comportamiento resulta el previsto, presenta un problema importante de precisión en la aproximación final al objeto de pateo, ya que los comandos de navegación de alto nivel no están diseñados para correcciones del orden de centímetros. Esto provoca un elevado número de rectificaciones automáticas hasta alcanzar una posición aceptable.
Vistas estas limitaciones, el proyecto contempla dos líneas de mejora:
La primera consiste en mejorar el proceso de pateo. Actualmente se emplea una acción predefinida de alto nivel del software de Unitree (un saludo), en la que el robot levanta la pata frontal derecha para golpear la pelota. Esta acción es siempre la misma, independientemente de la posición relativa respecto al objeto o del punto destino. El objetivo es implementar un nuevo comportamiento de pateo de bajo nivel que permita mayor control sobre la pierna de golpeo y la selección del punto de impacto sobre la pelota, para así determinar mejor su trayectoria final. Por bajo nivel se entiende el control directo de la posición o torque de cada motor del robot, sin recurrir a comportamientos predefinidos. Esto incluye también el control de los motores que no participan en el pateo pero que son necesarios para mantener la estabilidad. La propuesta es calcular el ángulo del hombro para modificar el plano de movimiento de la pata durante el pateo y, de ser necesario, ajustar el ángulo de flexión.
La segunda mejora consiste en añadir comportamientos de bajo nivel para realizar desplazamientos precisos del orden del centímetro y alineamientos del orden del grado durante la fase final de acercamiento al objeto (taconeo). La propuesta es mover cada articulación en secuencia, manteniendo las otras tres patas en el suelo para preservar el equilibrio. Coordinando el movimiento de cada pata se podrá hacer avanzar, retroceder, desplazar lateralmente o girar al robot con precisión y sin rectificaciones.
El proyecto usará ROS2 como plataforma de gestión de nodos, sensores y lógica de control, integrando datos de las cámaras RGB, la cámara LIDAR 3D y modelos de inteligencia artificial ya disponibles. La implementación de los comportamientos de bajo nivel se realizará mediante la interfaz unitree_ros2, en C++ o Python, usando mensajes del Unitree SDK2 (como LowCmd), definiendo posturas concretas a través de los valores de las articulaciones del robot.
Dado el riesgo para la integridad del robot, el desarrollo y las pruebas iniciales se realizarán en un entorno simulado: ROS2 en un contenedor Docker sobre Ubuntu junto con un software de simulación con físicas realistas (como MuJoCo), donde se modelarán el robot, los objetos y el terreno. Una vez validado el comportamiento en simulación, se realizarán pruebas sobre el robot físico.
El principal reto técnico del proyecto es procesar la información sensorial del robot, controlar a bajo nivel sus motores y mantener el equilibrio sin apoyo de funciones de alto nivel, con el fin de lograr un golpeo preciso que siga una trayectoria determinada.
# Obxectivos concretos
1. Creación de un entorno de simulación en Docker con Ubuntu, ROS2 y Mujoco.
2. Desarrollar un comportamiento a bajo nivel que permita realizar el pateo de una pelota hacia un objetivo.
   1. Usar el entorno simulado para comprobar el correcto funcionamiento del pateo y verificar los cálculos computados.
   2. Validar el comportamiento desarrollado en el robot real, comprobando que los pateos son efectivos, el robot mantiene el equilibrio y la pelota sigue la trayectoria planeada.
3. Desarrollar comportamientos a bajo nivel que permitan realizar desplazamientos y alineamientos del robot precisos en distancias cortas sin comprometer su integridad. Realizar primero las pruebas en el simulador y posteriormente validarlas en el robot real. 
4. Comprobar el correcto funcionamiento de la solución propuesta  tanto en simulación como en el robot real.

# Método de traballo
Se empleará una metodología ágil, con una planificación iterativa y flexible del trabajo. Esto permitirá estructurar el desarrollo en ciclos cortos facilitando la identificación temprana de problemas y la evolución gradual del sistema mediante la implementación y ajuste de los distintos módulos.

# Fases principais
1. Preparación entorno desarrollo:
   - Instalación de una imagen en Docker con Ubuntu, ROS y Mujoco.
   - Configuración del entorno con paquetes ROS2 para el Go2 (simulación y control).
   - Conexión con el robot real.
2. Estudio preliminar del comportamiento de pateo:
   - Búsqueda de trabajo relacionado.
   - Verificación de estabilidad del robot con una pata levantada y libertad de movimiento de la misma.
   - Implementación sencilla de un pateo que no comprometa la integridad del robot.
   - Verificación del funcionamiento en Mujoco y ajustes necesarios. 
   - Verificación en robot real y ajustes necesarios.
3. Mejora del comportamiento de pateo:
   - Uso posiciones relativas del objeto de pateo y del destino.
   - Cálculo de la trayectoria según el punto de golpeo sobre la pelota.
   - Verificación del funcionamiento en Mujoco y ajustes necesarios. 
   - Verificación en robot real y ajustes necesarios.
4. Desarrollo de los pequeños desplazamientos:  avanzar y retroceder, derecha e izquierda
   - Cambiar la posición de una pata respecto al robot.
   - Secuenciar los movimientos de las cuatro patas para conseguir un desplazamiento efectivo.
   - Verificación del funcionamiento en Mujoco y ajustes necesarios. 
   - Verificación en robot real y ajustes necesarios.
5. Desarrollo de pequeños ajustes de alineamiento:
   - Secuenciar y modificar los comportamientos de las cuatro patas para conseguir cambiar la orientación del robot.
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

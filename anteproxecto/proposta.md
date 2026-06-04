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

El interés por este proyecto surge del contexto de competiciones robóticas como la RoboCup, donde se plantean desafíos avanzados de percepción, decisión y acción en entornos dinámicos. Este proyecto trata de mejorar el comportamiento de un robot cuadrúpedo Go2 diseñado en un TFG anterior, "Un robot cuadrúpedo que juega al fútbol: detectar y patear una pelota" de Samuel Pérez Fente. En dicho proyecto el robot detecta un objeto de pateo y otro de destino, se acerca al primero y lo golpea para que colisione con el segundo.

El acercamiento se divide en dos fases: una basada en datos de la cámara RGBD y una fase final apoyada en datos del LIDAR. Aunque el comportamiento resulta el previsto, presenta dos problemas principales:

- La acción de chute empleada es una acción predefinida de alto nivel del software Unitree (shake hand), que mueve la pata delantera de forma invariante, por lo que la pata debe de estar perfectamente alineada con la pelota.

- Debido a que los comandos de navegación de alto nivel usados están pensados para desplazamientos continuos y ágiles, no para ajustes y pequeñas distancias, la fase final de acercamiento/alineamiento realiza gran número de rectificaciones que se anulan entre si. Esto produce desplazamientos sucesivos en sentidos opuestos sin converger a la posición deseada, incluso golpeando en ocasiones el objeto de pateo de forma accidental comprometiendo la integridad del experimento.

Vistas estas limitaciones, proponemos llevar a cabo las siguientes mejoras:

1. Nuevo comportamiento de pateo a bajo nivel: Por bajo nivel se entiende el control directo de la posición o torque de cada motor del robot. Esto incluye también el control de los motores que no participan en el pateo pero que son necesarios para mantener la estabilidad. Se desarrollará entonces un nuevo comportamiento que permita mayor control sobre la pierna de golpeo y la selección del punto de impacto sobre la pelota, para así determinar mejor su trayectoria final. Esto añadirá cierta flexibilidad a la fase final del proceso de acercamiento/alineación, pues en vez de ajustar la posición global del robot respecto a la pelota, podremos calcular el ángulo del hombro para modificar el plano de movimiento de la pata. De este modo se controla como incide la pata sobre la pelota y, con esto, la trayectoria resultante.

2. Comportamientos de desplazamiento y alineamiento a bajo nivel más preciso que los actualmente disponible: El objetivo es acercarse con suficiente precisión al objeto para poder usar el comportamiento anterior, pero sin tocarlo accidentalmente. Para conseguirlo es necesario mover cada articulación en secuencia pero manteniendo el equilibrio del robot, de modo que pueda avanzar, retroceder, desplazarse lateralmente o girar con la precisión requerida y reducir así las rectificaciones.

El principal reto técnico del proyecto es procesar la información sensorial del robot, controlar a bajo nivel sus motores y mantener el equilibrio, con el fin de lograr un golpeo preciso que siga una trayectoria determinada.

# Obxectivos concretos

1. Creación de un entorno de simulación reporducible mediante un contenedor Docker con Ubuntu, ROS2 y Mujoco con todos los paquetes y librerias necesarias para el control del robot Go2.
2. Desarrollar un comportamiento a bajo nivel que permita realizar el pateo de una pelota hacia un objetivo.
   1. Verificar el correcto funcionamiento del pateo y la estabilidad del robot usando el entorno simulado.
   2. Validar el comportamiento desarrollado en el robot real, comprobando que los pateos son efectivos, el robot mantiene el equilibrio y la pelota sigue la trayectoria esperada.
3. Desarrollar comportamientos a bajo nivel que permitan realizar desplazamientos y alineamientos más precisos que los actuales en distancias cortas sin comprometer la integridad del robot ni tocar el objeto a patear. 
   1. Verificar el funcionamiento en el entorno simulado.
   2. Validar el comportamiento en el robot real.
4. Integrar y verificar el correcto funcionamiento de la integración de los comportamientos desarrollados, tanto en simulación como en el robot real, como parte del sistema completo de acercamiento y chute. 

# Método de traballo

Se empleará una metodología ágil, con una planificación iterativa y flexible, estructurando el desarrollo en ciclos cortos que faciliten la identificación temprana de problemas y la evolución gradual del sistema mediante la implementación y ajuste de los distintos módulos.

Como plataforma de gestión de nodos, sensores y lógica de control se usará ROS2, lo que permitirá integrar los datos de la cámara RGB, la cámara LIDAR 3D y los modelos de inteligencia artificial ya disponibles. Para implementar los comportamientos a bajo nivel se utilizará la interfaz de unitree_ros2 (en C++ o Python), mediante mensajes del Unitree SDK2 (como LowCmd). Para definir estos comportamientos indicaremos directamente los valores de las articulaciones, realizando transiciones entre posturas concretas (definidas por los valores anteriores) para producir los movimientos deseados.

Dado el riesgo para la integridad del robot, el desarrollo y las pruebas iniciales se realizarán en un entorno simulado: ROS2 en un contenedor Docker sobre Ubuntu junto con un software de simulación con físicas realistas (como MuJoCo), donde se modelarán el robot, los objetos y el terreno. Una vez validado el comportamiento en simulación, se realizarán pruebas sobre el robot físico.

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
  - Robot cuadrúpedo tipo Go2 de Unitree (con cámara RGB y LIDAR 3D)
  - PC para desarrollo del software y control del robot
- Software:
  - ROS2 y sus derivados (Rviz, rosbag, etc.)
  - Simulador robótico Mujoco
  - Contenedores Docker
  - Herramientas de visión por computador (OpenCV, YOLO, etc.)
  - SDK del Go2

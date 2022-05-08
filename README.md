[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-c66648af7eb3fe8bc4f294546bfd86ef473780cde1dea487d3c4ff354943c9ae.svg)](https://classroom.github.com/online_ide?assignment_repo_id=7776068&assignment_repo_type=AssignmentRepo)
# attention-k-bots

## Introducción

En esta práctica realizaremos un sistema cognitivo sencillo donde se identificará objetos específicos como mesas, muebles, etc.

## Tareas a completar
  - Detectar todos los objetos de la escena y seleccionar los que nos interesa
  - Mantener la mirada sobre el objeto siempre y cuando esté dentro de nuestro rango de visión. En el caso de que haya varios irá alternando entre ellos. 
  
    ** El robot será teleoperado

## Desarrollo


**2 Nodos: *attention* y *object_attention***

### attention: 
Añadimos cada objeto como nodo en el grafo y lo conectamos con el nodo odom.
Luego, conectaremos los objetos que se desea mirar al nodo del robot (kbot) mediante el arco want_see.

![rosgraph](https://user-images.githubusercontent.com/78978037/167293557-3e53c63a-cd8e-4213-a16d-032a627cc185.png)

### object_attention













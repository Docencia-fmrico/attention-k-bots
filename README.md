[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-c66648af7eb3fe8bc4f294546bfd86ef473780cde1dea487d3c4ff354943c9ae.svg)](https://classroom.github.com/online_ide?assignment_repo_id=7776068&assignment_repo_type=AssignmentRepo)
# attention-k-bots

## Introducción

En esta práctica realizaremos un sistema cognitivo sencillo donde se identificará objetos específicos como mesas, muebles, etc.

## Tareas a completar
  - Detectar todos los objetos de la escena y seleccionar los que nos interesa
  - Mantener la mirada sobre el objeto siempre y cuando esté dentro de nuestro rango de visión. En el caso de que haya varios irá alternando entre ellos. 
  
    ** El robot será teleoperado

## Desarrollo


2 Nodos: *attention* y *stare_at_object*

### attention: 
Añadimos todos los objetos de la escena en el grafo y lo conectamos con el nodo odom. Luego, conectamos los objetos que se desea mirar al nodo del robot (kbot) mediante el arco want_see.

![rosgraph](https://user-images.githubusercontent.com/78978037/167293557-3e53c63a-cd8e-4213-a16d-032a627cc185.png)

### stare_at_object
En este nodo implementamos la tarea de mirar objetos.



## Cómo ejecutar
Lanzamos el primer launcher para el nodo attention, este launcher leerá de un fichero de parámetros yaml los objetos que se desea mirar 
```
ros2 launch attention attention_launch.py
```

A continuación, lanzamos el launcher del nodo stare_at_object, el cual leerá de un fichero de parámetros la distancia máxima a mirar y el tiempo que mantiene la mirada en caso de que haya varios objetos
```
ros2 launch stare_at_object stare_at_object_launch.py
```










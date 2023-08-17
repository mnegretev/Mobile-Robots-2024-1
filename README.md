# Mobile-Robots-2024-1
Software para el curso "Robots Móviles" semestre 2024-1, FI, UNAM

## Requerimientos

* Ubuntu 20.04
* ROS Noetic http://wiki.ros.org/noetic/Installation/Ubuntu

## Instalación

Nota: se asume que ya se tiene instalado Ubuntu y ROS.

* $ cd
* $ git clone https://github.com/mnegretev/Mobile-Robots-2024-1
* $ cd Mobile-Robots-2024-1
* $ ./Setup.sh
* $ cd catkin_ws
* $ catkin_make -j2 -l2

## Pruebas

Para probar que todo se instaló y compiló correctamente:

* $ cd 
* $ source Mobile-Robots-2024-1/catkin_ws/devel/setup.bash
* $ roslaunch surge_et_ambula justina_gazebo.launch

Si todo se instaló y compiló correctamente, se debería ver un visualizador como el siguiente:
![rviz](https://github.com/mnegretev/Mobile-Robots-2024-1/assets/17507149/f4642036-ca45-4ab8-be41-d23bc4ab3720)

Un ambiente simulado como el siguiente:
![gazebo](https://github.com/mnegretev/Mobile-Robots-2024-1/assets/17507149/d8d33093-51f2-4ce1-81a3-79339f86506c)

Y una GUI como la siguiente:
![GUIExample](https://github.com/mnegretev/Mobile-Robots-2024-1/assets/17507149/ee62eb5d-ddf8-45f3-a16a-88a9a45a0721)

Se puede descargar una máquina virtual para [VirtualBox](https://www.virtualbox.org/wiki/Downloads) con Ubuntu y ROs ya instalado de [esta dirección.](https://drive.google.com/drive/folders/1T-gDsEb5Opl4gbx2qPQYEelyPnEe91Yk?usp=share_link) <br>
El siguiente video muestra un ejemplo de cómo usar la máquina virtual:

https://github.com/mnegretev/Mobile-Robots-2024-1/assets/17507149/d4d0ab86-cd33-48c6-a669-11ab6894908d

Se recomienda configurar la máquina virtual con 4 CPUs y 4GB de RAM.<br>
Usuario: justina <br>
Contraseña: pumas


## Contacto
Dr. Marco Negrete<br>
Profesor Asociado C<br>
Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
marco.negrete@ingenieria.unam.edu<br>

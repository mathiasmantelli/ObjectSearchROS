Muda pra branch correta:
 git checkout ros-kinetic

Antes de tudo (só na 1ª vez), compila o codigo pelo terminal:
catkin_make

Pra rodar o simulador:
roslaunch phi_exploration example-indoor-gazebo.launch

Pra rodar um controle simples do robô (que não está funcionando bem):
rosrun phi_exploration simple_control.py

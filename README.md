# Navegação SLAM

> Não se esqueça de acessar o tutorial de ROS2 para facilitar esta 
tarefa!

É essencial para os robôs autônomos que naveguem de forma eficaz em ambientes desconhecidos <br>
ou dinâmicos. Para isso, surgiu o SLAM (Simultaneous Localization and Mapping), uma técnica <br>
na qual o robô mapeia um ambiente desconhecido enquanto localiza sua posição no mesmo. 

### Bibliotecas de navegação.
Hoje, existem diversas bibliotecas para implementação de SLAM, comoa Nav2 -- utilizada para o <br>
planejamento de trajetória, a evasão de obstáculos e outras tarefas de navegação em robótica <br>
-- e o pacote SLAM TollBox. Sendo assim, os utilizaremos para criar nosso próprio mapa. Então, <br>
 certifique-se de instalar:
```
 $ sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
 $ sudo apt install ros-humble-slam-toolbox
```

### O simulador Gazebo
Uma das grandes vantagens do ROS2 é a possibilidade de simularmos robôs e ambientes de forma <br>
simples. Para isso, utilizaremos o simulador Gazebo, o qual possui uma ótima integração com o <br>
 sistema operacional. Para instalar o Gazebo, execute:
```
$ sudo apt install ros-humble-gazebo-*
```

### Mapeando . . .
Vamos mapear nosso ambiente no Gazebo! Para isso, inicialize o ROS2 em um novo terminal e <br>
inicialize o turtlebot3 em uma casa simulada no Gazebo. <br> 
Lembre-se: Para iniciar o conjunto de pacotes do Turtlebot3, primeiro  precisamos exportar<br>
 uma variável de ambiente para especificar qual versão queremos iniciar (burger, waffle, <br>
 waffle pi). Vamos usar a versão "burguer", equivalente à versão física que possuimos.
```
$ source /opt/ros/humble/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

Agora, em um novos terminais, inicialize o Nav2 e o SLAM Toolbox, a implementação de SLAM <br>
escolhida, fornecida pela Nav2:
```
$ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
```
$ ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

No quarto terminal, iniciaremos o RViz, uma ferramenta de visualização 3D que permite os <br>
usuários visualizarem dados de sensores e informações do robô em tempo real. Para facilitar, <br>
podemos começar com uma configuração do RViz já existente para o Nav2.
```
$ ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Anteriormente, você já utilizou o teclado para controlar a simulação do robô. Então, inicialize <br> 
o ROS2 em um novo terminal e execute o pacote teleop:
```
$ ros2 run turtlebot3_teleop teleop_keyboard
```

Tudo pronto! Agora, através do LIDAR, o Turtlebot3 irá mapear o ambiente e localizar sua posição <br>
 no mesmo. Por isso, fique atento: ele só enxerga o que está na altura do sensor.

Quando estiver satisfeito com seu mapa, salve-o:
```
$ ros2 run nav2_map_server map_saver_cli -f my_map
```
Agora, você deve ter dois novos arquivos:
1. `my_map.yaml`: Contém os metadados do mapa e o caminho para o arquivo de imagem.
2. `my_map.pgm`: Este é o arquivo de imagem com pixels brancos, pretos e cinza, representando o <br>
espaço livre, ocupado e desconhecido.

Perfeito! Agora você poderá utilizar o mapa que criou para tarefas como navegação autônoma <br>
e planejamento de trajetória.

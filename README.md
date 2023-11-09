# Navegação SLAM

> Não se esqueça de acessar o tutorial de ROS2 para facilitar esta 
tarefa!<br> Assumimos que você já possui o [sistema ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) instalado <br>


É essencial para os robôs autônomos que naveguem de forma eficaz em ambientes desconhecidos <br>
ou dinâmicos. Para isso, surgiu o SLAM (Simultaneous Localization and Mapping), uma técnica <br>
na qual o robô mapeia um ambiente desconhecido enquanto localiza sua posição no mesmo. 

### Bibliotecas de navegação.
Hoje, existem diversas bibliotecas para implementação de SLAM, comoa Nav2 -- utilizada para o <br>
planejamento de trajetória, a evasão de obstáculos e outras tarefas de navegação em robótica <br>
-- e o pacote SLAM TollBox. Sendo assim, os utilizaremos para criar nosso próprio mapa. Então, <br>
 certifique-se de instalar:
```
 sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
 sudo apt install ros-humble-slam-toolbox
```

### O simulador Gazebo
Uma das grandes vantagens do ROS2 é a possibilidade de simularmos robôs e ambientes de forma <br>
simples. Para isso, utilizaremos o simulador Gazebo, o qual possui uma ótima integração com o <br>
 sistema operacional. Para instalar o Gazebo, execute:
```
 sudo apt install ros-humble-gazebo-*
```

### O Turtlebot3
O robô que utilizaremos para mapear o ambiente é o Turtlebot3, muito utilizado na área educacional. <br>
Ele apresenta um sensor de distância à laser, o LIDAR que utilizaremos para detectar objetos. Logo, <br>
é importante não se esquecer que os raios atingem apenas a altura do sensor. A seguir você pode <br>
observar a versão Burger do robô e suas features:<br>
<img src="https://cdn.shopify.com/s/files/1/0928/0230/files/turtlebot3_main_components_burger.png?931120592119287692" alt="Turtlebot3 burger" width="450">

Sendo desenvolvido para o ROS2, o robô possui seus próprios pacotes, crie seu workspace para o <br>
turtlebot3 e clone os arquivos referentes à ele:
```
 mkdir -p ~/turtlebot3_ws/src
 cd ~/turtlebot3_ws/src/
 git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
 git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
 git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
 git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

Compile:
```
source /opt/ros/<distro>/setup.bash
cd ~/turtlebot3_ws && colcon build --symlink-install
source ~/turtlebot3_ws/install/setup.bash
```

Se lembra que devemos inicializar o ROS a cada terminal que abrimos? Agora, para configurar<br>
seu workspace para uso, é necessário digitar em cada terminal:
```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=30 #TURTLEBOT3

```
Não se esqueça que é possível adicionar os comandos para o arquivo bashrc, evitando ter que <br>
digita-los novamente.
Como você deve ter notado, para iniciar o conjunto de pacotes do robô, precisamos exportar<br>
uma variável de ambiente para especificar qual versão queremos iniciar (burger, waffle, <br>
waffle pi). Vamos usar a versão "burguer", equivalente à versão física que possuimos.


### Mapeando . . .
Vamos mapear nosso ambiente no Gazebo! Para isso, inicialize o ROS2 em um novo terminal e <br>
inicialize o turtlebot3 em uma casa simulada no Gazebo. <br> 
```
 ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

Agora, em um novos terminais, inicialize o Nav2 e o SLAM Toolbox, a implementação de SLAM <br>
escolhida, fornecida pela Nav2:
```
 ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
```
 ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

No quarto terminal, iniciaremos o RViz, uma ferramenta de visualização 3D que permite os <br>
usuários visualizarem dados de sensores e informações do robô em tempo real. Para facilitar, <br>
podemos começar com uma configuração do RViz já existente para o Nav2.
```
 ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Anteriormente, você já utilizou o teclado para controlar a simulação do robô. Então, inicialize <br> 
o ROS2 em um novo terminal e execute o pacote teleop:
```
 ros2 run turtlebot3_teleop teleop_keyboard
```

Tudo pronto! Agora, através do LIDAR, o Turtlebot3 irá mapear o ambiente e localizar sua posição <br>
no mesmo.

Quando estiver satisfeito com seu mapa, salve-o:
```
 ros2 run nav2_map_server map_saver_cli -f my_map
```
Agora, você deve ter dois novos arquivos:
1. `my_map.yaml`: Contém os metadados do mapa e o caminho para o arquivo de imagem.
2. `my_map.pgm`: Este é o arquivo de imagem com pixels brancos, pretos e cinza, representando o <br>
espaço livre, ocupado e desconhecido.

Perfeito! Agora você poderá utilizar o mapa que criou para tarefas como navegação autônoma <br>
e planejamento de trajetória. Então, desafio-os à fazer o Turtlebot3 caminhar sozinho até um ponto <br>
especificado por você no mapa do RViz!


### Possíveis problemas: 
1. Ao tentar compilar seu workspace usando colcon build, é possível que receba a seguinte mensagem <br>
de erro: “setup.py install is deprecated. Use build and pip and other standards-based tools”. <br>
Nesse caso, a sua versão do python deve ser 58.2.0 ou anterior. Para isso, digite:<br>
	$ pip install setuptools==58.2.0.

2. Caso seu computador avise que algum pacote do turtlebot3 está faltando mesmo após seguir todos <br>
os passos, baixe:<br>
	$ sudo apt install ros-humble-turtlebot3*

3. Caso o Gazebo mostre alguma mensagem de erro tente rodar o seguinte comando antes de tentar <br>
resolver de alguma outra forma: <br>
        $ killall -9 gzserver<br>
Um problema conhecido dessa versão do gazebo é que às vezes o server side dele acaba não sendo <br>
encerrado corretamente após uma execução.<br>

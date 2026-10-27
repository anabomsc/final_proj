# Fusão de Mapas 2D para Sistemas Multi-robôs
Código da Monografia apresentada ao curso de Engenharia Elétrica da Universidade Federal do Maranhão, como parte dos requisitos necessários para obtenção do grau de Bacharel em Engenharia Elétrica.

## Metodologia
O objetivo é usar múltiplos robôs para exploração por teleoperação assíncrona de um espaço interno e desconhecido, de forma a explorar individualmente partes suficientes do ambiente para formar um mapa global coeso através da fusão dos mapas locais produzidos. Quando as partes individuais forem suficientes para representar o ambiente inteiro de forma conjunta, os robôs param na mesma área do ambiente e o map merging ocorre, completando a tarefa.

O sistema proposto implementa simulação dos robôs, sensores e ambientes no Gazebo Fortress, enquanto toda a estrutura de SLAM individual, MR-SLAM e comunicação sem fio é feita pelo ROS 2 Humble.

<img src="https://github.com/anabomsc/final_proj/blob/main/assets/complete-overview.png" width="950">

## Cite este Trabalho
```
REF TBA
```

## Uso do Repositório
1. Crie um workspace
  ```shell
  mkdir -p ~/mono_ws/src
  cd ~/mono_ws/src
  ```

2. Clone o repositório
  ```shell
  git clone https://github.com/anabomsc/final_proj.git
  ```
3. Dê permissão de execução para os arquivos em `experiments` e `scripts`

4. Construa o pacote
  ```shell
  cd ~/mono_ws
  colcon build --packages-select final_proj
  ```
5. Abra um novo terminal

6. Rodar experimentos
```
run_session.sh
```

## Reconhecimento
* [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
* [Gazebo Fortress](https://gazebosim.org/docs/fortress/getstarted/)
* [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/humble)
* [Josh Newans - Articulated Robotics](https://github.com/joshnewans/articubot_one)
* [Map Fusion Method](https://github.com/abdulkadrtr/mapMergeForMultiRobotMapping-ROS2)
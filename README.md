# Fusão de Mapas 2D para Sistemas Multi-robôs
Código da Monografia apresentada ao curso de Engenharia Elétrica da Universidade Federal do Maranhão, como parte dos requisitos necessários para obtenção do grau de Bacharel em Engenharia Elétrica.

## Metodologia
O objetivo é usar múltiplos robôs para exploração por teleoperação assíncrona de um espaço interno e desconhecido, de forma a explorar individualmente partes suficientes do ambiente para formar um mapa global coeso através da fusão dos mapas locais produzidos. Quando as partes individuais forem suficientes para representar o ambiente inteiro de forma conjunta, os robôs param na mesma área do ambiente e o map merging ocorre, completando a tarefa.

O sistema proposto implementa simulação dos robôs, sensores e ambientes no Gazebo Fortress, enquanto toda a estrutura de SLAM individual, MR-SLAM e comunicação sem fio é feita pelo ROS 2 Humble.

IMAGES TBA

## Monografia
```
REF TBA
```

## Uso do Repositório
1. Crie um workspace
  ```shell
  mkdir -p ~/mr_ws/src
  cd ~/mr_ws/src
  ```

2. Clone o repositório
  ```shell
  git clone https://github.com/anabomsc/rmp_2026.git
  ```

3. Construa o pacote
  ```shell
  cd ~/mr_ws
  colcon build --packages-select final_proj
  ```
4. Source o workspace
  - Abra um novo terminal

5. Dê permissão de execução para os arquivos em `experiments` e `scripts`

6. Rodar experimentos
```
run_session.sh
```
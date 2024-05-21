# TurtlesimDraw - Gustavo Wagon Widman

## Descrição

Este é um pacote ROS desenvolvido para uma atividade ponderada que servia de introdução ao ROS. O pacote inclui uma aplicação Python que utiliza o framework ROS (Robot Operating System) para controlar uma tartaruga no simulador `turtlesim`. A tartaruga é programada para desenhar os números "6" e "9" no simulador (nice).

## Instalação

Certifique-se de que você possui o ROS Humble instalado em sua máquina.

Para instalar o pacote, clone o repositório:

```bash
git clone https://github.com/GustavoWidman/TurtlesimDraw.git
```

Em seguida, compile o pacote:

```bash
cd TurtlesimDraw
colcon build
```

## Execução

Agora abra, em um terminal, o simulador `turtlesim`:

```bash
ros2 run turtlesim turtlesim_node
```

E em outro terminal, execute o pacote:

```bash
source install/setup.bash
ros2 run ros_turtle_draw main
```

## Demonstração

A tartaruga desenhará os números "6" e "9" no simulador `turtlesim`. Segue o vídeo da demonstração:

[Screencast from 2024-05-06 00-59-45.webm](https://github.com/GustavoWidman/PonderadaROS/assets/123963822/40112491-847f-4eb3-901b-60c2136fa9d4)

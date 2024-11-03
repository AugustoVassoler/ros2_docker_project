# ROS2 Docker Project

Este projeto fornece uma estrutura de pacotes ROS2 desenvolvida para execução em um ambiente Docker. Os pacotes realizam tarefas relacionadas ao monitoramento de sistema, simulação de sensor e cálculos numéricos.

## Descrição do Projeto

Este repositório contém três pacotes ROS2 implementados em Python. Cada pacote realiza uma tarefa específica:

1. **Pacote 1 - `package_1`**: Publica informações sobre a quantidade total de memória, uso de memória RAM em Gigabyte e percentual de uso, a cada segundo.

2. **Pacote 2 - `package_2`**: Simula a leitura de um sensor com uma taxa de amostragem de 1 Hz. Os dados passam por um filtro de média móvel, que considera os últimos 5 valores adquiridos. Este pacote inclui duas interfaces de serviço:
   - Uma interface retorna os últimos 64 valores gerados pelo filtro.
   - A outra interface zera os dados gerados pelo filtro.

3. **Pacote 3 - `package_3`**: Calcula o décimo número primo utilizando uma interface de ação. Durante o cálculo, o pacote fornece feedback intermediário e, ao final, o resultado completo da ação.

## Requisitos

- [Docker](https://www.docker.com/products/docker-desktop) instalado no sistema operacional.
- [ROS2 Humble](https://docs.ros.org/en/humble/) (recomenda-se usar o Docker para simplificar a configuração).

## Estrutura do Projeto

```plaintext
ros2_docker_project/
├── Dockerfile               # Dockerfile para construir a imagem ROS2 com os pacotes
├── packages/
│   ├── package_1/           # Pacote 1: Publicador de informações sobre memória
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── src/
│   │       └── memory_publisher.py
│   ├── package_2/           # Pacote 2: Simulador de sensor com filtro de média móvel
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── src/
│   │       └── sensor_simulator.py
│   └── package_3/           # Pacote 3: Servidor de ação para cálculo do décimo primo
│       ├── package.xml
│       ├── setup.py
│       └── src/
│           └── prime_number_action.py
└── README.md                # Instruções detalhadas do projeto
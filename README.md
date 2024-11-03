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
```


## Instruções para Construção e Execução
### Clonar o repositório:
```bash
git clone https://github.com/AugustoVassoler/ros2_docker_project.git
cd ros2_docker_project
```

### Construir a imagem Docker:
```bash
docker build -t ros2_humble_image .
```
Esse comando cria uma imagem Docker chamada ros2_humble_image usando as instruções do Dockerfile. O ponto (.) no final do comando indica ao Docker para procurar o Dockerfile no diretório atual.

### Executar o container:
```bash
docker run -it ros2_humble_image
```
Este comando cria e inicia um novo container baseado na imagem ros2_humble_image. A opção -it permite a interação direta com o container pelo terminal.

### Execução dos pacotes ROS2 dentro do container:
Uma vez dentro do container, é possível executar cada pacote conforme descrito abaixo.

#### Pacote 1 - Publicador de Informações de Memória
```bash
ros2 run package_1 memory_publisher
```

#### Pacote 2 - Simulador de Sensor com Filtro de Média Móvel
```bash
ros2 run package_2 sensor_simulator
```
- Para visualizar os dados do sensor publicados, é necessário abrir um novo terminal e executar o seguinte comando: 
```bash
ros2 topic echo /sensor_data
```
- Para chamar o serviço que retorna os últimos valores do filtro de média móvel:
```bash
ros2 service call /get_last_values std_srvs/srv/Trigger
```
- Para zerar os dados do filtro de média móvel:
```bash
ros2 service call /reset_filter std_srvs/srv/Trigger
```

#### Pacote 3 - Cálculo do Décimo Número Primo com Feedback Intermediário
```bash
ros2 run package_3 prime_number_action
```
- Para enviar uma requisição de ação que calcula o décimo número primo e recebe feedback intermediário, é necessário abrir um novo terminal e executar o seguinte comando:
```bash
ros2 action send_goal /find_nth_prime example_interfaces/action/Fibonacci "{order: 10}"
```

## Limpeza
O container pode ser encerrado com o comando 'exit'. Para removê-lo e suas imagens criadas, pode-se utilizar:
```bash
# Listar todos os containers
docker ps -a
# Remover um container específico
docker rm <container_id>
# Remover a imagem Docker
docker rmi ros2_humble_image
```
O '<container_id>' deve ser substituído pelo ID do container a ser removido, o que pode ser encontrado com o comando 'docker ps -a'.

## Teste e validação
- **Pacote 1:** Deve-se verificar no console as informações de memória, uso em GB e percentual de uso.

- **Pacote 2:** Veja os dados filtrados no tópico '/sensor_data' e use os serviços para visualizar ou zerar os últimos valores.

- **Pacote 3:** O cálculo do décimo primo fornecerá feedback e o resultado final via interface de ação.
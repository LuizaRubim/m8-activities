# Atividades de programação módulo 8


## Atividade 1: Robô no labirinto

Essa atividade resolve um labirinto por meio de um serviço ROS de movimentação com base na navegação reativa, ou seja, o robô toma decisões de movimento com base em sensores de proximidade indicando blocos livres ou ocupados ao seu redor. O objetivo é chegar ao ponto final do labirinto sem conhecimento do mapa, apenas com a navegação reativa. Foi utilizado o serviço ROS de movimentação e o código foi construido em C++;

### Como executar:

Tendo em base que a confurguração do ambiente ROS já foi feita, siga os passos abaixo:

1. Clone o repositório em sua máquina:
```bash
git clone git@github.com:LuizaRubim/m8-activities.git
```

2. Entre na pasta do projeto:
```bash
cd m8-activities/maze_workspace/src
```

3. Compile o projeto:
```bash
colcon build
```

4. Execute o labirinto:
```bash
ros2 run cg maze
```

5. Em outro terminal, vá até a pasta src e execute o serviço de movimentação:
```bash
ros2 run maze_finder maze_client
```

6. Acompanhe o robô se movimentando pelo labirinto até chegar ao ponto final.

## Atividade 2: Chatbot com RAG 

Essa atividade consiste em um chatbot que utiliza o modelo RAG (Retrieval Augmented Generation) para responder perguntas sobre Workshop rules and safety considerations, considerando esse [documento](https://www.ku.edu.bh/wp-content/uploads/2016/09/Engineering-workshop-health-and-safety-guidelines-catalog.pdf)

### Como executar:

1. Clone o repositório em sua máquina:
```bash
git clone git@github.com:LuizaRubim/m8-activities.git```
```

2. Entre na pasta do projeto:
```bash
cd m8-activities/chatbot_workspace/src
```

3. Crie um ambiente virtual e instale as dependências:
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

4. Execute o chatbot:
```bash
streamlit run app.py
```


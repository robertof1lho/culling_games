# Projeto Culling Games (cg)

Este documento fornece um tutorial sobre como construir, executar e interagir
com o projeto Culling Games ROS 2.

## Vídeo de Demonstração e Explicação dos Algoritmos

Assista à demonstração do projeto e à explicação detalhada dos algoritmos de navegação no vídeo abaixo:

[Link do vídeo de Demonstração e Explicação no Drive](https://drive.google.com/file/d/1ySrMYpMOAKik-tnmmJLH3cR5QuNpGbs_/view?usp=sharing)

## 1. Construindo o Workspace

Antes de executar qualquer parte do projeto, você precisa construir os pacotes.
Navegue até a raiz do workspace e execute:

```bash
colcon build
```

Este comando irá compilar todos os pacotes (`cg`, `cg_interfaces`, `cg_teleop`, `cg_navigator`).
Lembre-se de "source" o workspace em qualquer novo terminal que você abrir:

```bash
source install/setup.bash
```

## 2. Ponderada - Navegação Automática (`cg_navigator`)

### 2.1 Parte 1 -`navigator_node` (usa o mapa completo)

1. Terminal 1 (jogo):
   ```bash
   ros2 run cg maze
   ```
2. Terminal 2 (auto-navegação completa):
   ```bash
   source install/setup.bash
   ros2 run cg_navigator navigator_node
   ```

### 2.2 Parte 2 -`mapper_node` (mapeia com sensores e depois navega)

1. Terminal 1 (jogo):
   ```bash
   ros2 run cg maze
   ```
2. Terminal 2 (DFS para mapear, reset, BFS final até o alvo):
   ```bash
   source install/setup.bash
   ros2 run cg_navigator mapper_node
   ```

## 3. Executando o Jogo

O jogo principal é uma janela Pygame que exibe o labirinto e o movimento do
robô.

Para iniciar o jogo, execute o seguinte comando em um terminal:

```bash
ros2 run cg maze
```

### Opções de Carregamento do Labirinto

Você pode especificar como o labirinto é carregado usando argumentos adicionais:

*   **Carregar um Labirinto Aleatório (Padrão):** Se nenhum argumento for fornecido, o jogo selecionará um labirinto aleatório do diretório `src/cg/maps`.
    ```bash
    ros2 run cg maze
    ```
*   **Carregar um Labirinto Específico:** Para carregar um labirinto pelo nome do arquivo (por exemplo, `test.csv`):
    ```bash
    ros2 run cg maze -- --map test.csv
    ```
*   **Gerar um Novo Labirinto:** Para gerar um novo labirinto aleatório e usá-lo imediatamente (esta opção tem precedência sobre `--map`):
    ```bash
    ros2 run cg maze -- --generate
    ```

## 4. Controlando o Robô

Existem duas maneiras de controlar o robô: usando o nó de teleoperação fornecido
ou enviando chamadas de serviço.

### Método A: Usando o Nó de Teleoperação por Teclado

Esta é a maneira mais fácil de jogar.

1.  Em um **terminal separado** (enquanto o comando `ros2 run cg maze` ainda
    estiver em execução), inicie o nó de teleoperação:
    ```bash
    ros2 run cg_teleop teleop_keyboard
    ```
2.  O terminal exibirá as teclas de atalho. Use as seguintes teclas neste
    terminal para mover o robô na janela do jogo:
    *   **Cima:** `w`, `k`, ou a tecla Seta para Cima
    *   **Baixo:** `s`, `j`, ou a tecla Seta para Baixo
    *   **Esquerda:** `a`, `h`, ou a tecla Seta para Esquerda
    *   **Direita:** `d`, `l`, ou a tecla Seta para Direita

### Método B: Enviando Chamadas de Serviço Manuais

Você também pode enviar comandos de movimento individuais usando o serviço
`/move_command`. Isso é útil para scripts ou depuração.

Para mover o robô um passo, use o comando `ros2 service call`. Por exemplo, para
mover para cima:

```bash
ros2 service call /move_command cg_interfaces/srv/MoveCmd "{direction: 'up'}"
```

Substitua `'up'` por `'down'`, `'left'` ou `'right'` para outras direções.

## 5. Sensoriamento do Ambiente

O robô publica continuamente seus arredores imediatos em um tópico. Isso simula
dados de sensores, mostrando o que está nas 8 células adjacentes (incluindo
diagonais).

*   **Tópico:** `/culling_games/robot_sensors`
*   **Tipo de Mensagem:** `cg_interfaces/msg/RobotSensors`

Para ver esses dados em tempo real, abra um novo terminal e execute:

```bash
ros2 topic echo /culling_games/robot_sensors
```

Você verá um fluxo de mensagens mostrando o que está nas células `up`, `down`,
`left`, `right`, `up_left`, etc., em relação ao robô.

## 6. Reiniciando o Jogo

O serviço `/reset` permite reiniciar o tabuleiro do jogo. Ele suporta dois
modos.

### Reiniciando o Labirinto Atual

Se você quiser tentar o *mesmo* labirinto novamente desde o início.

*   **Com Teleoperação:** Pressione a tecla `r` no terminal `cg_teleop`.
*   **Comando Manual:**
    ```bash
    ros2 service call /reset cg_interfaces/srv/Reset "{is_random: false}"
    ```

### Carregando um Novo Labirinto Aleatório

Se você quiser um novo desafio com um labirinto novo e selecionado
aleatoriamente.

*   **Com Teleoperação:** Pressione a tecla `n` no terminal `cg_teleop`.
*   **Comando Manual:**
    ```bash
    ros2 service call /reset cg_interfaces/srv/Reset "{is_random: true}"
    ```
A resposta do serviço informará o nome do arquivo do novo labirinto que foi
carregado.

## 7. Obtendo os Dados Completos do Labirinto

Se você quiser obter o layout de todo o labirinto atual (por exemplo, para
construir um mapa externo), você pode usar o serviço `/get_map`.

```bash
ros2 service call /get_map cg_interfaces/srv/GetMap
```

Isso retornará uma representação "achatada" da grade do labirinto e suas
dimensões.

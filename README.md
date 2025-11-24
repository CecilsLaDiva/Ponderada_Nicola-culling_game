# 🧠 Culling Games - Navegação e Mapeamento com ROS 2 (Versão C++)

> [!IMPORTANT]
> **ESTA É A VERSÃO EM C++ DA SOLUÇÃO.**
> A lógica dos algoritmos foi portada para C++ para atender aos requisitos de desempenho e especificações do projeto, mantendo o projeto antigo.

Este repositório contém a solução desenvolvida para a atividade ponderada de Robótica Computacional. O projeto aborda dois desafios clássicos de robótica móvel em labirintos:

* **Parte 1:** Com acesso ao mapa, desenvolver um algoritmo para encontrar a rota otimizada até o alvo.
* **Parte 2:** Mapeamento de um labirinto desconhecido. O robô deve navegar, mapear o ambiente e comprovar que o mapa gerado é suficiente para reproduzir a rota.

## 🎥 Demonstração

> **Nota:** O vídeo demonstrativo completo do funcionamento deste projeto, comprovando a execução dos algoritmos, encontra-se anexado junto ao link deste repositório na plataforma **Adalove**.

---

## 📂 Estrutura do Projeto

A solução foi desenvolvida em um novo pacote chamado `cg_solution`. A estrutura do workspace contém:

1.  **`cg`**: Pacote original do simulador (Python) que gerencia o jogo e a interface gráfica.
2.  **`cg_solution`**: Pacote desenvolvido (C++) contendo os nós de controle e inteligência do robô.

Os principais scripts desenvolvidos (`src/cg_solution/src/`) são:

### PARTE 1

* **`resolver.cpp`:** Implementa o algoritmo de **Busca em Largura (BFS)**. Solicita o mapa completo ao serviço, converte para uma matriz C++ e calcula a rota mais curta instantaneamente.

### PARTE 2

* **`resolverpt2.cpp` (DFS Guiado):** Implementa a exploração de labirinto desconhecido usando **DFS com Backtracking + Heurística**.
    * *Diferencial:* Utiliza a geometria conhecida do labirinto (alvo no centro 14,14) para priorizar vizinhos que aproximam o robô do objetivo, otimizando a busca.
* **`semalvopt2.cpp` (DFS Cego):** Implementa a exploração "cega" usando **DFS Puro**.
    * *Diferencial:* Funciona em qualquer cenário, sem pressupor a posição do alvo, garantindo varredura completa se necessário.

---

## 🛠️ Instalação e Compilação

Pré-requisitos: **ROS 2 (Jazzy ou Humble)**, compiladores C++ e Python 3 instalados em ambiente Linux/WSL.

1.  **Pegue o repositório:**
    ```bash
    Baixe as files ncessárias do repositório para dentro da sua maquina e entre no local correto
    ```

2.  **Compile os pacotes:**
    É necessário compilar tanto o jogo (`cg`) quanto a solução (`cg_solution`). Na raiz do workspace, execute:
    ```bash
    colcon build
    ```

3.  **Configure o ambiente:**
    Sempre que abrir um novo terminal, execute:
    ```bash
    source install/setup.bash
    ```

---

## 🚀 Como Rodar

Para executar as soluções, você precisará de **dois terminais**. Certifique-se de rodar o comando `source install/setup.bash` em ambos.

### Parte 1: Navegação com Mapa Conhecido (BFS)

1.  **Terminal 1 (Simulador):**
    Inicie o ambiente do jogo.
    ```bash
    ros2 run cg maze
    ```

2.  **Terminal 2 (Solução C++):**
    Execute o nó que resolve o labirinto via BFS.
    ```bash
    ros2 run cg_solution resolver
    ```

### Parte 2: Mapeamento + Navegação (DFS)

1.  **Terminal 1 (Simulador):**
    Inicie o jogo (sem argumentos para permitir reset de mapas variados).
    ```bash
    ros2 run cg maze
    ```

2.  **Terminal 2 (Solução C++):**
    Execute o nó de exploração.

    * **Opção A (Recomendada - Guiada):**
        ```bash
        ros2 run cg_solution resolverpt2
        ```

    * **Opção B (Busca Cega):**
        ```bash
        ros2 run cg_solution semalvopt2
        ```

> **O que esperar:** O robô explorará o labirinto autonomamente. Ao encontrar o alvo, o terminal exibirá: `ALVO ENCONTRADO! Validando...`, seguido de `SUCESSO! Mapa válido`, confirmando que o mapa gerado na memória do processo C++ permite traçar uma rota segura.

---

## 🧠 Lógica dos Algoritmos

### Solução 1: Busca em Largura (BFS)
* **Estratégia:** Utiliza filas (`std::deque`) para expandir a busca em camadas. Como o grafo não tem pesos (custo de movimento constante), o BFS garante matematicamente o menor caminho.

### Solução 2: DFS com Backtracking
* **Estratégia:**
    1.  **Sensoriamento:** Lê o tópico `/robot_sensors` e atualiza uma matriz de strings (`std::vector<std::vector<std::string>>`) na memória.
    2.  **Decisão:** Escolhe um vizinho não visitado. No caso do `resolverpt2`, ordena os vizinhos pela Distância de Manhattan até o centro `(14,14)`.
    3.  **Backtracking:** Se entrar em um beco sem saída, utiliza uma pilha (`std::vector` usado como Stack) para retornar à última bifurcação.
    4.  **Prova:** Ao final, executa uma função `bfs_check` interna para validar a topologia descoberta.

---
Desenvolvido para a atividade ponderada de Robótica Computacional.

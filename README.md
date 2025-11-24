# Culling Games - Navegação e Mapeamento com ROS 2

> [!IMPORTANT]
> ## *A VERSÃO EM C++ ESTÁ NA BRANCH "c++"*

Este repositório contém a atividade ponderada do professor Nicola Sem6/Mod8. Aqui temos a solução de ambos os desafios da ponderada:

* parte 1: tendo acesso ao mapa, você deverá desenvolver um algoritmo para encontrar a rota otimizada até o alvo;

* parte 2: envolve o mapeamento do labirinto. O algoritmo desenvolvido deve navegar pelo mapa, mapeando-o no processo. A seguir, deve-se comprovar que o mapa criado é suficiente para reproduzir a rota da parte 1.

## Demonstração

> **Nota:** O vídeo comprovando e demonstrando a execução, encontra-se anexado junto ao link deste repositório na plataforma **Adalove** e foi enviado diretamente para o professor (Você 🫵 Nicola).

---

## Estrutura do Projeto

O projeto foi desenvolvido com base no pacote \`cg\` (Culling Games) que estava no repositório do professor. Os principais scripts desenvolvidos foram:

### PARTE 1

* **resolver.py (Parte 1):**
  Implementa o algoritmo de **Busca em Largura (BFS)**. Ele solicita o mapa completo ao servidor e calcula a rota mais curta do robô até o alvo instantaneamente.
  
> [!NOTE]
> **Caminho para a execução resolver.py:**  \`src/cg/cg/resolver.py\`

### PARTE 2

* **resolverpt2.py (Parte 2):**
  Implementa um algoritmo que permite que o robô explore um labirinto desconhecido mas com algumas informações base que o auxiliam, construindo um mapa interno e validando-o ao final.

* **semalvopt2.py (Parte 2):**
  Implementa um algoritmo que permite que o robô explore um labirinto desconhecido utilizando apenas sensores de proximidade, construindo um mapa interno e validando-o ao final.

> [!NOTE]
> **Caminho para a execução resolverpt2.py:**  \`src/cg/cg/resolverpt2.py\
> **Caminho para a execução semalvopt2.py:**  \`src/cg/cg/semalvopt2.py\
  

* **setup.py:**
  Arquivo de configuração do pacote ROS 2, modificado para registrar os executáveis dos scripts acima. Só copiar e colar no local do arquivo antigo, ou adicioonar as novas linhas de código no arquivo já existente.

> `'resolver = cg.resolver:main',
  'resolverpt2 = cg.resolverpt2:main',
  'semalvopt2 = cg.semalvopt2:main',
`

---

### Pré-requisitos: ROS 2 (Jazzy) e Python 3 instalados em ambiente Linux/WSL.


## Como Rodar

### Parte 1: Navegação com Mapa Conhecido

1. **Terminal 1 (Simulador):**
   Inicie o jogo.
   \`\`\`
   ros2 run cg maze
   \`\`\`

2. **Terminal 2 (Solução):**
   Execute o script de resolução. Ele solicitará um novo mapa aleatório automaticamente.
   \`\`\`
   ros2 run cg resolver
   \`\`\`

---

### Parte 2: Mapeamento + Navegação

1. **Terminal 1 (Simulador):**
   Inicie o jogo.
   \`\`\`
   ros2 run cg maze
   \`\`\`

2. **Terminal 2 (Solução):**
   Execute o script de exploração.
   \`\`\`
   ros2 run cg resolverpt2
   ou
   ros2 run cg semalvopt2
   \`\`\`

---

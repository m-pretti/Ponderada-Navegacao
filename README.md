# Ponderada: Navegação Autônoma com ROS 2

Este projeto apresenta uma solução em **C++** para o desafio de navegação em labirintos do simulador **Culling Games**.
O sistema é capaz de:

* Planejar rotas em mapas conhecidos
* Explorar ambientes desconhecidos construindo mapas dinamicamente

---

## Estrutura do Projeto

O pacote `atividade` contém dois executáveis principais:

### **1. Etapa 01 — Solver (Mapa Completo)**

Recebe o mapa completo do servidor e calcula a rota ótima usando **BFS** (Busca em Largura).

### **2. Etapa 02 — Explorer (Mapeamento Dinâmico)**

Navega às cegas usando sensores, constrói o mapa com **DFS + Backtracking** e valida a solução executando a rota ótima ao final.

---

## Instalação e Compilação

Certifique-se de estar na raiz do workspace e ter o pacote `cg_interfaces` instalado.

### **1. Compile o projeto**

Se for a primeira vez que está compilando:

```bash
colcon build
```

Depois de buildar: 

```bash
colcon build --packages-select atividade
```

### **2. Carregue o ambiente**

```bash
source install/setup.bash
```

---

## Como Executar

Para qualquer etapa, **é necessário ter o Simulador rodando em um terminal separado**.

---

### **0. Iniciar o Simulador (Terminal 1)**

Mantenha este terminal aberto durante todo o processo.

```bash
source install/setup.bash
ros2 run cg maze
```

**Gerar novo labirinto aleatório (opcional):**

```bash
ros2 service call /reset cg_interfaces/srv/Reset "{is_random: true}"
```

---

### **1. Etapa 1 — Planejamento Global (Terminal 2)**

Nesta etapa, o robô **conhece** o mapa.
Ele calcula o caminho mais curto usando **BFS** e vai direto ao alvo.

```bash
source install/setup.bash
ros2 run atividade etapa01
```

**O que esperar:**

* Log: `[etapa01]: Caminho calculado: X passos.`
* O robô segue a rota ótima sem explorar.

---

### **2. Etapa 2 — Exploração e Mapeamento (Terminal 2)**

Nesta etapa, o robô **não conhece** o mapa.
Ele explora usando **DFS com backtracking**, desenha o mapa em tempo real e, ao final, valida a solução.

```bash
source install/setup.bash
ros2 run atividade etapa02
```

#### **Visualização no Terminal**

O robô desenha um mapa ASCII enquanto avança:

| Símbolo | Significado  |
| ------- | ------------ |
| `R`     | Robô         |
| `##`    | Parede       |
| `..`    | Desconhecido |
| `X`     | Alvo         |

#### **Comportamento esperado:**

* Exploração total do labirinto.
* Retorno ao ponto inicial ao concluir o mapeamento.
* Cálculo interno da rota ótima.

#### **Validação da Etapa 2 (Mesmo Terminal)**

```bash
ctrl + c
ros2 run atividade etapa01
```

* **Melhor caminho:** execução rápida da rota calculada para validar o mapa.

### Vídeo Explicativo


[Vídeo](https://drive.google.com/drive/folders/18ymObB7-tpWtJleKrv3zLws9moR35ceg?usp=drive_link)

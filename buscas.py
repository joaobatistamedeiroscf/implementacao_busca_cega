import sys
import time
import tracemalloc
import heapq
import itertools


# ============================================================
# NODE
# ============================================================

class Node:

    def __init__(self, state, parent, action, cost=0, depth=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost
        self.depth = depth


# ============================================================
# PILHA - usada pela busca em profundidade limitada
# ============================================================

class StackFrontier:

    def __init__(self):
        self.frontier = []

    def add(self, node):
        self.frontier.append(node)

    def contains_state(self, state):
        return any(node.state == state for node in self.frontier)

    def empty(self):
        return len(self.frontier) == 0

    def remove(self):

        if self.empty():
            raise Exception("empty frontier")

        node = self.frontier[-1]
        self.frontier = self.frontier[:-1]

        return node


# ============================================================
# FILA
# ============================================================

class QueueFrontier(StackFrontier):

    def remove(self):

        if self.empty():
            raise Exception("empty frontier")

        node = self.frontier[0]
        self.frontier = self.frontier[1:]

        return node


# ============================================================
# FILA DE PRIORIDADE - usada pela UCS
# ============================================================

class PriorityQueueFrontier:

    def __init__(self):

        self.frontier = []
        self.counter = itertools.count()

    def add(self, node):

        count = next(self.counter)

        heapq.heappush(
            self.frontier,
            (node.cost, count, node)
        )

    def empty(self):

        return len(self.frontier) == 0

    def remove(self):

        if self.empty():
            raise Exception("empty frontier")

        cost, count, node = heapq.heappop(self.frontier)

        return node


# ============================================================
# LABIRINTO
# ============================================================

class Maze:

    def __init__(self, filename):

        with open(filename, "r") as f:
            contents = f.read()

        if contents.count("A") != 1:
            raise Exception(
                "maze must have exactly one start point"
            )

        if contents.count("B") != 1:
            raise Exception(
                "maze must have exactly one goal"
            )

        contents = contents.splitlines()

        self.height = len(contents)
        self.width = max(len(line) for line in contents)

        self.walls = []
        self.costs = []

        for i in range(self.height):

            row = []
            cost_row = []

            for j in range(self.width):

                try:

                    character = contents[i][j]

                    if character == "A":

                        self.start = (i, j)
                        row.append(False)
                        cost_row.append(1)

                    elif character == "B":

                        self.goal = (i, j)
                        row.append(False)
                        cost_row.append(1)

                    elif character == " ":

                        row.append(False)
                        cost_row.append(1)

                    elif character.isdigit():

                        row.append(False)
                        cost_row.append(int(character))

                    else:

                        row.append(True)
                        cost_row.append(None)

                except IndexError:

                    row.append(False)
                    cost_row.append(1)

            self.walls.append(row)
            self.costs.append(cost_row)

        self.solution = None
        self.cost = None
        self.num_explored = 0
        self.explored = set()


    # ========================================================
    # VIZINHOS
    # ========================================================

    def neighbors(self, state):

        row, col = state

        candidates = [
            ("up", (row - 1, col)),
            ("down", (row + 1, col)),
            ("left", (row, col - 1)),
            ("right", (row, col + 1))
        ]

        result = []

        for action, (r, c) in candidates:

            if (
                0 <= r < self.height
                and
                0 <= c < self.width
                and
                not self.walls[r][c]
            ):

                result.append(
                    (
                        action,
                        (r, c),
                        self.costs[r][c]
                    )
                )

        return result


    # ========================================================
    # CONSTRÓI A SOLUÇÃO
    # ========================================================

    def build_solution(self, node):

        actions = []
        cells = []

        cost = node.cost

        while node.parent is not None:

            actions.append(node.action)
            cells.append(node.state)

            node = node.parent

        actions.reverse()
        cells.reverse()

        self.solution = (actions, cells)
        self.cost = cost


    # ========================================================
    # BUSCA DE CUSTO UNIFORME - UCS
    # ========================================================

    def solve_ucs(self):

        self.num_explored = 0
        self.explored = set()
        self.solution = None
        self.cost = None

        start = Node(
            state=self.start,
            parent=None,
            action=None,
            cost=0,
            depth=0
        )

        frontier = PriorityQueueFrontier()
        frontier.add(start)

        while True:

            if frontier.empty():
                raise Exception("no solution")

            node = frontier.remove()

            if node.state in self.explored:
                continue

            self.num_explored += 1

            if node.state == self.goal:

                self.build_solution(node)

                return

            self.explored.add(node.state)

            for action, state, step_cost in self.neighbors(
                node.state
            ):

                if state in self.explored:
                    continue

                child = Node(
                    state=state,
                    parent=node,
                    action=action,
                    cost=node.cost + step_cost,
                    depth=node.depth + 1
                )

                frontier.add(child)


    # ========================================================
    # BUSCA EM PROFUNDIDADE LIMITADA - DLS
    # ========================================================

    def solve_dls(self, limit):

        self.num_explored = 0
        self.explored = set()
        self.solution = None
        self.cost = None

        start = Node(
            state=self.start,
            parent=None,
            action=None,
            cost=0,
            depth=0
        )

        frontier = StackFrontier()
        frontier.add(start)

        cutoff_occurred = False

        while True:

            if frontier.empty():

                if cutoff_occurred:
                    return "CUTOFF"

                return "FAILURE"

            node = frontier.remove()

            self.num_explored += 1

            if node.state == self.goal:

                self.build_solution(node)

                return "FOUND"

            self.explored.add(node.state)

            if node.depth < limit:

                for action, state, step_cost in self.neighbors(
                    node.state
                ):

                    if (
                        not frontier.contains_state(state)
                        and
                        state not in self.explored
                    ):

                        child = Node(
                            state=state,
                            parent=node,
                            action=action,
                            cost=node.cost + step_cost,
                            depth=node.depth + 1
                        )

                        frontier.add(child)

            else:

                cutoff_occurred = True


    # ========================================================
    # APROFUNDAMENTO ITERATIVO - IDDFS
    # ========================================================

    def solve_iddfs(self, max_limit=None):

        total_explored = 0
        limit = 0

        while True:

            result = self.solve_dls(limit)

            total_explored += self.num_explored

            if result == "FOUND":

                self.num_explored = total_explored

                return limit

            if result == "FAILURE":

                raise Exception("no solution")

            if (
                max_limit is not None
                and
                limit >= max_limit
            ):

                self.num_explored = total_explored

                raise Exception(
                    f"no solution found up to limit {max_limit}"
                )

            limit += 1


    # ========================================================
    # IMPRIME O LABIRINTO
    # ========================================================

    def print(self):

        solution = (
            self.solution[1]
            if self.solution is not None
            else None
        )

        print()

        for i, row in enumerate(self.walls):

            line = []

            for j, col in enumerate(row):

                if col:

                    line.append("█")

                elif (i, j) == self.start:

                    line.append("A")

                elif (i, j) == self.goal:

                    line.append("B")

                elif (
                    solution is not None
                    and
                    (i, j) in solution
                ):

                    line.append("*")

                else:

                    line.append(" ")

            print("".join(line))

        print()


    # ========================================================
    # GERA IMAGEM DA SOLUÇÃO
    # ========================================================

    def output_image(
        self,
        filename,
        show_solution=True,
        show_explored=False
    ):

        try:

            from PIL import Image, ImageDraw

        except ImportError:

            print(
                "Pillow não está instalado. "
                "A imagem não será gerada."
            )

            return

        cell_size = 50
        cell_border = 2

        img = Image.new(
            "RGBA",
            (
                self.width * cell_size,
                self.height * cell_size
            ),
            "black"
        )

        draw = ImageDraw.Draw(img)

        solution = (
            self.solution[1]
            if self.solution is not None
            else None
        )

        for i, row in enumerate(self.walls):

            for j, col in enumerate(row):

                if col:

                    fill = (40, 40, 40)

                elif (i, j) == self.start:

                    fill = (255, 0, 0)

                elif (i, j) == self.goal:

                    fill = (0, 171, 28)

                elif (
                    solution is not None
                    and show_solution
                    and (i, j) in solution
                ):

                    fill = (220, 235, 113)

                elif (
                    show_explored
                    and
                    (i, j) in self.explored
                ):

                    fill = (212, 97, 85)

                else:

                    fill = (237, 240, 252)

                draw.rectangle(
                    [
                        (
                            j * cell_size + cell_border,
                            i * cell_size + cell_border
                        ),
                        (
                            (j + 1) * cell_size - cell_border,
                            (i + 1) * cell_size - cell_border
                        )
                    ],
                    fill=fill
                )

        img.save(filename)


# ============================================================
# EXECUTA A BUSCA E MEDE TEMPO E MEMÓRIA
# ============================================================

def executar_busca(maze, algoritmo, limite=None):

    tracemalloc.start()

    inicio = time.perf_counter()

    profundidade = None

    if algoritmo == "ucs":

        maze.solve_ucs()

        status = "FOUND"

    elif algoritmo == "dls":

        status = maze.solve_dls(limite)

    elif algoritmo == "iddfs":

        profundidade = maze.solve_iddfs()

        status = "FOUND"

    else:

        tracemalloc.stop()

        raise Exception("algoritmo inválido")

    fim = time.perf_counter()

    memoria_atual, memoria_pico = (
        tracemalloc.get_traced_memory()
    )

    tracemalloc.stop()

    tempo = fim - inicio

    memoria_mb = memoria_pico / (1024 * 1024)

    return (
        status,
        tempo,
        memoria_mb,
        profundidade
    )


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

if __name__ == "__main__":

    if len(sys.argv) < 3:

        sys.exit(
            "\n"
            "Uso:\n"
            "\n"
            "python maze_search.py labirinto.txt algoritmo\n"
            "\n"
            "Algoritmos:\n"
            "  ucs\n"
            "  dls L\n"
            "  iddfs\n"
            "\n"
            "Exemplos:\n"
            "python maze_search.py maze1.txt ucs\n"
            "python maze_search.py maze1.txt dls 50\n"
            "python maze_search.py maze1.txt iddfs\n"
        )

    arquivo = sys.argv[1]
    algoritmo = sys.argv[2].lower()

    limite = None

    if algoritmo == "dls":

        if len(sys.argv) < 4:

            sys.exit(
                "Para DLS informe o limite L.\n\n"
                "Exemplo:\n"
                "python maze_search.py maze1.txt dls 50"
            )

        limite = int(sys.argv[3])

    elif algoritmo not in ["ucs", "iddfs"]:

        sys.exit(
            "Algoritmo inválido.\n"
            "Use: ucs, dls ou iddfs."
        )

    maze = Maze(arquivo)

    print()
    print("=" * 60)
    print("LABIRINTO")
    print("=" * 60)

    print(f"Arquivo: {arquivo}")

    print("\nLabirinto:")
    maze.print()

    print(
        f"Executando {algoritmo.upper()}..."
    )

    try:

        (
            status,
            tempo,
            memoria_mb,
            profundidade
        ) = executar_busca(
            maze,
            algoritmo,
            limite
        )

    except Exception as erro:

        print()
        print("ERRO:", erro)

        sys.exit(1)

    print()
    print("=" * 60)
    print("RESULTADOS")
    print("=" * 60)

    print(f"Labirinto: {arquivo}")
    print(f"Algoritmo: {algoritmo.upper()}")

    if status == "FOUND":

        print("Status: solução encontrada")

    elif status == "CUTOFF":

        print(
            "Status: corte (CUTOFF) - "
            "nenhuma solução dentro do limite"
        )

    else:

        print("Status: sem solução")

    if algoritmo == "dls":

        print(
            f"Limite de profundidade: {limite}"
        )

    if algoritmo == "iddfs":

        print(
            f"Profundidade da solução: {profundidade}"
        )

    print(
        f"Estados explorados: "
        f"{maze.num_explored}"
    )

    print(
        f"Custo da solução: "
        f"{maze.cost}"
    )

    print(
        f"Tempo: "
        f"{tempo:.6f} segundos"
    )

    print(
        f"Memória de pico: "
        f"{memoria_mb:.2f} MB"
    )

    print("=" * 60)

    if maze.solution is not None:

        print("\nSolução:")
        maze.print()

        try:

            maze.output_image(
                "maze.png",
                show_solution=True,
                show_explored=True
            )

            print("Imagem salva em: maze.png")

        except Exception as erro:

            print(
                f"Erro ao gerar imagem: {erro}"
            )
import math
from Space import *
from Constants import *


def DFS(g: Graph, sc: pygame.Surface):
    print('Implement DFS algorithm')

    open_set = [g.start.value]
    closed_set = []
    father = [-1] * g.get_len()

    # TODO: Implement DFS algorithm using open_set, closed_set, and father
    path = []
    visited = {}
    st = []
    st.append(g.start)
    prevNode = g.start
    while len(st) > 0:
        currentNode = st.pop()
        prevNode.set_color(blue)
        if currentNode in visited:
            continue

        currentNode.set_color(yellow)
        if currentNode not in visited.keys():
            visited[currentNode] = prevNode
            path.append(currentNode)
        if currentNode == g.goal:
            g.goal.set_color(purple)
            prevNode = None
            while currentNode != g.start:
                prevNode = currentNode
                currentNode = visited[currentNode]
                currentNode.set_color(grey)
                prevNode.draw_line(sc, currentNode)
                g.draw(sc)
            break
        for adjacent in g.get_neighbors(currentNode):
            if adjacent not in visited.keys():
                adjacent.set_color(red)
                st.append(adjacent)

        prevNode = currentNode
        g.draw(sc)


def BFS(g: Graph, sc: pygame.Surface):
    print('Implement BFS algorithm')

    open_set = [g.start.value]
    closed_set = []
    father = [-1] * g.get_len()

    # TODO: Implement BFS algorithm using open_set, closed_set, and father
    path = []
    visited = {}
    queue = []

    queue.append(g.start)
    visited[g.start] = None

    while len(queue):
        cur_node = queue.pop(0)
        cur_node.set_color(yellow)

        for item in g.get_neighbors(cur_node):
            if item not in visited.keys():
                queue.append(item)
                visited[item] = cur_node
                item.set_color(red)
                if item == g.goal:
                    g.goal.set_color(purple)
                    prevNode = None
                    while item != g.start:
                        prevNode = item
                        item = visited[item]
                        item.set_color(grey)
                        prevNode.draw_line(sc, item)
                        g.draw(sc)
                    return

        g.draw(sc)
        cur_node.set_color(blue)


def UCS(g: Graph, sc: pygame.Surface):
    print('Implement UCS algorithm')

    open_set = {}
    open_set[g.start.value] = 0
    closed_set: list[int] = []
    father = [-1] * g.get_len()
    cost = [100_000] * g.get_len()
    cost[g.start.value] = 0

    # TODO: Implement UCS algorithm using open_set, closed_set, and father
    while len(open_set) > 0:
        min_node = min(open_set, key=open_set.get)
        open_set.pop(min_node)
        closed_set.append(min_node)
        node = g.get_node(min_node)
        node.set_color(yellow)
        if min_node == g.goal.value:
            g.goal.set_color(purple)
            prevNode = None
            while node != g.start:
                prevNode = node
                node = g.get_node(father[node.value])
                node.set_color(grey)
                prevNode.draw_line(sc, node)
                g.draw(sc)
            break
        for item in g.get_neighbors(g.get_node(min_node)):
            if item.value not in closed_set:
                item.set_color(red)
                if item.value not in open_set.keys():
                    open_set[item.value] = cost[min_node] + item.value
                    cost[item.value] = cost[min_node] + item.value
                    father[item.value] = min_node
                else:
                    if cost[min_node] + item.value < cost[item.value]:
                        open_set[item.value] = cost[min_node] + item.value
                        cost[item.value] = cost[min_node] + item.value
                        father[item.value] = min_node
        g.draw(sc)
        node.set_color(blue)


def heuristic(current: Node, goal: Node):
    return math.sqrt((current.value - goal.value) ** 2 + abs(current.value - goal.value) ** 2)


def AStar(g: Graph, sc: pygame.Surface):
    print('Implement A* algorithm')

    open_set = {}
    open_set[g.start.value] = 0
    closed_set: list[int] = []
    father = [-1] * g.get_len()
    cost = [100_000] * g.get_len()
    cost[g.start.value] = 0

    # TODO: Implement A* algorithm using open_set, closed_set, and father
    while len(open_set) > 0:
        min_node = min(open_set, key=open_set.get)
        open_set.pop(min_node)
        closed_set.append(min_node)
        node = g.get_node(min_node)
        node.set_color(yellow)
        if min_node == g.goal.value:
            g.goal.set_color(purple)
            prevNode = None
            while node != g.start:
                prevNode = node
                node = g.get_node(father[node.value])
                node.set_color(grey)
                prevNode.draw_line(sc, node)
                g.draw(sc)
            break
        if node not in closed_set:
            closed_set.append(node)

        for i in g.get_neighbors(node):
            if i not in closed_set:
                if g.is_goal(i) == False:
                    i.set_color(red)
                open_set[i.value] = cost[i.value]
        g.draw(sc)
        node.set_color(blue)
        for i in g.get_neighbors(node):
            if i not in closed_set:
                cost[i.value] = cost[node.value]
                open_set[i.value] = cost[i.value] + heuristic(i, g.goal)
                father[i.value] = node.value

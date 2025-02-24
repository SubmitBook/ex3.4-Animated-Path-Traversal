import networkx as nx
import matplotlib.pyplot as plt
import heapq
import time


def dijkstra(graph, start):
    """执行 Dijkstra 算法，并返回最短路径树和路径成本"""
    pq = [(0, start)]
    distances = {node: float('inf') for node in graph.nodes}
    distances[start] = 0
    previous_nodes = {node: None for node in graph.nodes}

    while pq:
        current_distance, current_node = heapq.heappop(pq)

        for neighbor in graph.neighbors(current_node):
            weight = graph[current_node][neighbor]['weight']
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))

    return previous_nodes, distances


def shortest_path(previous_nodes, start, end):
    """从 previous_nodes 构造最短路径"""
    path = []
    while end is not None:
        path.insert(0, end)
        end = previous_nodes[end]
    return path if path[0] == start else []


def animate_shortest_path(graph, path):
    """绘制 Dijkstra 最短路径的动画"""
    pos = nx.spring_layout(graph)
    plt.figure(figsize=(8, 6))

    for i in range(len(path) - 1):
        plt.clf()
        nx.draw(graph, pos, with_labels=True, node_color='lightgray', edge_color='gray', node_size=700, font_size=12)

        # 绘制已遍历的路径
        nx.draw_networkx_edges(graph, pos, edgelist=[(path[j], path[j + 1]) for j in range(i + 1)], edge_color='red',
                               width=2)
        nx.draw_networkx_nodes(graph, pos, nodelist=path[:i + 2], node_color='lightblue', node_size=700)

        plt.pause(0.5)

    plt.show()


# 构建加权图
G = nx.Graph()
edges = [
    (1, 2, 4), (1, 3, 1), (2, 3, 2), (2, 4, 5), (3, 4, 8), (3, 5, 10),
    (4, 5, 2), (4, 6, 6), (5, 6, 3)
]
G.add_weighted_edges_from(edges)

# 运行 Dijkstra 算法
start_node, end_node = 1, 6
prev_nodes, _ = dijkstra(G, start_node)
path = shortest_path(prev_nodes, start_node, end_node)

# 动画展示
animate_shortest_path(G, path)

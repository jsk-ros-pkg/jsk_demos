#!/usr/bin/env python3

import sys

from bosdyn.api.graph_nav import map_pb2

def load_graph_file(filename):
    with open(filename, 'rb') as graph_file:
        data = graph_file.read()
        graph = map_pb2.Graph()
        graph.ParseFromString(data)
        return graph

def main():
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        print('Usage: print_node_list.py <graph file>')
        return
    graph = load_graph_file(filename)

    for index, waypoint in enumerate(graph.waypoints):
        print('{} th: {}'.format(index,waypoint.id))

if __name__=='__main__':
    main()

#!/usr/bin/env python3

import rospy
from bosdyn.api.graph_nav import map_pb2
from spot_person_leader.srv import GetStairRanges, GetStairRangesResponse

def load_graph_file(filename):
    with open(filename, 'rb') as graph_file:
        data = graph_file.read()
        graph = map_pb2.Graph()
        graph.ParseFromString(data)
        return graph

def calc_total_cost(graph):
    total_cost = 0.0
    for edge in graph.edges:
        total_cost += edge.annotations.cost.value
    return total_cost

def get_stair_ranges(graph):
    result = []
    start_id = -1
    end_id = -1
    for index, edge in enumerate(graph.edges):
        if start_id == -1 and edge.annotations.stairs.state == map_pb2.ANNOTATION_STATE_SET:
            start_id = index
        elif start_id != -1 and edge.annotations.stairs.state == map_pb2.ANNOTATION_STATE_NONE:
            end_id = index
            result.append((start_id,end_id))
            start_id = -1
            end_id = -1
    return result


def handler(req):
    graph = load_graph_file(req.upload_filepath+'/graph')
    stair_ranges = get_stair_ranges(graph)
    return GetStairRangesResponse(result='{}'.format(stair_ranges))

def main():
    rospy.init_node('get_stair_ranges')
    rospy.Service('~/get_stair_ranges', GetStairRanges, handler)
    rospy.loginfo('Initialized.')
    rospy.spin()

def test():
    filename = '/home/sktometometo/Desktop/eng2_73b2_to_81c1_night.walk/graph'
    graph = load_graph_file(filename)
    stair_ranges = get_stair_ranges(graph)
    cost = calc_total_cost(graph)
    print('cost: {}'.format(cost))
    print('stair_ranges: {}'.format(stair_ranges))

if __name__=='__main__':
    main()
    #test()

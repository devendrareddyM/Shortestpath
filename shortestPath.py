from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp



def create_data_model():

    data = {}

    data['distance_matrix'] = [
                                [0,66,28,60,34,34,3,108],
                                [22,0,12,91,121,111,71,0],
                                [39,113,0,130,35,40,0,0],
                                [63,21,57,0,83,0,0,0] ,
                                [9,50,60,0,0,0,0,0],
                                [27,81,0,0,0,0,0,0],
                                [90,0,0,0,0,0,0,0]
                                ]
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

'''Here in this list of index positions represnts names of the cities in the output
    0.Mysore
    1.Mandya
    2.Chennapatna
    3.Nanjangud
    4.Bandipur
    5.Nagarhole
    6.Somanathpur
'''

def print_solution(manager, routing, assignment):
    index = routing.Start(0)
    plan_output = ''
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print("The shortest path is:\n",plan_output)


def main():
    data = create_data_model()


    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])


    routing = pywrapcp.RoutingModel(manager)


    def distance_callback(from_index, to_index):

        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    assignment = routing.SolveWithParameters(search_parameters)

    if assignment:
        print_solution(manager, routing, assignment)


if __name__ == '__main__':
    main()

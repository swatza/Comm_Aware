
import math

#RETURNS THE INDEXES FOR THE BEST POSITION
def find_optimal_2_node_location(MapA,MapB,x_length,y_length):
    #initialize output values
    storedCost = 9999 #high value
    storedLocationIndex = [0,0]
    #TODO! Loop through the maps
    for a in range(0,x_length):
        for b in range(0,y_length):
            #Calculate the cost
            results = SimpleCostFunction(MapA[a,b],MapB[a,b])
            if storedCost > results:
                #store the new min cost
                storedCost = results
                storedLocationIndex = [a,b]
            elif storedCost == results:
                print "Multiple best locations found: Keeping previous results"
            #end check
    #end for loops
    return storedCost, storedLocationIndex

def SimpleCostFunction(valueA,valueB):
    cost = valueA + valueB + math.fabs(valueA - valueB)
    return cost


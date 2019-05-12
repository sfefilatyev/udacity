#!/usr/bin/env python
# coding: utf-8

# In[1]:


# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space


grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid, init, goal, cost):
    # Implemented thorugh Breadth-first search
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1
    x = init[0]
    y = init[1]
    g = 0
    # Expansion list of 
    open = [[g, x, y]]

    found = False  # Set when search is complete
    resign = False  # Set when search cannot be completed

    while found is False and resign is False:
        # Check if we still have have cells to expand into
        if len(open) == 0:
            resign = True
            print("Failed to find goal")
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[1]
            y = next[2]
            g = next[0]

        if x == goal[0] and y == goal[1]:
            found = True
            print("Search successful, g=" + str(g))
        else:
            # Expand winning element
            for i in range(len(delta)):
                x2 = x + delta[i][0]
                y2 = y + delta[i][1]
                if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                    if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                        g2 = g + cost
                        open.append([g2, x2, y2])
                        closed[x2][y2] = 1
                    
                
search(grid,init,goal,cost)
    
    
#    return path


# In[ ]:


##### Do Not Modify ######

#import grader

#try:
#    response = grader.run_grader(search)
#    print(response)    
    
#except Exception as err:
#    print(str(err))


# In[ ]:


##### SOLUTION: Run this cell to watch the solution video ######
#from IPython.display import HTML
#HTML('<iframe width="560" height="315" src="https://www.youtube.com/embed/cl8Kdkr4Gbg" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>')
#

-- Inf2d Assignment 1 2019-2020
-- Matriculation number: s1846118
-- {-# OPTIONS -Wall #-}


module Inf2d1 where

import Data.List (sortBy, elemIndices, elemIndex, union)
import ConnectFourWithTwist



{- NOTES:

-- DO NOT CHANGE THE NAMES OR TYPE DEFINITIONS OF THE FUNCTIONS!
You can write new auxillary functions, but don't change the names or type definitions
of the functions which you are asked to implement.

-- Comment your code.

-- You should submit this file when you have finished the assignment.

-- The deadline is the  10th March 2020 at 3pm.

-- See the assignment sheet and document files for more information on the predefined game functions.

-- See the README for description of a user interface to test your code.

-- See www.haskell.org for haskell revision.

-- Useful haskell topics, which you should revise:
-- Recursion
-- The Maybe monad
-- Higher-order functions
-- List processing functions: map, fold, filter, sortBy ...

-- See Russell and Norvig Chapters 3 for search algorithms,
-- and Chapter 5 for game search algorithms.

-}

-- Section 1: Uniform Search



-- The Node type defines the position of the agent on the graph.
-- The Branch type synonym defines the branch of search through the graph.
type Node = Int
type Branch = [Node]
type Graph= [Node]

numNodes::Int
numNodes = 4  {- Shows how many nodes are in the graph, since their are currently four nodes in the graph
				 our graph array will have 16 entries (as we are using an adjacency matrix).
			  -}


-- The next function should return all the possible continuations of input search branch through the graph.
-- Your function should return an empty list if the input search branch is empty.
-- This implementation of next function does not backtrace branches.

-- This function takes the larger graph and a node and returns the row specific in the adjacency matrix to this node
subGraph :: Node -> Graph -> Graph 
subGraph node graph =  [x | x <- drop (node*numNodes) (take (node*numNodes + numNodes) (graph))]

--Makes a list of tuple pairs which represent (node, neighbour). 
findNeighbours :: Node -> Graph -> [(Node, Node)]
findNeighbours node graph = zip (replicate 4 node) [x | (x,y) <- zip [0..(length (subGraph node graph))] (subGraph node graph), y /= 0]

--This takes the tuple pairs which find neighbours returns to a list. 
pairToList :: (a, a) -> [a]
pairToList (x,y) = [x,y]

--This refines the subgraph list down to the list of neighbours
almostNext :: Node -> Graph -> [Branch]
almostNext node graph = [pairToList (y,x) | (x,y) <- (findNeighbours node graph)]

--This applies the 'almostNext' function to every node in the branch.
next::Branch -> Graph ->  [Branch]
next branch g = concat [almostNext x g | x <- branch]
       

-- |The checkArrival function should return true if the current location of the robot is the destination, and false otherwise.
checkArrival::Node -> Node -> Bool
checkArrival destination curNode = destination == curNode


explored::Node-> [Node] -> Bool
explored point exploredList = elem point exploredList

-- Section 3 Uniformed Search
-- | Breadth-First Search
-- The breadthFirstSearch function should use the next function to expand a node,
-- and the checkArrival function to check whether a node is a destination position.
-- The function should search nodes using a breadth first search order.

-- Starting node is 0 and the end node is 4 as defined in 'GraphSearch.hs'
-- What we want to do is call 'next' on our start node. This gives a list of branches we shall check if the end of each
-- of these branches are the destination. If not we reccursivley call 'next' on the end of each branch untill goal node 
-- is found.

-- We pass 'next' function in the argumentent so marker can pass their own if yours is incorrect...
-- When calling 'bfs' reccursivley, we will continually call the next function on different nodes within the search agenda 
-- at each stage.
elemOfListOfLists :: [Branch] -> [Node]
elemOfListOfLists [b] = b
elemOfListOfLists (b:bs) = b `union` elemOfListOfLists bs

goalInBranches :: [Branch] -> Node -> Bool --Please excuse the shit name. This function checks if the goal node is in the branches we have searched.
goalInBranches [] gNode = False --Should probably return error but for makes it easier if we return false.
goalInBranches [x] gNode = gNode == last x
goalInBranches (b:bs) gNode
                          | checkArrival gNode (last b) = True
                          | otherwise = goalInBranches bs gNode

returnSolution :: [Branch] -> Node -> Maybe Branch --This function goes through the list of branches and returns the first one which contains our goal node.
returnSolution [] goal = Nothing
returnSolution (b:bs) goal 
                           | goal `elem` b = Just b
                           | otherwise = returnSolution bs goal

upDateBranches :: Graph -> [Branch] -> [Node] -> [Branch] --This is going to update our branches. Similar to the 'next' function but it adds to the whole branch
upDateBranches [] _ _ = error ("Empty graph?")
upDateBranches g [b] exploredList = [a `union` b | a <- (next [head b] g), not((head a) `elem` exploredList)] --Adds neighbours which are not in 'explored' list.
upDateBranches g (b:bs) exploredList = [a `union` b | a <- (next [head b] g), not((head a) `elem` exploredList)] ++ --We must now apply this reccursivley while adding to the 'exploredList' at every call.
                                       upDateBranches g bs (exploredList ++ (elemOfListOfLists [a `union` b | a <- (next [head b] g), not((head a) `elem` exploredList)])) --This confusing list comprehension repeates the code at the begining but I dont yet know how to make this look neater
 
breadthFirstSearch::Graph -> Node->(Branch ->Graph -> [Branch])->[Branch]->[Node]->Maybe Branch
breadthFirstSearch [] _ _ _ _ = Nothing
breadthFirstSearch g destination next [] exploredList = Nothing --Covers the case where we have no branches left to search.
breadthFirstSearch g destination next [x] exploredList --We must cover this unique starting case. This allows us to enter our starting node only in the branches args without getting an error.
                                                     | checkArrival destination (last x) = Just x
                                                     | otherwise = breadthFirstSearch g destination next (next x g) (exploredList `union` x) --Now in my code below I can cover only the cases where 'branches' has more than one path :)
breadthFirstSearch g destination next branches exploredList
                                                          | goalInBranches branches destination = returnSolution branches destination
                                                          | otherwise = breadthFirstSearch g destination next (upDateBranches g branches exploredList) ((elemOfListOfLists branches) `union` exploredList)

-- | Depth-Limited Search
-- The depthLimitedSearch function is similiar to the depthFirstSearch function,
-- except its search is limited to a pre-determined depth, d, in the search tree.
depthLimitedSearch::Graph ->Node->(Branch ->Graph-> [Branch])->[Branch]-> Int->[Node]-> Maybe Branch
depthLimitedSearch g destination next branches  d exploredList = undefined



-- | Section 4: Informed search


-- | AStar Helper Functions

-- | The cost function calculates the current cost of a trace. The cost for a single transition is given in the adjacency matrix.
-- The cost of a whole trace is the sum of all relevant transition costs.
cost :: Graph ->Branch  -> Int
cost gr branch = undefined


    
-- | The getHr function reads the heuristic for a node from a given heuristic table.
-- The heuristic table gives the heuristic (in this case straight line distance) and has one entry per node. It is ordered by node (e.g. the heuristic for node 0 can be found at index 0 ..)  
getHr:: [Int]->Node->Int
getHr hrTable node = undefined  


-- | A* Search
-- The aStarSearch function uses the checkArrival function to check whether a node is a destination position,
---- and a combination of the cost and heuristic functions to determine the order in which nodes are searched.
---- Nodes with a lower heuristic value should be searched before nodes with a higher heuristic value.

aStarSearch::Graph->Node->(Branch->Graph -> [Branch])->([Int]->Node->Int)->[Int]->(Graph->Branch->Int)->[Branch]-> [Node]-> Maybe Branch
aStarSearch g destination next getHr hrTable cost branches exploredList =undefined

-- | Section 5: Games
-- See ConnectFourWithTwist.hs for more detail on  functions that might be helpful for your implementation. 



-- | Section 5.1 Connect Four with a Twist

 

-- The function determines the score of a terminal state, assigning it a value of +1, -1 or 0:
eval :: Game -> Int
eval game = undefined

-- | The alphabeta function should return the minimax value using alphabeta pruning.
-- The eval function should be used to get the value of a terminal state. 
alphabeta:: Role -> Game -> Int
alphabeta  player game = undefined


-- | OPTIONAL!
-- You can try implementing this as a test for yourself or if you find alphabeta pruning too hard.
-- If you implement minimax instead of alphabeta, the maximum points you can get is 10% instead of 15%.
-- Note, we will only grade this function IF YOUR ALPHABETA FUNCTION IS EMPTY.
-- The minimax function should return the minimax value of the state (without alphabeta pruning).
-- The eval function should be used to get the value of a terminal state.
minimax:: Role -> Game -> Int
minimax player game=undefined
{- Auxiliary Functions
-- Include any auxiliary functions you need for your algorithms below.
-- For each function, state its purpose and comment adequately.
-- Functions which increase the complexity of the algorithm will not get additional scores
-}

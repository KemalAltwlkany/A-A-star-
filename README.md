# Astar


----------------------------------------ALGORITHM PROPERTIES--------------------------------
This algorithm runs on a map of dimensions m x n (where both m and n are positive integers). Since this is a 2D algorithm implementation, the map can be considered as a matrix (or grid).
The map/matrix should only be consisted of 0's and 1's. If a matrix element is equal to 0 that means that it is traversable, similarly, if a matrix element is equal to 1 that means that it is blocked (non-traversable).
This implementation allows the pathfinder to move towards both horizontal and vertical, but also diagonal neighbours (provided they are traversable).
This means that the neighbours of one matrix element, are:
	1 2 3
	4 x 5
	6 7 8
In this implementation matrix elements are called nodes.
Diagonal movements cost more than adjacent movements (by a factor of sqrt(2)).
The algorithm by default calculates f values by the following formula;

	fvalue=gvalue+heuristic

where: - gvalue is the Euclidian distance between two nodes (since they are matrix elements they have 2 coordinates and can be represented in a plain)
       - heuristic is computed by the following formula  { 1.4*std::sqrt( (gx-x)*(gx-x) + (gy-y)*(gy-y) ) }. (gx,gy are the goal node coordinates, x and y are the current node coordinates). In other words, it is the Euclidian distance between the goal node and given node but scaled/weighted by a factor of 1.4.

In order to change how the heuristic is computed, it is enough to just modify the method heuristic. The code already contains the octile distances, which can be activated by simply uncommenting them in method heuristic.


-------------------------------------------HOW-TO-USE----------------------------------------

This contains basic information on how to use the program.
If you only want to use the A* algorithm, then:

1.) In function main, declare an instance of class mapp.
For example;
mapp RandomName;

2.) Call method Test_mapp_functionality().
This method will allow you to generate a mapp through a menu. To do so, just follow the instructions which are provided through outputs (printed on screen). By following the simple instructions, a mapp will be generated.

3.) That's it. A* will launch automatically and notify you about the results once it finishes.

--------------------------------MODIFICATIONS-----------------------------------------------

1.) If you wish to change the way a map is generated the only method that needs to be modified is Generate_mapp().
Lets say you would like to run A* on many maps, one way of doing so would be as provided:
Create a for-loop in which you shall generate an instance of class mapp through every iteration.

for(int i=0; i<number_of_maps; i++){
	mapp RandomName;
	RandomName.Generate_mapp();
	RandomName.Astar();
}

And ofcourse, modify the Generate_mapp() function to your needs. Afterwards call A*.

-IMPORTANT - if you modify method Generate_mapp() remeber to never leave the 6 attributes of mapp uninitialized!!!! (start node, goal node, m, n, rows, columns and matrix should always be initialized).
start node - the coordinates of the start node.
goal node - the coordinates of the goal node.
rows - the number of rows of the given map.
columns - the number of columns of the given map.
m=rows-1 (just initialize it this way)
n=columns-1
matrix - this would actually be the "map". matrix should be a matrix consisted of 0's and 1's only. 0 represents a TRAVERSABLE node, while 1 represents a BLOCKED node.

2.) If you wish to measure the runtime of the algorithm, it is enough to declare objects just as done in the Test_mapp_functionality().
Declare the first clock exactly before you call A*, and the second clock exactly after you call A* and just subtract them.
Example;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        (*this).Astar();
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    double dif = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
    std::cout << "\nTime elapsed: " << dif;

you can change the parameter miliseconds to nano, micro and to just "seconds" depending on the desired format.

3.) If you wish to plot these results in MATLAB and generate an image as provided in the respository just change the path in the functions Pass_to_MATLAB().
Remember to call this method only after A* has terminated.
In order to now get the results to MATLAB use the script provided in the repository (since the method Pass_to_MATLAB() only generates a txt file, which MATLAB loads data from afterwards).
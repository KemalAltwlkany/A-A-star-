/* A* - a path finding algorithm
    Kemal Altwlkany, May 2018.
    */

#include <iostream>
#include <vector>
#include <cmath>
#include <iterator>
#include <algorithm>
#include <utility>
#include <list>
#include <cstdlib>
#include <chrono>
#include <fstream>

//A class which contains basic information about a node. It's coordinates, it's parents' coordinates and its g and f values.
class node{
    friend class mapp;
    int x,y; //coordinates of the given node
    double f_value, g_value; //f values and g values of the given node, respectively
    bool g_inf; //true iff the node has an infinite g_value
    bool null_parent; //true iff the node has no parent (null_parent)
    int x_p, y_p; //parent coordinates of the given node
public:
        node() { x=0; y=0; f_value=0; g_value=0; x_p=0; y_p=0; g_inf=true; null_parent=true; }

        node(int a, int b) { x=a; y=b; f_value=0; g_value=0; x_p=0; y_p=0; g_inf=true; null_parent=true; }

        double heuristic(int gx, int gy);
        friend bool Compare(node s1, node s2);
        friend double c_value(node s1, node s2);
        friend bool operator==(node s1, node s2);
        void Print_node_info();
};

//A help function that prints out all relevant information of a given node. Used for testing only.
void node::Print_node_info(){
    std::cout << "\n        Node coordinates are == (" << x << "," << y << "). ";
    std::cout << "\n        Parent node coordinates are == (" << x_p << "," << y_p << "). ";
    std::cout << "\n        Current f_value is == " << f_value;
    std::cout << "\n        Current g_value is == " << g_value;
    std::cout << "\n        Boolean g_inf is == " << g_inf;
    std::cout << "\n        Boolean null_parent is == " << null_parent << std::endl;
}


//computes and returns the heuristic for the given instance of class node, with respect to the goal node
double node::heuristic(int gx, int gy){
   //straight line distance scaled by a factor of 1.4
   return 1.4*std::sqrt( (gx-x)*(gx-x) + (gy-y)*(gy-y) );
   /* Uncomment the remaining lines of this function and comment the line above in order to use the octile distances as
   given heuristic instead of straight-line distance.
   int dx(std::abs(x-gx));
   int dy(std::abs(y-gy));
   return (std::sqrt(2)*std::min(dx,dy)+std::max(dx,dy)-std::min(dx,dy))*1.4;
  */
}

//computes the c value between two given nodes, which is equal to the distance between the two in a straight line
double c_value(node s1, node s2){
    return std::sqrt( (s1.x-s2.x)*(s1.x-s2.x) + (s1.y-s2.y)*(s1.y-s2.y) );
}

//compares the f_values of two nodes (f_value is they open_list sorting criteria (key)
bool Compare(node s1, node s2){
    if( s2.g_inf ) return true;
    if( s1.g_inf && !s2.g_inf) return false;
    return ( (s1.f_value - s2.f_value )<0 );
}

// Returns boolean depending on whether two input argument nodes are equal. Since we use one map only, it is enough to compare their coordinates.
bool operator==(node s1, node s2){
    return ( (s1.x==s2.x) && (s1.y==s2.y) );
}

//core class of this program. Allows operations such as generating a map, printing the map as a matrix of integers.
// contains all sub-functions required to run algorithms such as the A-star.
class mapp{
    std::vector<std::vector<int> > matrix; // the "map"
    std::list<node> open_list;
    std::list<node> closed_list;
    std::list<node> neighbours;
    int m,n; //m is the number of rows, n is the number of columns both decremented by 1. A redundant info kept for speed-up.
    int rows,columns; // the actual number of columns and rows
    node start, goal;
public:
        void Generate_mapp();
        void Print_mapp();
        void Test_mapp_functionality();
        void Astar();
        void Update_neighbours(node s);
        void Filter_obstacles();
        bool In_closedlist(node s);
        bool In_openlist(node s);
        void Update_vertex(node s, node sp);
        void Erase_from_openlist(node s);
        void Path_extraction();
        void Pass_to_MATLAB();
};
/*This function is optional and not required in order to run A*. It's purpose is to pass the results of A*
to MATLAB in order to plot them there or use them for further needs.
This function only creates a .txt file which contains the matrix data. A MATLAB script is used afterwards to extract the
data from the txt file and plot it. */
void mapp::Pass_to_MATLAB(){
    //enter desired file path in line bellow
    std::ofstream output("E:\\Example\\Example_folder\\matrixdata.txt");
    if (!output){
        std::cout << "Error while creating the matrixdata.txt file!" << std::endl;
        return;
    }
    for(int i=0; i<rows; i++){
        for(int j=0; j<rows; j++){
            if (matrix[i][j]==1) matrix[i][j]=50;
            output << matrix[i][j] << " ";
        }
        output << std::endl;
    }
}


//Used to extract the path. Begins from the goal node and follows the parents until the start node has been reached.
void mapp::Path_extraction(){
    int x,y;
    x=goal.x_p;
    y=goal.y_p;
    matrix[goal.x][goal.y]=100;
    for(std::list<node>::reverse_iterator it=closed_list.rbegin(); it!=closed_list.rend(); it++){
        if( x==(*it).x && y==(*it).y ){
            matrix[x][y]=100;
            x=(*it).x_p;
            y=(*it).y_p;
            if( x==start.x && y==start.y ){
                matrix[x][y]=100;
                return;
            }
        }
    }
    return;
}

void mapp::Erase_from_openlist(node s){
    for(std::list<node>::iterator it=open_list.begin(); it!=open_list.end(); it++){
        if( (*it)==s ){
            open_list.erase(it);
            break;
        }
    }
}

void mapp::Update_vertex(node s, node sp){
    //check whether the g_value of s + c_value(s,sp) < g_value(sp)
    if( s.g_inf ){ return; }
    if( sp.g_inf ){
        sp.g_inf=false;
        sp.null_parent=false;
        sp.g_value=s.g_value+c_value(s,sp);
        sp.x_p=s.x;
        sp.y_p=s.y;
        sp.f_value=sp.g_value+sp.heuristic(goal.x, goal.y);
        (*this).Erase_from_openlist(sp);
        open_list.insert(open_list.end(), sp);
        return;
    }
    if( ( ( s.g_value + c_value(s,sp) ) - sp.g_value ) < 0 ){
        sp.g_inf=false;
        sp.null_parent=false;
        sp.g_value=s.g_value+c_value(s,sp);
        sp.x_p=s.x;
        sp.y_p=s.y;
        sp.f_value=sp.g_value+sp.heuristic(goal.x, goal.y);
        (*this).Erase_from_openlist(sp);
        open_list.insert(open_list.end(), sp);
        return;
    }
}

bool mapp::In_closedlist(node s){
    std::list<node>::iterator it(closed_list.begin());
    while( it!=closed_list.end() ){
        if( (*it)==s ) return true;
        ++it;
    }
    return false;
}

bool mapp::In_openlist(node s){
    std::list<node>::iterator it(open_list.begin());
    while( it!=open_list.end() ){
        if( (*it)==s ) return true;
        ++it;
    }
    return false;
}


void mapp::Filter_obstacles(){
    std::list<node> pom;
    for(std::list<node>::iterator it=neighbours.begin(); it!=neighbours.end(); it++){
        if (! matrix[(*it).x][(*it).y] ) pom.insert(pom.begin(),(*it));
    }
    neighbours=pom;
}


/* The following function updates the list -neighbours-, depending on where the current node is. The function is long
and appears not optimized, but on the contrary it is much faster than resolving it with 9 if statements which would've been
put in a nested 3x3 for loop */
void mapp::Update_neighbours(node s){
        //node in corner (0,0)
        if( s.x==0 && s.y==0 ){
            neighbours.insert( neighbours.end(),node(0,1) );
            neighbours.insert( neighbours.end(),node(1,1) );
            neighbours.insert( neighbours.end(),node(1,0) );
            return;
        }
        //node in corner (0,columns-1)
        if( s.x==0 && s.y==n ){
            neighbours.insert( neighbours.end(),node(0,n-1) );
            neighbours.insert( neighbours.end(),node(1,n-1) );
            neighbours.insert( neighbours.end(),node(1,n) );
            return;
        }
        //node in corner (rows-1, 0)
        if( s.x==m && s.y==0 ){
            neighbours.insert( neighbours.end(),node(m-1,0) );
            neighbours.insert( neighbours.end(),node(m-1,1) );
            neighbours.insert( neighbours.end(),node(m,1) );
            return;
        }
        //node in corner (rows-1, columns-1)
        if ( s.x==m && s.y==n ){
            neighbours.insert( neighbours.end(),node(m-1,n) );
            neighbours.insert( neighbours.end(),node(m-1,n-1) );
            neighbours.insert( neighbours.end(),node(m,n-1) );
            return;
        }
        //node in first row
        if (s.x==0){
            neighbours.insert( neighbours.end(),node(0,s.y-1) );
            neighbours.insert( neighbours.end(),node(0,s.y+1) );
            neighbours.insert( neighbours.end(),node(1,s.y-1) );
            neighbours.insert( neighbours.end(),node(1,s.y) );
            neighbours.insert( neighbours.end(),node(1,s.y+1) );
            return;
        }
        //node in first column
        if ( s.y==0 ){
            neighbours.insert( neighbours.end(),node(s.x-1,0) );
            neighbours.insert( neighbours.end(),node(s.x+1,0) );
            neighbours.insert( neighbours.end(),node(s.x-1,1) );
            neighbours.insert( neighbours.end(),node(s.x,1) );
            neighbours.insert( neighbours.end(),node(s.x+1,1) );
            return;
        }
        //node in last row
        if (s.x==m){
            neighbours.insert( neighbours.end(),node(m,s.y-1) );
            neighbours.insert( neighbours.end(),node(m,s.y+1) );
            neighbours.insert( neighbours.end(),node(m-1,s.y-1) );
            neighbours.insert( neighbours.end(),node(m-1,s.y) );
            neighbours.insert( neighbours.end(),node(m-1,s.y+1) );
            return;
        }
        //node in last column
        if (s.y==n){
            neighbours.insert( neighbours.end(),node(s.x-1,n) );
            neighbours.insert( neighbours.end(),node(s.x+1,n) );
            neighbours.insert( neighbours.end(),node(s.x-1,n-1) );
            neighbours.insert( neighbours.end(),node(s.x,n-1) );
            neighbours.insert( neighbours.end(),node(s.x+1,n-1) );
            return;
        }
        // opsti slucaj
        neighbours.insert( neighbours.end(), node(s.x-1,s.y-1)  );
        neighbours.insert( neighbours.end(), node(s.x-1,s.y)  );
        neighbours.insert( neighbours.end(), node(s.x-1,s.y+1)  );
        neighbours.insert( neighbours.end(), node(s.x,s.y-1) );
        neighbours.insert( neighbours.end(), node(s.x,s.y+1) );
        neighbours.insert( neighbours.end(), node(s.x+1,s.y-1) );
        neighbours.insert( neighbours.end(), node(s.x+1,s.y)  );
        neighbours.insert( neighbours.end(), node(s.x+1,s.y+1) );
        return;
}
/*If the map is obtained from sources other than this function, it can be skipped or edited.
The important thing is to update the rows attributes of class instance mapp.
(rows, columns, m, n, matrix, start and goal node (6 attributes)). */
void mapp::Generate_mapp(){
    int row,column;
    std::cout << "\n          " << "Enter the number of rows: ";
    std::cin >> row;
    if( row<1 ) std::cout << "ERROR!";
    std::cout << "\n          " << "Enter the number of columns: ";
    std::cin >> column;
    if( column<1 ) std::cout << "ERROR!";
    std::vector<std::vector<int> > temp(row, std::vector<int> (column,0));
    matrix=std::move(temp);
    rows=row;
    columns=column;
    m=row-1;
    n=column-1;
    int c;
    std::cout << "\n     How would you like to generate obstacles: ";
    std::cout << "\n     (1) no obstacles ";
    std::cout << "\n     (2) a column-wise snake ";
    std::cout << "\n     (3) a row-wise snake ";
    std::cout << "\n     (4) random obstacles ";
    std::cout << "\n     (any other integer) manual insertion of obstacles ";
    std::cin >> c;
    switch(c){
        case 1:
            break;
        case 2:
            {
            bool pom(false);
            for(int j=0; j<columns; j++){
                for(int i=0; i<rows; i++){
                    if(j%2==1){
                        matrix[i][j]=1;
                    }
                }
                if( pom && j%2==1 ){
                    matrix[0][j]=0;
                    pom=false;
                }
                else if( !pom && j%2==1 ){
                    pom=true;
                    matrix[m][j]=0;
                }
            }
            }
            break;
        case 3:
            {
            bool pom2(false);
            for(int i=0; i<rows; i++){
                for(int j=0; j<columns; j++){
                    if(i%2==1){
                        matrix[i][j]=1;
                    }
                }
                if( pom2 && i%2==1){
                    matrix[i][0]=0;
                    pom2=false;
                }
                else if( !pom2 && i%2==1 ){
                    matrix[i][n]=0;
                    pom2=true;
                }
            }
            break;
            }
        case 4:
            {
            int br(0),brmax(0);
            std::cout << "\n        How many obstacles do you want? ";
            std::cin >> brmax;
            if( brmax<0 || brmax>(rows*columns-2) ) std::cout << "\n      ERROR! Number of obstacles should be a positive integer and less than the map size!";
            int rand_x(0), rand_y(0);
            while( br < brmax   ){
                rand_x= std::rand()%rows;
                rand_y= std::rand()%columns;
                if (matrix[rand_x][rand_y]==1) br--;
                matrix[rand_x][rand_y]=1;
                br++;
            }
            break;
            }
        default:{
            std::cout << "\n        Input the x coordinate and afterwards the y coordinate of the node where you wish to place an obstacle. Enter a negative x coordinate to terminate";
            std::cout << std::endl;
            int p,q;
            while(1){
                std::cout << "      x coordinate:\n";
                std::cin >> p;
                std::cout << "      y coordinate:\n";
                std::cin >> q;
                if( p<0 || q<0 ) break;
                if( p>m || q>n ){
                        std::cout << "\n       ERROR! Index is not valid!";
                        break;
                }
                matrix[p][q]=1;
            }
            break;
        }
    };
    int preview(0);
    std::cout << "\n        Would you like to see a map preview?";
    std::cout << "\n        No - 0, Yes - any integer";
    std::cin >> preview;
    if (preview){
        std::cout << "\n        Here's a map preview!";
        (*this).Print_mapp();
    }
    int x,y;
    while(1){
        std::cout << "\n        Enter the start node coordinates:";
        std::cout << "\n x=";
        std::cin >> x;
        std::cout << "\n y=";
        std::cin >> y;
        if ( x<0 || y<0 || x>m || y>n || matrix[x][y]==1 ){
            std::cout << "\n        ERROR! Coordinates invalid or chosen node contains an obstacle. Repeat.";
        }
        else break;
    }
    start.x=x;
    start.y=y;
    while(1){
        std::cout << "\n        Enter the goal node coordinates:";
        std::cout << "\n x=";
        std::cin >> x;
        std::cout << "\n y=";
        std::cin >> y;
        if ( x<0 || y<0 || x>m || y>n || matrix[x][y]==1 ){
            std::cout << "\n        ERROR! Coordinates invalid or chosen node contains an obstacle. Repeat.";
        }
        else break;
    }
    goal.x=x;
    goal.y=y;
}

//Function that prints the mapp
void mapp::Print_mapp(){
    std::cout << std::endl;
    for(int i=0; i<rows; i++){
            std::cout << "          ";
        for(int j=0; j<columns; j++){
            std::cout << matrix[i][j] << "  ";
        }
        std::cout << std::endl;
    }
}

void mapp::Test_mapp_functionality(){
    (*this).Generate_mapp(); //REQUIRED - calls the map generation menu!
    //std::cout << "\n        Map preview: "; uncomment if a map preview is wanted
    //(*this).Print_mapp();
    //the chrono declarations are used to measure algorithm time only!
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        (*this).Astar();
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    double dif = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
    std::cout << "\nTime elapsed: " << dif;
    std::cout << "\n        Would you like a map preview (no -0, yes - any)";
    int preview(0);
    std::cin >> preview;
    if(preview){ (*this).Print_mapp(); }
    //(*this).Pass_to_MATLAB(); ONLY UNCOMMENT if the pass to matlab function has been appropriately  modified!
}

//Essential function. Runs A* on a given instance of class mapp.
void mapp::Astar(){
    //initialize by setting start node parameters
    start.g_inf=false;
    start.null_parent=false;
    start.x_p=start.x;
    start.y_p=start.y;
    start.g_value=0;
    start.f_value=start.g_value+start.heuristic(goal.x,goal.y);
    open_list.insert(open_list.end(),start);
    while( !open_list.empty() ){
        open_list.sort(Compare);
        node s;
        s=*open_list.begin();
        open_list.erase(open_list.begin()); //remove current node from open list
        if ( s==goal ){
            goal.x_p=s.x_p;
            goal.y_p=s.y_p;
            std::cout << "\n        Path found!";
            (*this).Path_extraction();
            return;
        }
        closed_list.insert(closed_list.end(), s);   //insert node which is about to be expanded into closed list
        while( !neighbours.empty() ){
            neighbours.erase(neighbours.begin());    //make sure neighbours list is empty
        }
        (*this).Update_neighbours(s);
        (*this).Filter_obstacles();
        for( std::list<node>::iterator it=neighbours.begin(); it!=neighbours.end(); it++){
            if( !(*this).In_closedlist(*it) ){
                if( !(*this).In_openlist(*it) ){
                    (*it).g_value=0;
                    (*it).f_value=0;
                    (*it).null_parent=true;
                    (*it).x_p=0;
                    (*it).y_p=0;
                    (*it).g_inf=true;
                }
                (*this).Update_vertex(s, (*it));
            }
        }
    }
    std::cout << "\n        No path found!";
    return;
}



int main(){
    std::cout << "A* by Kemal Altwlkany" << std::endl;
    mapp example;
    example.Test_mapp_functionality();
    return 0;
}

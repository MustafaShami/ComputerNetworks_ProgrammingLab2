//Mustafa Shami    May 3rd 2022
// TO RUN PROGRAM IN TERMINAL (Example)
//      1.  'make clean'   -> command to remove old executable
//      2.  'make'         -> to create new executable
//      3.  './mylinkstate input.txt 7 0'   -> parameters are: inputfile name AND which node you want to find all the paths for
//                                                                  AND flag 1 or 0. (1 will display immediate table for each iteration)

//NOTE: The formatting gets messed up with large number of nodes
//NOTE: I added a table that tells distance from any node to the source node
//             It was not required, but I added it because it helped see the data
//              if the formatting got messed up.

//NOTE: 1073741824 represents infinity..... below this number can be a valid edgeweight i think.

#include <iostream>
#include <fstream>
#include<vector>
#include<ctime>
#include<iomanip>
using namespace std;


// Max Number of vertices possible in the graph
#define MAX_NODES 500
#define INF 1073741824 //Represents Infinity basically

class Node
{
public:
    int cost;
    int path;
    int link;

    void setCostnPath(int cost1, int path1);
    void printTableLine(int V, int step, int node, vector<int> listFoundV);
};
void Node::printTableLine(int V, int step, int node, vector<int> listFoundV)
{

    int countSpace = 41;
    if(node == 0) //format the initial nodes so they look good
    {
        if(cost == INF)
        {
            cout << right << setw(countSpace-step*2) << "INF" << "," << path;
        }
        if(cost >= 10 || path >= 10)
        {
            cout << right << setw(countSpace-step*2-1) << cost << "," << path;
        }
        else if(step == 7 || step == 8 || step == 9)
        {
            cout << right << setw(countSpace-step*2-1) << cost << "," << path;
        }
        else
        {
            cout << right << setw(countSpace-step*2) << cost << "," << path;
        }
    }
    else
    {
        if(cost == INF)
        {
            cout << right << setw(8) << "INF" <<  ","  << path;
        }
        else if(path >= 100)
        {
            cout << right << setw(6) << cost << "," << path;
        }
        else if (path >= 10)
        {
            cout << right << setw(7) << cost << "," << path;
        }
        else
        {
            cout << right << setw(8) << cost << "," << path;
        }
    }

}
void Node::setCostnPath(int cost1, int path1)
{
    cost = cost1;
    path = path1;
}

void outputFormat(int numVertices, int src); //put the function definition under main

//function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int shortestPath[], bool included[], int V)
{
    int min = INF; //minimum value is infinity until we find a better path
    int minIndex; //the index of from where the shortest path was found

    for (int v = V-1; v >= 0; v--)
        if (included[v] == false && shortestPath[v] <= min)
        {
            min = shortestPath[v];
            minIndex = v;
        }
    return minIndex; //returns the next vertex we are going to visit
}

// A function to print the constructed distance array
void printSolution(int dist[], int V)
{
    cout <<"Vertex \t Distance from Source" << endl;
    for (int i = 1; i <= V; i++)
    {
        cout << i << "\t\t"<<dist[i-1]<< endl;
    }
}

void printForwardingTable(Node table[], int V, int src) //prints out the forwarding table
{
    cout << endl;
    cout << endl;
    cout << "FORWARDING TABLE:" << endl;
    cout << right << "Destination ";
    cout << "|" << right << setw(8) << "Link" << endl;
    cout << "------------------------" << endl;
    for(int i=0; i<V; i++)
    {
        if(i == src) //don't need to print out the source cuz we already know the source routes to itself
        {
            continue;
        }
        cout << right << setw(5) << i + 1;
        cout << right << setw(8) << "|";
        cout << right << setw(5) << "(" << table[i].path << "," << table[i].link << ")";

        cout << endl;
    }
}
void createForwardingTable(Node array[], int V, int src) //end up with an object that contains the forwarding table
{
    // Source will be represented by Path
    Node forwardTable[V];
    for(int i=0;i<V;i++)
    {
        forwardTable[i].path = INF;
        forwardTable[i].cost = INF;
        forwardTable[i].link = INF;
    }

    int hopToNodes; //will jump around in the while loop until it finds the correct link for the node
    for(int i=0; i<V; i++)
    {
        if(array[i].path == src+1) //+1 cuz zero indexed
        {
            forwardTable[i].path = src+1;
            forwardTable[i].link = i+1;
        }
        else
        {
            hopToNodes = i;
            while (forwardTable[i].path != src+1)
            {
                hopToNodes = array[hopToNodes].path-1; //-1 here because hoptonodes is traversing zero indexed
                if(array[hopToNodes].path == src+1)
                {
                    forwardTable[i].path = src+1;
                    forwardTable[i].link = hopToNodes+1;
                }
            }
        }
    }

    printForwardingTable(forwardTable, V, src); // Calls the function to print out the forwarding table
//
//    for(int f=0; f < V; f++)
//    {
//        cout << forwardTable[f].path << "," << forwardTable[f].link << "   ";
//    }

}

// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
void dijkstra(int graph[MAX_NODES][MAX_NODES], int V, int src, int flag)
{
    int shortestPath[V]; //will hold the shortest distance from src to i
    bool included[V]; // included[i] will be true if vertex i is included in shortest path tree

    bool printedN[V]; // OUTPUT FORMATTING will store whether we have already printed out the node for the current step
    vector<int> orderOfNodes; //Keeps track of what order we visited nodes
    Node nodeArray[V][V]; //will store the table values (cost,pathUsed)
    Node forwardTable[V]; //use this array to make forwarding table

    // Initialize all distances as INFINITE and all nodes as false
    for (int i = 0; i < V; i++)
    {
        shortestPath[i] = INF;
        included[i] = false;
        printedN[i] = false;
    }

    // Distance of source vertex from itself is always 0
    shortestPath[src] = 0;

    // Find shortest path for all vertices
    for (int count = 0; count < V; count++) {
        // Pick the minimum distance vertex from the set of vertices not yet visited. u is always equal to src in the first iteration.
        int u = minDistance(shortestPath, included, V);

        // Mark the picked vertex as processed
        included[u] = true;

        cout << left  << count << "\t {"; // OUTPUT FORMATTING tells current step #

        // Update dist value of the adjacent vertices of the picked vertex.
        for (int v = 0; v < V; v++)
        {
            // Update shortestPath only if current vertex has not been visited and there is an edge from
            // u node to v node, and the total weight of path from src to v through u is
            // smaller than current value of the shortest path
            if (!included[v] && graph[u][v] && shortestPath[u] != INF && shortestPath[u] + graph[u][v] < shortestPath[v])
            {
                shortestPath[v] = shortestPath[u] + graph[u][v]; //new shortest path found!

                if(printedN[u] == false)
                {
                    printedN[u] = true;
                    orderOfNodes.insert(orderOfNodes.begin(), u+1); //insert into the list of visited nodes
                    for(int n=0; n < orderOfNodes.size(); n++)
                    {
                        if(n+1 == orderOfNodes.size())
                        {
                            cout << left <<orderOfNodes[n]; //so we don't get the extra comma at the end
                        }
                        else
                        {cout << left << orderOfNodes[n] << ",";}
                    } cout << "}";

                    nodeArray[count][v].setCostnPath(shortestPath[v], u+1); //insert int matrix of all the paths with costs
                }
                else
                {
                    nodeArray[count][v].setCostnPath(shortestPath[v], u+1); //if already visited add path to the array
                }
            }
            else
            {
                if(printedN[u] == false)
                {
                    printedN[u] = true;
                    orderOfNodes.push_back(u+1);
                    for(int n=0; n < orderOfNodes.size(); n++)
                    {
                        if(n+1 == orderOfNodes.size())
                        {
                            cout << left << orderOfNodes[n];
                        }
                        else
                        {cout << left << orderOfNodes[n] << ",";}
                    } cout << "}";

                    nodeArray[count][v].setCostnPath(shortestPath[v], u+1);
                }
                else
                {
                    nodeArray[count][v].setCostnPath(shortestPath[v], u+1);
                }
            }
        }
//        Organizing the array and removing all the unnecessary paths to try and clean up the final look
        for(int z=0; z<V; z++) //z is rows
        {
            if(count > 0)
            {
                if(nodeArray[count][z].cost == nodeArray[count-1][z].cost)
                {
                    nodeArray[count][z].path = nodeArray[count-1][z].path;
                }
            }
        }
        // PRINT THE TABLE
        for(int z=0; z<V; z++) //z is rows
        {
            if(z == src) //don't need to include the source vertex's column since we already know it will go to itself with 0 path weight
            {
                continue;
            }
            nodeArray[count][z].printTableLine(V, count, z, orderOfNodes);
        }
        cout << endl;
        //Iterative print based on if input parameter flag was 1
        if(flag == 1)
        {
            cout << endl;
            if(count == V-1)
            {
                cout << endl;
            }
            else
            {
                outputFormat(V, src);
            }

        }
    }
    //NOW DO FORWARDING TABLE
    for(int col=0; col < V; col++)
    {
        forwardTable[col].cost = nodeArray[V-1][col].cost;
        forwardTable[col].path = nodeArray[V-1][col].path;
    }
    createForwardingTable(forwardTable, V, src);
    cout << endl;

    cout <<"-------------------------------" << endl;
    // print the constructed distance array
    printSolution(shortestPath, V);
}


// driver program to test above function
int main(int argc, char* argv[]) {
    clock_t time = clock(); // start the clock

    ifstream inputFile;

    ////TODO#1: Comment this out for submission
    //inputFile.open("/Users/mustafa/Desktop/SPRING 2022/Intro Networks CSCI3761/ProgrammingAssignment2/input.txt");

    ////TODO#2: Uncomment This for submission to CSEGRID (or to run from terminal)
    inputFile.open(argv[1]);
    int sourceNode = atoi(argv[2]);
    int flag = atoi(argv[3]);

    int numVertices = 0;
    int graph[MAX_NODES][MAX_NODES];

    if (inputFile.is_open()) {
        inputFile >> numVertices;
        int node1;
        int node2;
        int edgeWeight;
        while (inputFile >> node1) {
            inputFile >> node2;
            inputFile >> edgeWeight;

            if (edgeWeight == 1073741824) {
                edgeWeight = 0;
            }

            graph[node1 - 1][node2 - 1] = edgeWeight; // -1 cuz zero indexed
        }

    } else {
        cout << "Error::Unable to open input file!" << endl;
    }


    ////TODO #3: Uncomment this for submission to CSEGRID (or to run from terminal)
    outputFormat(numVertices, sourceNode-1);
    int nodeToVisit = sourceNode - 1;
    dijkstra(graph, numVertices, nodeToVisit, flag);

    ////TODO #4: Comment this out for submission
//    int MANUAL_SOURCE = 6;
//    outputFormat(numVertices, MANUAL_SOURCE);;
//    dijkstra(graph, numVertices, MANUAL_SOURCE, 0);


    inputFile.close();

    time = clock() - time; //stop clock
    cout << "--------------------------------" << endl;
    cout << "Execution Time: " << ((float) time / CLOCKS_PER_SEC) * 1000 << " milliseconds"
         << endl; //calculate runtime and print it out

    return 0;
}

void outputFormat(int numVertices, int src) {
    int count = 0;
    while(count != numVertices)
    {
        cout << "--------------------";
        count++;
    }
    cout << endl;
    cout << "Step" << setw(20)  << "N'";


    for(int i=0; i<numVertices; i++)
    {
        if(i==0)
        {
            cout << right << setw(30) << i+1;
        }
        else
        {
            if(i==src)
            {
                continue;
            }
            else
            {
                cout << right << setw(10) << i+1;
            }
        }
    }
    cout << endl;
    count=0;
    while(count != numVertices)
    {
        cout << "--------------------";
        count++;
    }
    cout<<endl;
}

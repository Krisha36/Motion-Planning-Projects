/*================================================================= 
*
* planner.c
*
*=================================================================*/
#include "planner.h"
#include <math.h>
#include <iostream>
using namespace std;
#include <queue>
#include <vector>
#include <unordered_map>
#include <set> 
#include <chrono> 
using namespace std::chrono;
#include <string>
#include <time.h>

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#define GETKEYINDEX(X, Y, T, XSIZE, YSIZE, TSIZE) ((X-1)*TSIZE+(Y-1)*XSIZE*(TSIZE)+T)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 9
#define e 1

class node{
public:
    int posX, posY;
    double h;
    double g;
    int t;
    int cost;
    node* parent;
};

struct CompareG{
    bool operator()(const node* lhs, const node* rhs) const
    {
        return (lhs->g) > (rhs->g);
    }
};

struct CompareH{
    bool operator()(const node* lhs, const node* rhs) const
    {
        return (lhs->g + e*lhs->h) > (rhs->g + e*rhs->h);
    }    
};

unordered_map<int,int> calculate_heuristic(int x_size, int y_size, double* target_traj, int time_steps, double* map, int collision_thresh){
    // 8-connected grid
    int dX[8] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[8] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // various important instantiations
    priority_queue<node*, vector<node*>, CompareG> open_list;
    node* currentPtr;
    node* neighborPtr;
    unordered_map<int, node*> openPtrList;
    unordered_map<int, bool> closedPtrList;
    int neighborX, neighborY, neighborKey, currentKey;
    unordered_map<int,int> heuristic(GETMAPINDEX(x_size, y_size, x_size, y_size));

    //initialise all the trajectory nodes and put it in the open_list.
    for (int i = 0; i < time_steps; i++){
        currentKey = GETMAPINDEX(target_traj[i], target_traj[i+time_steps], x_size, y_size);
        currentPtr = new node();
        currentPtr->posX = target_traj[i];
        currentPtr->posY = target_traj[i+time_steps];
        currentPtr->g = 0;
        currentPtr->cost = (int)map[GETMAPINDEX(currentPtr->posX,currentPtr->posY,x_size,y_size)];
        heuristic[currentKey] = currentPtr->g;
        open_list.push(currentPtr);
    }

    //Dijkstra search for heuristic
    while(!open_list.empty()){
        //remove the top node from the open_list with the minimum f value and add it to the closed list.
        currentPtr = open_list.top();
        open_list.pop();
        currentKey = GETMAPINDEX(currentPtr->posX, currentPtr->posY, x_size, y_size);

        //check if the node has been popped before. If it has, then it is a duplicate entry in the openlist.
        if (closedPtrList[currentKey]){
            continue;
        }
        closedPtrList[currentKey] = true;

        //go through the neighbours of the current node.
        for (int dir = 0; dir < 8; dir++){
            int neighborX = currentPtr->posX + dX[dir];
            int neighborY = currentPtr->posY + dY[dir];

            //only if the neighbor is within bounds and within collision threshold
            if ((neighborX >= 1 && neighborX <= x_size && neighborY >= 1 && neighborY <= y_size) && ((int)map[GETMAPINDEX(neighborX,neighborY,x_size,y_size)] < collision_thresh)){

                // generate the neighbor key.
                neighborKey = GETMAPINDEX(neighborX, neighborY, x_size, y_size);
            
                //if the neighbor is already in the closed list, skip.
                if (closedPtrList[neighborKey]){
                    continue;
                }

                //check if the neighbor has been visited before
                if (openPtrList[neighborKey]){
                    neighborPtr = openPtrList[neighborKey];
                    //update the gvalue and the parent
                    if(neighborPtr->g > currentPtr->g + neighborPtr->cost){
                        neighborPtr->g = currentPtr->g + neighborPtr->cost;
                        heuristic[neighborKey] = neighborPtr->g;
                        open_list.push(neighborPtr);
                    }
                }
                else{
                    neighborPtr = new node();
                    neighborPtr->posX = neighborX;
                    neighborPtr->posY = neighborY;
                    neighborPtr->cost = (int)map[GETMAPINDEX(neighborX,neighborY,x_size,y_size)];
                    openPtrList[neighborKey] = neighborPtr;

                    //update the g values
                    neighborPtr->g = currentPtr->g + neighborPtr->cost;
                    heuristic[neighborKey] = neighborPtr->g;
                    open_list.push(neighborPtr);
                }
            }
        }
    }
    return heuristic;
}

bool goalReached(node* currentPtr, double* target_traj, int target_steps, int extra_time){
    // check from current time step(curr_time) to the end time step(target_steps), if any of the goal state has been expanded
    int time1;
    time1 = currentPtr->t;

    if ((currentPtr->posX == target_traj[time1 + extra_time]) && (currentPtr->posY == target_traj[time1 + extra_time + target_steps])){
            return true;
        }
    return false;
}

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{   
    // 8-connected grid. Add extra dx and dy to signify staying at a position.
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 0};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 0};
    int key;
    static int step;
    static unordered_map<int, int> actions_x;
    static unordered_map<int, int> actions_y;
    
    if (curr_time == 0){
        time_t start;
        step = 0;
        start = time(NULL);

        // various important instantiations
        static priority_queue<node*, vector<node*>, CompareH> open_list;
        static unordered_map<int, node*> openPtrList;
        static unordered_map<int, bool> closedPtrList;
        unordered_map<int, int> count;
        unordered_map<int,int> heuristic;
        node* currentPtr;
        node* neighborPtr;
        node* previousPtr;
        time_t end;
        int neighborX, neighborY, neighborKey, neighborKey2, currentKey,time1, extra_time;

        //clear the open_list, openPtrList, closedPtrList.
        open_list = priority_queue<node*, vector<node*>, CompareH>();
        openPtrList.clear();
        closedPtrList.clear();
        heuristic = calculate_heuristic(x_size, y_size, target_traj, target_steps, map, collision_thresh);

        //Create the start_node pointer and store it's values, push it into the open_list.
        node* startPtr = new node();
        startPtr->posX = robotposeX; 
        startPtr->posY = robotposeY;
        startPtr->t = curr_time;
        open_list.push(startPtr);
        
        //A* search  
        while(!open_list.empty()){
            //remove the top node from the open_list with the minimum f value and add it to the closed list.
            currentPtr = open_list.top();
            open_list.pop();
            currentKey = GETKEYINDEX(currentPtr->posX, currentPtr->posY, currentPtr->t, x_size, y_size, target_steps);
            time1 = currentPtr->t;

            //check if the node has been popped before. If it has, then it is a duplicate entry in the openlist.
            if (closedPtrList[currentKey]){
                continue;
            }
            closedPtrList[currentKey] = true;

            //if the current node added to the closed_list is the goal node, exit the A* search
            end = time(NULL);
            extra_time = end-start+1;
            if (goalReached(currentPtr, target_traj, target_steps, extra_time)){
                break;
            }

            //go through the neighbours of the current node.
            for (int dir = 0; dir < NUMOFDIRS; dir++){
                neighborX = currentPtr->posX + dX[dir];
                neighborY = currentPtr->posY + dY[dir];
                
                //only if the neighbor is within bounds and within collision threshold
                if ((neighborX >= 1 && neighborX <= x_size && neighborY >= 1 && neighborY <= y_size) && ((int)map[GETMAPINDEX(neighborX,neighborY,x_size,y_size)] < collision_thresh)){

                    // generate the neighbor key.
                    neighborKey = GETKEYINDEX(neighborX, neighborY, time1+1, x_size, y_size, target_steps);
                
                    //if the neighbor is already in the closed list, skip.
                    if (closedPtrList[neighborKey]){
                        continue;
                    }

                    //check if the neighbor has been visited before
                    if (openPtrList[neighborKey]){
                        neighborPtr = openPtrList[neighborKey];
                        //update the gvalue and the parent
                        if(neighborPtr->g > currentPtr->g + neighborPtr->cost){
                            neighborPtr->g = currentPtr->g + neighborPtr->cost;
                            neighborPtr->parent = currentPtr;
                            neighborPtr->t = time1+1;
                            open_list.push(neighborPtr);
                        }
                    }
                    else{
                        neighborPtr = new node();
                        neighborPtr->posX = neighborX;
                        neighborPtr->posY = neighborY;
                        neighborKey2 = GETMAPINDEX(neighborX, neighborY, x_size, y_size);
                        // count[neighborKey2] = count[neighborKey2] + 1;
                        neighborPtr->h = heuristic[neighborKey2];
                        neighborPtr->cost = (int)map[neighborKey2];
                        openPtrList[neighborKey] = neighborPtr;

                        //update the g values
                        neighborPtr->g = currentPtr->g + neighborPtr->cost;
                        neighborPtr->parent = currentPtr;
                        neighborPtr->t = time1+1;
                        open_list.push(neighborPtr);
                    }
                }
            }
        }

        for (int t = 0; t<= extra_time; t++){
            currentKey = GETKEYINDEX(currentPtr->posX, currentPtr->posY, currentPtr->t + t, x_size, y_size, target_steps);
            actions_x[currentKey] = currentPtr->posX;
            actions_y[currentKey] = currentPtr->posY;
        }

        previousPtr = currentPtr->parent;

        while(previousPtr){
            key = GETKEYINDEX(previousPtr->posX, previousPtr->posY, previousPtr->t, x_size, y_size, target_steps);
            actions_x[key] = currentPtr->posX;
            actions_y[key] = currentPtr->posY;
            currentPtr = previousPtr;
            previousPtr = currentPtr->parent;
        }
    }

    key = GETKEYINDEX(robotposeX, robotposeY, step, x_size, y_size, target_steps);
    //set the action
    action_ptr[0] = actions_x[key];
    action_ptr[1] = actions_y[key];
    step++;

    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make


#ifndef Astar_h
#define Astar_h

 

class Astar

public:
	     //Declaring some constants
	const mapWidth = 80, mapHeight = 60, tileSize = 10, numberPeople = 3;
	int onClosedList = 10;
	const notfinished = 0, notStarted = 0;// path-related constants
	const found = 1, nonexistent = 2; 
	const walkable = 0, unwalkable = 1;// walkability array constants

	//Create needed arrays
	char walkability [mapWidth][mapHeight];
	int openList[mapWidth*mapHeight+2]; //1 dimensional array holding ID# of open list items
	int whichList[mapWidth+1][mapHeight+1];  //2 dimensional array used to record 
                                             //whether a cell is on the open list or on the closed list.
	int openX[mapWidth*mapHeight+2]; //1d array stores the x location of an item on the open list
	int openY[mapWidth*mapHeight+2]; //1d array stores the y location of an item on the open list
	int parentX[mapWidth+1][mapHeight+1]; //2d array to store parent of each cell (x)
	int parentY[mapWidth+1][mapHeight+1]; //2d array to store parent of each cell (y)
	int Fcost[mapWidth*mapHeight+2];	//1d array to store F cost of a cell on the open list
	int Gcost[mapWidth+1][mapHeight+1]; 	//2d array to store G cost for each cell.
	int Hcost[mapWidth*mapHeight+2];	//1d array to store H cost of a cell on the open list
	int pathLength[numberPeople+1];     //stores length of the found path  
	int pathLocation[numberPeople+1];   //stores current position along the chosen path  	
	int* pathBank [numberPeople+1];

	//Path reading variables
	int pathStatus[numberPeople+1];
	int xPath[numberPeople+1];
	int yPath[numberPeople+1];

// functions included: 
void InitializePathfinder(void);
void EndPathfinder(void);
int FindPath (int pathfinderID,int startingX, int startingY, int targetX, int targetY);





#endif
	


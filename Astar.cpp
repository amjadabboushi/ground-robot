
 

#include "Astar.h"

class Astar


//Allocating memory for the pathfinder.
 void Astar::InitializePathfinder (void)
{
	for (int x = 0; x < numberPeople+1; x++)
		pathBank [x] = (int*) malloc(4);
}


//Freeing the memory used by the pathfinder.
 void Astar::EndPathfinder (void)
{
	for (int x = 0; x < numberPeople+1; x++)
	{
		free (pathBank [x]);
	}
}


//Finding a path using A*  

int Astar::FindPath (int pathfinderID,int startingX, int startingY, int targetX, int targetY)
{
	int onOpenList=0, parentXval=0, parentYval=0,	a=0, b=0, m=0, u=0, v=0, temp=0, corner=0, numberOfOpenListItems=0;
	 int addedGCost=0, tempGcost = 0, path = 0,	tempx, pathX, pathY, cellPosition,	newOpenListItemID=0;

//1) Convert location data to coordinates in the walkability array.
	int startX = startingX/tileSize;
	int startY = startingY/tileSize;	
	targetX = targetX/tileSize;
	targetY = targetY/tileSize;

//2) Path Checks for being under some unexpected circumstances that result in no path. 
 
//	If starting location and target are in the same location 
	if (startX == targetX && startY == targetY && pathLocation[pathfinderID] > 0)
		return found;
	if (startX == targetX && startY == targetY && pathLocation[pathfinderID] == 0)
		return nonexistent;

//	If target square is unwalkable, return that it's a nonexistent path.
	if (walkability[targetX][targetY] == unwalkable)
		goto noPath;

//3) Reseting some variables that need to be cleared 
	if (onClosedList > 1000000) //reset whichList occasionally 
	{
		for (int x = 0; x < mapWidth;x++) {
			for (int y = 0; y < mapHeight;y++)
				whichList [x][y] = 0;
		}
		onClosedList = 10;	
	}
	onClosedList = onClosedList+2; //changing the values of onOpenList and onClosed list is faster than redimming whichList() array
	onOpenList = onClosedList-1;
	pathLength [pathfinderID] = notStarted;//i.e, = 0
	pathLocation [pathfinderID] = notStarted;//i.e, = 0
	Gcost[startX][startY] = 0; //reset starting square's G value to 0

//4) Adding the starting location to the open list of squares to be checked.
	numberOfOpenListItems = 1;
	openList[1] = 1;//assign it as the top (and currently only) item in the open list, which is maintained as a binary heap (explained below)
	openX[1] = startX ; openY[1] = startY;

//5)Do the following until a path is found or it doesn't exist. 
	do
	{

//6)If the open list is not empty, taking the first cell off of the list.
//	This is the lowest F cost cell on the open list.
	if (numberOfOpenListItems != 0)
	{

//7) Poping the first item off the open list.
	parentXval = openX[openList[1]];
	parentYval = openY[openList[1]]; //record cell coordinates of the item
	whichList[parentXval][parentYval] = onClosedList;//add the item to the closed list

//	Open List = Binary Heap: Delete this item from the open list, which is maintained as a binary heap.
	numberOfOpenListItems = numberOfOpenListItems - 1;//reduce number of open list items by 1	
		
//	Deleting the top item in binary heap and reorder the heap, with the lowest F cost item rising to the top.
	openList[1] = openList[numberOfOpenListItems+1]; //move the last item in the heap up to slot #1, other slots aren't in order. 
	v = 1;

//	Repeat the following until the new item in slot #1 sinks to its proper spot in the heap.
	do
	{
	u = v;		
	if (2*u+1 <= numberOfOpenListItems) //if both children exist.
	{
	 	//Checking if the F cost of the parent is greater than each child then select the lowest of the two children.
		if (Fcost[openList[u]] >= Fcost[openList[2*u]]) 
			v = 2*u;
		if (Fcost[openList[v]] >= Fcost[openList[2*u+1]]) 
			v = 2*u+1;		
	}
	else
	{
		if (2*u <= numberOfOpenListItems) //if only child #1 exists
		{
	 	//Checking if the F cost of the parent is greater than child #1	
			if (Fcost[openList[u]] >= Fcost[openList[2*u]]) 
				v = 2*u;
		}
	}

	if (u != v) //if parent's F is > one of its children, then they will be swapped: 
	{
		temp = openList[u];
		openList[u] = openList[v];
		openList[v] = temp;			
	}
	else
		break; //otherwise, exiting the loop.
		
	}
	while (!KeyDown(27));//reorder the binary heap


//7.Check the adjacent squares or the children 
	for (b = parentYval-1; b <= parentYval+1; b++){
	for (a = parentXval-1; a <= parentXval+1; a++){

//	just to check if they are included within the map or not ... for the chosen map size.
	if (a != -1 && b != -1 && a != mapWidth && b != mapHeight){

//	If not already on the closed list (items on the closed list have
//	already been considered and can now be ignored).			
	if (whichList[a][b] != onClosedList) { 
	
//	If not an obstacle square or not a free square.
	if (walkability [a][b] != unwalkable) { 
		
//	This part is very important, since the squares are most likely going to be equal to the size of the robot then cutting across corners must be avoided. 
	corner = walkable;	              // set to walkable unless otherwise is confirmed. 
	if (a == parentXval-1) 
	{
		if (b == parentYval-1)
		{
			if (walkability[parentXval-1][parentYval] == unwalkable
				|| walkability[parentXval][parentYval-1] == unwalkable) \
				corner = unwalkable;
		}
		else if (b == parentYval+1)
		{
			if (walkability[parentXval][parentYval+1] == unwalkable
				|| walkability[parentXval-1][parentYval] == unwalkable) 
				corner = unwalkable; 
		}
	}
	else if (a == parentXval+1)
	{
		if (b == parentYval-1)
		{
			if (walkability[parentXval][parentYval-1] == unwalkable 
				|| walkability[parentXval+1][parentYval] == unwalkable) 
				corner = unwalkable;
		}
		else if (b == parentYval+1)
		{
			if (walkability[parentXval+1][parentYval] == unwalkable 
				|| walkability[parentXval][parentYval+1] == unwalkable)
				corner = unwalkable; 
		}
	}	
	if (corner == walkable) {                          // if the corner has passed the test for unwalkability. 
	
//	adding all item to the open list which means that they will be investigated unless they are already on the open list from previous loops. 		
	if (whichList[a][b] != onOpenList) 
	{	

		//adding a new open list item to the binary heap.
		newOpenListItemID = newOpenListItemID + 1;    //each new item has a unique ID #
		m = numberOfOpenListItems+1;
		openList[m] = newOpenListItemID;             //place the new open list item (actually, its ID#) at the bottom of the heap
		openX[newOpenListItemID] = a;
		openY[newOpenListItemID] = b;               //record the x and y coordinates of the new item

		//Figure out its G cost
		if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
			addedGCost = 14;                                    //cost of going to diagonal squares, 14 and 10 has been used instead of 1 and 1.4 for faster calculation. 
		else	
			addedGCost = 10;                                    //cost of going to non-diagonal squares.				
		Gcost[a][b] = Gcost[parentXval][parentYval] + addedGCost;               // adding the previous best finished path until this point cost.

		//Finding the H and F costs and the parent: 
		Hcost[openList[m]] = 10*(abs(a - targetX) + abs(b - targetY));
		Fcost[openList[m]] = Gcost[a][b] + Hcost[openList[m]];
		parentX[a][b] = parentXval ; parentY[a][b] = parentYval;	

		//Moving the new open list item to the proper place in the binary heap.which starts at the bottom then by successively comparing to parent items,
		// then swapping as needed until the item finds its place in the heap which doesn't mean the exact correct location 
		//but its not number one unless it has lowest F cost.
		while (m != 1) //While item hasn't bubbled to the top (m=1)	
		{
			//Check if child's F cost is < parent's F cost. If so, swap them.	
			if (Fcost[openList[m]] <= Fcost[openList[m/2]])
			{
				temp = openList[m/2];
				openList[m/2] = openList[m];
				openList[m] = temp;
				m = m/2;
			}
			else
				break;
		}
		numberOfOpenListItems = numberOfOpenListItems+1;       //adding one to the number of items in the heap

		//Changing whichList to show that the new item is on the open list.
		whichList[a][b] = onOpenList;
	}

//8)If adjacent cell is already on the open list, checking to see if this path to that cell from the starting location is a better one. 
//	If so, change the parent of the cell and its G and F costs.	

	else                                                       //If whichList(a,b) = onOpenList
	{
	
		//Figure out the G cost of this possible new path
		if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
			addedGCost = 14;//cost of going to diagonal tiles	
		else	
			addedGCost = 10;//cost of going to non-diagonal tiles				
		tempGcost = Gcost[parentXval][parentYval] + addedGCost;
		
		//If this path is shorter (G cost is lower) then the parent cell must be changed, G cost and F cost. 		
		if (tempGcost < Gcost[a][b])                  //if G cost is less,
		{
			parentX[a][b] = parentXval;              //changing the square's parent
			parentY[a][b] = parentYval;
			Gcost[a][b] = tempGcost;                 //changing the G cost			

			//Because changing the G cost also changes the F cost, and so if the item is on the open list, the recorded F cost must be changed 
			//and its position on the open list must be updated.
			for (int x = 1; x <= numberOfOpenListItems; x++)         //looking for the item in the heap by matching its x and y coordinates to the list values.
			{
			if (openX[openList[x]] == a && openY[openList[x]] == b)  
			{
				Fcost[openList[x]] = Gcost[a][b] + Hcost[openList[x]];//updating the F cost
				
				//checking if the last update required a new location update in the heap.
				m = x;
				while (m != 1)                                      //While the item hasn't jumped to the top of the tree (m=1)	
				{
					//Checking if child is < parent, then swap them if so.	
					if (Fcost[openList[m]] < Fcost[openList[m/2]])
					{
						temp = openList[m/2];
						openList[m/2] = openList[m];
						openList[m] = temp;
						m = m/2;
					}
					else
						break;
				} 
				break; //exit for x = loop
			} //If openX(openList(x)) = a
			} //For x = 1 To numberOfOpenListItems
		}//If tempGcost < Gcost(a,b)

	}//else If whichList(a,b) = onOpenList	
	}//the end of If statement for cutting a corner
	}//the end of If statement for not a wall/obstacle square.
	}//the end of If statement for not already on the closed list 
	}//the end of If statement for not off the map statement
	}//the end of for loop for : (a = parentXval-1; a <= parentXval+1; a++) loop
	}//the end of for loop for: (b = parentYval-1; b <= parentYval+1; b++){ loop

	}//the end of  if (numberOfOpenListItems != 0)

//9) If open list is empty and nothing has been added by looping then there is no path = reached dead end.
	else
	{
		path = nonexistent; break;
	}  

	//If target is added to open list then path has been found.
	if (whichList[targetX][targetY] == onOpenList)
	{
		path = found; break;
	}

	}
	while (1); //infinite loop until the goal has been reached or doesn't exist. 

//10.Saving the path if it exists.
	if (path == found)
	{

//a.Working backwards from the target to the starting location by checking
//	each cell's parent and also figuring out the length of the path.
	pathX = targetX; pathY = targetY;
	do
	{
		//Looking up the parent of the current cell.	
		tempx = parentX[pathX][pathY];		
		pathY = parentY[pathX][pathY];
		pathX = tempx;

		//path length.
		pathLength[pathfinderID] = pathLength[pathfinderID] + 1;
	}
	while (pathX != startX || pathY != startY);

//b.Resize the data bank to the right size in bytes.
	pathBank[pathfinderID] = (int*) realloc (pathBank[pathfinderID],
		pathLength[pathfinderID]*8);

//c. Now copy the path information over to the databank. butin  backwards by copying the information to the data bank in reverse order. The result is.

	pathX = targetX ; pathY = targetY;
	cellPosition = pathLength[pathfinderID]*2;//start at the end	
	do
	{
	cellPosition = cellPosition - 2;//working backwards 2 integers 
	pathBank[pathfinderID] [cellPosition] = pathX;
	pathBank[pathfinderID] [cellPosition+1] = pathY;

//d.Looking up the parent of the current cell.	
	tempx = parentX[pathX][pathY];		
	pathY = parentY[pathX][pathY];
	pathX = tempx;

//e.If we have reached the starting square, exit the loop.	
	}
	while (pathX != startX || pathY != startY);	

//11.Reading the first path step into xPath/yPath arrays
	ReadPath(pathfinderID,startingX,startingY,1);

	}
	return path;


//13.If there is no path to the selected target, then the xPath and yPath is going to be equal to the current location  with path doesn't existent.
noPath:
	xPath[pathfinderID] = startingX;
	yPath[pathfinderID] = startingY;
	return nonexistent;
	}




	


#endif
	


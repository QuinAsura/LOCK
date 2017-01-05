/* Description: Path finding using Astar Algorithm
 * Implementation Details: Generalised Weighted A-star 
 * Made for CMU Air-lab Assignment
*/


//Necessary Libraries
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <cmath>
#include <vector>

using namespace cv;
using namespace std;

//Initialise World Size to be zero
int WORLD_SIZE =0;


//Defining the Structure of the search cell
struct SearchCell
{
    public:
        // Create Necessary variables
        int map_x, map_y;
        int map_id;
        SearchCell * parent;
        float G;
        float H;

        //Default constructor
        SearchCell()
        {
            parent =0;
        }
        
        //Search cell constructor
        SearchCell(int x, int y, SearchCell * _parent)
        {
            map_x = x;
            map_y = y;
            parent = _parent;
            map_id = y*WORLD_SIZE + x;
            G=0;
            H=0;
        }

        //Returns Total Cost of the cell
        float GetF()
        {
            return G+H;
        }

        //Returns weighted Euclidean distance from current cell and a point
        float EuclideanDistance(SearchCell * nodeEnd)
        {
            float x = (float)(this->map_x- nodeEnd->map_x)*(float)(this->map_x- nodeEnd->map_x);
            float y = (float)(this->map_y- nodeEnd->map_y)* (float)(this->map_y- nodeEnd->map_y);
            return 3*(sqrt(x+y));
        }
};


//Defining Pathfinding Class
class PathFinding
{
    public:
        // Create Necessary variables
        bool m_foundGoal;
        SearchCell *m_startCell;
        SearchCell *m_goalCell;
        vector<SearchCell*> m_openList;
        vector<SearchCell*> m_visitedList;
        vector<Point2i> m_pathToGoal;
        Mat SearchSpace;

        //Constructor to Initialise the map world
        PathFinding(Mat graph)
        {
            SearchSpace = graph;
            m_foundGoal = false;
        }

        //This Function finds the shortest path and returns it as a vector
        vector<Point2i>  FindPath(Point2i currentPos, Point2i targetPos)
        {
            //Initialise Start and goal points
            SearchCell start;
            start.map_x = currentPos.x;
            start.map_y = currentPos.y;
            SearchCell goal;
            goal.map_x = targetPos.x;
            goal.map_y = targetPos.y;
            SetStartAndGoal(start,goal);

            //While goal not found and and full map not traversed,traverse the graph
            while((!m_foundGoal) && !(m_openList.empty()))
            {
                ContinuePath();
            }
            
            //terminate if all nodes were traversed and path not found
            if(m_openList.empty())
            {
                cout<<"Found No Path to Goal: Terminating"<<endl;
                exit(-1); 
            }
            //Return the path else
            else
            {
            return m_pathToGoal;
            }
        }
        
        //Function to set Start and goal cells
        void SetStartAndGoal(SearchCell start,SearchCell goal)
        {
            m_startCell = new SearchCell(start.map_x,start.map_y,NULL);
            m_goalCell = new SearchCell(goal.map_x,goal.map_y,NULL);
            m_startCell->G=0;
            m_startCell->H= m_startCell->EuclideanDistance(m_goalCell);
            m_startCell->parent = 0;
            m_openList.push_back(m_startCell);
        }

        void PathOpened(int x,int y,float newCost,SearchCell* parent)
        {
            
            //Check if point is valid for traversal- Return if not.
            if(CellBCheck(x,y))
            {
                return;
            }
            
            //Assign Id to the point
            int id = y * WORLD_SIZE + x;

            //Check if the Cell was visited previously. 
            //This prevents unnecessary rechecking of previously
            //visited points
            for(int i=0;i< m_visitedList.size();i++)
            {
                if(id == m_visitedList[i]->map_id)
                {
                    return;
                }
            }

            //Create a new search cell the point and assign Costs
            SearchCell * newChild = new SearchCell(x,y,parent);
            newChild->G = newCost;
            //Assign H to EuccledeanDistance to goal
            newChild->H = newChild->EuclideanDistance(m_goalCell);
            //Push the cell into openlist
            m_openList.push_back(newChild);
        }
        
        //To find next best to traverse
        //This function returns a cell with least cost in Openlist of nodes
        SearchCell *GetNextCell()
        {
            //Initialise best cost Inf
            float bestF = 9999999.0f;
            int cellIndex = -1;
            SearchCell * nextCell = NULL;

            //Traverse through openlist and find least cost node
            for(int i=0; i < m_openList.size();i++)
            {
                if(m_openList[i]->GetF()< bestF)
                {
                    bestF = m_openList[i]->GetF();
                    cellIndex = i;
                }
            }
            
            //assign best cell as next cell
            if(cellIndex>=0)
            {
                nextCell = m_openList[cellIndex];
                m_visitedList.push_back(nextCell);
                m_openList.erase(m_openList.begin()+ cellIndex);
            }
            return nextCell;
        }
        
        //Returns if current cell is valid for traversing or not
        bool CellBCheck(int x,int y)
        {
            //Check if out of bounds
            if((x<SearchSpace.size().width && x>=0) && (y<SearchSpace.size().height && y>=0))
            {
                //check if obstacle
                if(SearchSpace.at<uchar>(y,x)==0)
                {
                    return false;
                }
                else
                {
                    return true;
                }
            }
            else
            {
                return true;
            }

        }
        void ContinuePath()
        {

            //The cell with least cost in the openlist is assigned as current cell
            SearchCell * currentCell = GetNextCell();

            //If Goal node is reached
            if(currentCell->map_id == m_goalCell->map_id)
            {
                SearchCell* getPath;
                for(getPath = currentCell; getPath!= NULL; getPath= getPath->parent)
                {
                    //Add the nodes to a vector 
                    m_pathToGoal.push_back(Point2i(getPath->map_x,getPath->map_y));
                }

                //Set found goal flag 1
                m_foundGoal = true;
                return;
            }
            //Else expand into nodes in all Eight directions  
            else
            {
                //Here New cells are expanded and G cost is assinged as current cell G-cost+1
                //rightCell
                PathOpened(currentCell->map_x+1,currentCell->map_y,currentCell->G+1,currentCell);
                //leftCell
                PathOpened(currentCell->map_x-1,currentCell->map_y,currentCell->G+1,currentCell);
                //topCell
                PathOpened(currentCell->map_x,currentCell->map_y+1,currentCell->G+1,currentCell);
                //bottomCell
                PathOpened(currentCell->map_x,currentCell->map_y-1,currentCell->G+1,currentCell);
                //diagnaltopLeft
                PathOpened(currentCell->map_x-1,currentCell->map_y+1,currentCell->G+1,currentCell);
                //diagnaltopRight
                PathOpened(currentCell->map_x+1,currentCell->map_y+1,currentCell->G+1,currentCell);
                //diagnalbottomLeft
                PathOpened(currentCell->map_x-1,currentCell->map_y-1,currentCell->G+1,currentCell);
                //diagnalbottomRight
                PathOpened(currentCell->map_x+1,currentCell->map_y-1,currentCell->G+1,currentCell);
                
                //Erase current cell from Openlist, Since it didnt lead to goal  
                for(int i=0; i < m_openList.size();i++)
                {
                    if(currentCell->map_id == m_openList[i]->map_id)
                    {
                        m_openList.erase(m_openList.begin()+i);
                    }
                }

            }
        }
};

int main(int argc, char ** argv)
{

    //Initializing Matrices to store the images
    Mat src,bwsrc,thresholded;

    //Read image passed in the Command Line Arg. 
    src = imread(argv[1]);
    Size s = src.size();

    //Exit if No image is passed
    if(!src.data)
    {
        cerr<<"No Image Supplied"<<endl;
        return -1;
    }

    //Initialize plan output vector
    vector<Point2i> plan_point;

    //Set World size
    WORLD_SIZE=s.height*s.width;

    //Initialize SearchMap
    Mat map = Mat::zeros(s.height,s.width,CV_8UC1);
    
    //Convert input image to Gray scale (Since by default its loaded as color image
    cvtColor(src, bwsrc, cv::COLOR_RGB2GRAY);

    //Threshold the input DT image to obtain map 
    threshold(bwsrc,thresholded,10,255,THRESH_BINARY);
    threshold(thresholded,map,1,255,THRESH_BINARY_INV);

    //Add Map to A* Constructor
    PathFinding Astar(map); 

    //Find path and return the path found
    plan_point = Astar.FindPath(Point2i(140,200),Point2i(725,1095));
    
    //Add points A and B to source image to show start and end point 
    circle(src,Point(140,200),3,Scalar(255,0,0),2, 8, 0 );
    circle(src,Point(725,1095),3,Scalar(0,255,0),2, 8, 0 );

    //plot the path returned in the source image
    if(plan_point.size()>0)
    {
        cout<<"Optimal Path Found"<<endl;
        for(int i=0;i<plan_point.size();i++)
        {
            circle(src,Point(int(plan_point[i].x),int(plan_point[i].y)),1,Scalar(0,0,255),-1, 8, 0 );
        }

    //save the modified image into a file
    imwrite("Path.png",src);

    //Display the output
    namedWindow("MyImage1",CV_WINDOW_AUTOSIZE);
    imshow("MyImage1",src);
    waitKey();
    }
    else
    {
        cout<<"No Path found:"<<endl;
        return -1;
    }
    return 0;
}


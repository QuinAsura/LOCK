#include <iostream>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <SerialStream.h>
#include <string>
#include <sstream>
#include <unistd.h>


//GLOBAL DEFINES
#define PORT "/dev/ttyACM0"
#define PI 3.14159265358979323846
#define BOT_SIZE 5
#define WORLD_SIZE (640/BOT_SIZE)*(480/BOT_SIZE)
#define WORLD_W (640/BOT_SIZE)
#define WORLD_H (480/BOT_SIZE)


using namespace cv;
using namespace std;
using namespace LibSerial;
Point3f img2map(Point,Mat);
vector<Point3f> RetCenters(Mat);
bool BoundsCheck(int,bool);
Mat CreateMap(Mat,vector<Point2i> *);

VideoCapture cap(1);

struct SearchCell
{
    public:
        int m_xcoord, m_zcoord;
        int m_id;
        SearchCell * parent;
        float G;
        float H;

        SearchCell()
        {
            parent =0;
        }

        SearchCell(int x, int z, SearchCell * _parent)//parent =0
        {
            m_xcoord = x;
            m_zcoord = z;
            parent = _parent;
            m_id = z*WORLD_SIZE + x;
            G=0;
            H=0;
        }

        float GetF()
        {
            return G+H;
        }

        float ManHattanDistance(SearchCell * nodeEnd)
        {
            float x = (float)(fabs((float)(this->m_xcoord- nodeEnd->m_xcoord)));
            float z = (float)(fabs((float)(this->m_zcoord- nodeEnd->m_zcoord)));
            return x+z;
        }
};

class PathFinding
{
    public:
        bool m_intializedStartGoal;
        bool m_foundGoal;
        SearchCell *m_startCell;
        SearchCell *m_goalCell;
        vector<SearchCell*> m_openList;
        vector<SearchCell*> m_visitedList;
        vector<Point3f> m_pathToGoal;
        Mat SearchSpace;


        PathFinding(Mat graph)
        {
            SearchSpace = graph;
            m_intializedStartGoal = false;
            m_foundGoal = false;
        }

        void FindPath(Point3f currentPos, Point3f targetPos)
        {
            if(!m_intializedStartGoal)
            {
                m_openList.erase(m_openList.begin(),m_openList.end());
                m_visitedList.erase(m_visitedList.begin(),m_visitedList.end());
                m_pathToGoal.erase(m_pathToGoal.begin(),m_pathToGoal.end());
                m_openList.clear();
                m_visitedList.clear();
                m_pathToGoal.clear();
                cout<<m_openList.size()<<endl;
                cout<<m_visitedList.size()<<endl;
                SearchCell start;
                start.m_xcoord = currentPos.x;
                start.m_zcoord = currentPos.z;
                SearchCell goal;
                goal.m_xcoord = targetPos.x;
                goal.m_zcoord = targetPos.z;
                SetStartAndGoal(start,goal);
                m_intializedStartGoal = true;
            }

            while((!m_foundGoal) && !(m_openList.empty()))
            {
                ContinuePath();
            }

            if(m_openList.empty() && m_pathToGoal.size()==0)
            {
                //cout<<">>Found No Path to Goal: Terminating"<<endl;
                //cout<<">>Reset the Box Size"<<endl;
                m_intializedStartGoal = false;
                m_foundGoal = false;
                return;
            }
            m_intializedStartGoal = false;
            m_foundGoal = false;
            return;


        }

        void SetStartAndGoal(SearchCell start,SearchCell goal)
        {
            m_startCell = new SearchCell(start.m_xcoord,start.m_zcoord,NULL);
            m_goalCell = new SearchCell(goal.m_xcoord,goal.m_zcoord,NULL);//Made a change from &goal
            m_startCell->G=0;
            m_startCell->H= m_startCell->ManHattanDistance(m_goalCell);
            m_startCell->parent = 0;
            m_openList.push_back(m_startCell);
        }

        void PathOpened(int x,int z,float newCost,SearchCell* parent)
        {

            if(CellBCheck(x,z))
            {
                return;
            }

            int id = z * WORLD_SIZE + x;
            for(int i=0;i< m_visitedList.size();i++)
            {
                if(id == m_visitedList[i]->m_id)
                {
                    return;
                }
            }
            SearchCell * newChild = new SearchCell(x,z,parent);
            newChild->G = newCost;
            newChild->H = newChild->ManHattanDistance(m_goalCell);//replaced parent with newchild
            m_openList.push_back(newChild);
        }

        SearchCell *GetNextCell()
        {
            float bestF = 9999999.0f;
            int cellIndex = -1;
            SearchCell * nextCell = NULL;
            for(int i=0; i < m_openList.size();i++)
            {
                if(m_openList[i]->GetF()< bestF)
                {
                    bestF = m_openList[i]->GetF();
                    cellIndex = i;
                }
            }

            if(cellIndex>=0)
            {
                nextCell = m_openList[cellIndex];
                m_visitedList.push_back(nextCell);
                m_openList.erase(m_openList.begin()+ cellIndex);
            }
            return nextCell;
        }

        bool CellBCheck(int x,int z)
        {
            if((x<SearchSpace.size().width && x>=0) && (z<SearchSpace.size().height && z>=0))
            {
                if(SearchSpace.at<double>(z,x)==0)
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

        vector<Point3f> RetPlan()
        {
            return m_pathToGoal;
        }

        void ContinuePath()
        {

            SearchCell * currentCell = GetNextCell();
            if(currentCell->m_id == m_goalCell->m_id)
            {
                SearchCell* getPath;
                for(getPath = currentCell; getPath!= NULL; getPath= getPath->parent)
                {
                    m_pathToGoal.push_back(Point3f(getPath->m_xcoord,0,getPath->m_zcoord));
                }
                m_foundGoal = true;
                return;
            }
            else
            {
                //rightCell
                PathOpened(currentCell->m_xcoord+1,currentCell->m_zcoord,currentCell->G+1,currentCell);
                //leftCell
                PathOpened(currentCell->m_xcoord-1,currentCell->m_zcoord,currentCell->G+1,currentCell);
                //topCell
                PathOpened(currentCell->m_xcoord,currentCell->m_zcoord+1,currentCell->G+1,currentCell);
                //bottomCell
                PathOpened(currentCell->m_xcoord,currentCell->m_zcoord-1,currentCell->G+1,currentCell);

                //diagnaltopLeft
                PathOpened(currentCell->m_xcoord-1,currentCell->m_zcoord+1,currentCell->G+1.414,currentCell);
                //diagnaltopRight
                PathOpened(currentCell->m_xcoord+1,currentCell->m_zcoord+1,currentCell->G+1.414,currentCell);
                //diagnalbottomLeft
                PathOpened(currentCell->m_xcoord-1,currentCell->m_zcoord-1,currentCell->G+1.414,currentCell);
                //diagnalbottomRight
                PathOpened(currentCell->m_xcoord+1,currentCell->m_zcoord-1,currentCell->G+1.414,currentCell);
                /*
                //Experimental_Kinematically feasible paths
                PathOpened(currentCell->m_xcoord-2,currentCell->m_zcoord+2,currentCell->G+1,currentCell);
                //diagnaltopRight
                PathOpened(currentCell->m_xcoord+2,currentCell->m_zcoord+2,currentCell->G+1,currentCell);
                //diagnalbottomLeft
                PathOpened(currentCell->m_xcoord-2,currentCell->m_zcoord-2,currentCell->G+1,currentCell);
                //diagnalbottomRight
                PathOpened(currentCell->m_xcoord+2,currentCell->m_zcoord-2,currentCell->G+1,currentCell);
                */
                for(int i=0; i < m_openList.size();i++)
                {
                    if(currentCell->m_id == m_openList[i]->m_id)
                    {
                        m_openList.erase(m_openList.begin()+i);
                    }
                }

            }
        }
};

class SerialComm
{
    public:
        SerialStream node;

        SerialComm()
        {
            node.SetBaudRate(SerialStreamBuf::BAUD_9600);
            node.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
            node.Open(PORT);
        }

};


class Bot
{
    public:
        double Wradius,Blength,Bvelocity,pwm_lower,pwm_upper,omega_lower,omega_upper,rpm_lower,rpm_upper;
        int Wvelocities[2];
        double Kp,Ki,Kd,error,prev_error,Integ;
        long int time,prev_time,freq;

        Bot(double p,double i,double d)
        {
            Kp=p;Ki=i;Kd=d;error=0;prev_error=0;Integ=0;
            time=0;prev_time=0;freq = getTickFrequency();
            Wradius = 0.034;
            Blength = 0.105;
            Bvelocity = 0.045;
            pwm_lower = 35.0;
            pwm_upper = 254.0;
            omega_lower = 0.0;
            omega_upper = 8.35;

        }

        void Uni2DiffKinematics(double w)
        {
            double sum  = 2*Bvelocity/(Wradius);
            double diff = Blength*w/(Wradius);
            double l_w = (sum+diff)/2;
            double r_w = (sum-diff)/2;
            if ((l_w > omega_upper) || (r_w > omega_upper))
            {
                if (l_w > omega_upper)
                {
                    double temp = l_w - omega_upper;
                    l_w = omega_upper;
                    r_w -= temp;
                    if(r_w < omega_lower)
                    {
                        r_w =omega_lower;
                    }

                }
                else
                {
                    double temp = r_w -omega_upper;
                    r_w = omega_upper;
                    l_w -= temp;
                    if(l_w < omega_lower)
                    {
                        l_w=omega_lower;
                    }
                }
            }
            else if ((l_w < omega_lower) || (r_w < omega_lower))
            {
                if(l_w < omega_lower)
                {
                    double temp = omega_lower - l_w;
                    l_w = omega_lower;
                    r_w += temp;
                    if (r_w > omega_upper)
                    {r_w = omega_upper;}
                }
                else
                {
                    double temp = omega_lower - r_w;
                    r_w = omega_lower;
                    l_w += temp;
                    if(l_w > omega_upper)
                    {
                        l_w = omega_upper;
                    }
                }
            }
            l_w = pwm_lower + (((pwm_upper - pwm_lower)/(omega_upper -omega_lower))*l_w);
            r_w = pwm_lower + (((pwm_upper - pwm_lower)/(omega_upper -omega_lower))*r_w);
            Wvelocities[0]=int(l_w);
            Wvelocities[1]=int(r_w);
        }

        void ResetController()
        {
            prev_time = getTickCount();
            error=0;prev_error=0;Integ=0;
        }

        double PID_Controller(double accel)
        {
            double signal;
            error = accel;
            Integ+=error;
            time = getTickCount();
            double dt = (double(time - prev_time))/freq;
            signal = (error*Kp) + ((error - prev_error)*Kd*dt) + (Integ*Ki/dt);
            prev_error = error;
            prev_time = time;
            if(signal >=255)
            {
                return 255;
            }
            return signal;
        }
};

class SuperVisor
{
    public:
        int bLowH,bHighH,bLowS,bHighS,bLowV,bHighV;
        int rLowH,rHighH,rLowS,rHighS,rLowV,rHighV;
        int yLowH,yHighH,yLowS,yHighS,yLowV,yHighV;
        int gLowH,gHighH,gLowS,gHighS,gLowV,gHighV;
        Point Yc,Gc;
        vector< vector<Point2i> > final_plan;
        vector<double> t_wait;

        SuperVisor()
        {
            yLowH = 20;yHighH = 40;yLowS = 100;yHighS = 255;yLowV = 100;yHighV = 255;
            gLowH = 163;gHighH = 179;gLowS = 218;gHighS = 255;gLowV = 86;gHighV = 255;
        }

        //Return the indices to visit 
        void Planner()
        {
            Mat src,map; 
            bool bSuccess = cap.read(src); // read a new frame from video
            if (!bSuccess) //if not success, break loop
            {
                cout << "Cannot read the frame in Planner module" << endl;
            }
            vector<Point3f> circle_centers;
            vector<Point2i> map_point;
            vector<Point3f> plan_point;
            vector<Point2i> temp_point;
            circle_centers = RetCenters(src);
            for(int i=0;i < circle_centers.size();i++)
            {
                circle(src,Point(circle_centers[i].x,circle_centers[i].y),circle_centers[i].z+5,Scalar(255,255,255),-1,8,0);
            }
            //Circle Identification Check
            namedWindow("MyVideo",CV_WINDOW_AUTOSIZE);  
            imshow("MyVideo",src);
            waitKey(0);

            map = CreateMap(src,&map_point);
            PathFinding Astar(map);
            cout<<endl<<">>Map Created Place The Bot in the Arena and Enter Sample To Capture:"<<endl;
            int sample_weight;
            cin>>sample_weight;
            Mat src1;
            for(int i=0;i<10;i++)
            {
                bSuccess = cap.read(src1);
                if(!bSuccess)
                {
                    cout<<"Cannot read the second frame for GetXY in Planner module:"<<endl;
                } 
                GetXY(src1);
            }
            int visit_cnt=0;
            vector<Point> order_points;
            order_points.push_back(Yc);  
            int temp_cnt = 0;
            for(int j=30;j>=10;j-=10)
            {
                double cur_weight = (double(int(double(sample_weight)/j))); 
                if(cur_weight>0)
                {
                    order_points.push_back(Point(circle_centers[temp_cnt].x,circle_centers[temp_cnt].y));
                    t_wait.push_back(cur_weight);
                    visit_cnt++;
                }
                temp_cnt++;
                sample_weight = sample_weight % j;
            }
            cout<<visit_cnt<<endl;
            for(int i=0;i<visit_cnt;i++)
            {
                if(i==0)
                {
                cout<<order_points[i]<<endl;
                cout<<order_points[i+1]<<endl;
                Point3f start = img2map(order_points[i],src);
                //cout<<start<<endl;
                Point3f end   = img2map(order_points[i+1],src);
                Astar.FindPath(start,end);
                //cout<<end<<endl;
                plan_point = Astar.RetPlan();
                if(plan_point.size()>0)
                {
                    cout<<">>Optimal Path Detected:"<<endl;
                    for(int i=0;i<plan_point.size();i++)
                    {
                        Point2i temp = map_point[int((plan_point[i].x*(WORLD_H))+plan_point[i].z)];
                        //cout<<temp.x<<"\t"<<temp.y<<endl;
                        circle(src,Point(int((BOT_SIZE/2)+temp.x),int((BOT_SIZE/2)+temp.y)),3,Scalar(0,0,0),-1, 8, 0 );
                        temp_point.push_back(temp);
                    }
                    final_plan.push_back(temp_point);
                    temp_point.erase(temp_point.begin(),temp_point.end());
                    temp_point.clear();
                }
                }
                else
                {
                    Point3f end   = img2map(order_points[i+1],src);
                    Point2i temp = map_point[int((end.x*(WORLD_H))+end.z)];
                    cout<<temp<<endl;
                    circle(src,Point(int((BOT_SIZE/2)+temp.x),int((BOT_SIZE/2)+temp.y)),3,Scalar(0,0,0),-1, 8, 0 );
                    temp_point.push_back(temp);
                    final_plan.push_back(temp_point);
                    temp_point.erase(temp_point.begin(),temp_point.end());
                    temp_point.clear();
                }
                namedWindow("MyVideo",CV_WINDOW_AUTOSIZE);  
                imshow("MyVideo",src);
                waitKey(0);
            }
        }


        void GetXY(Mat frame)
        {
            Mat imgHSV,imgThresholded,greenthresh;
            vector<vector<Point> > contours,gcontours;
            vector<Vec4i> hierarchy,ghierarchy;
            Moments mu,gmu;
            cvtColor(frame,imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

            inRange(imgHSV, Scalar(yLowH, yLowS, yLowV), Scalar(yHighH, yHighS, yHighV), imgThresholded); //Threshold the image
            inRange(imgHSV, Scalar(gLowH, gLowS, gLowV), Scalar(gHighH, gHighS, gHighV), greenthresh);
            //morphological opening (removes small objects from the foreground)

            erode(greenthresh, greenthresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            dilate( greenthresh, greenthresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
            dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
            //morphological closing (removes small holes from the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
            dilate( greenthresh, greenthresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            erode(greenthresh, greenthresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            findContours(imgThresholded,contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
            findContours(greenthresh,gcontours, ghierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

            int maxArea =0;
            int bestcnt =0;
            for(int i = 0; i < contours.size(); i++)
            { 
                if (contourArea(contours[i])> maxArea)
                {
                    maxArea = contourArea(contours[i]);
                    bestcnt = i;
                }
            }

            int gmaxArea = 0;
            int gbestcnt = 0;
            for(int i = 0; i < gcontours.size(); i++)
            { 
                if (contourArea(gcontours[i]) > gmaxArea)
                {
                    gmaxArea = contourArea(gcontours[i]);
                    gbestcnt = i;
                }
            }


            if (!((gcontours.size()==0) || (contours.size()==0)))
            {

                mu = moments(contours[bestcnt],false);
                gmu = moments(gcontours[gbestcnt],false);
                Yc= Point2f( mu.m10/mu.m00,mu.m01/mu.m00 );
                Gc = Point2f( gmu.m10/gmu.m00,gmu.m01/gmu.m00 );
            }
        }

        double GoToGoal(Point goal)
        {
            Point oriented_dir = Gc-Yc;
            Point goal_dir = Gc - goal;
            double dot = oriented_dir.x*goal_dir.x + oriented_dir.y*goal_dir.y;;
            double det = oriented_dir.x*goal_dir.y - oriented_dir.y*goal_dir.x;
            return (atan2(det,dot));
        }


        void Run(Bot * Drive,SerialComm *TCom)
        {
            Planner();
            for(int i = 0; i <final_plan.size();i++)
            {
                (*Drive).ResetController();
                bool gate;
                for(int j = final_plan[i].size()-1; j>=0; j--)
                {
                    
                    do
                    {
                        Mat src;
                        bool bSuccess = cap.read(src); // read a new frame from video
                        if (!bSuccess) //if not success, break loop
                        {
                            cout << "Cannot read the frame in SuperVisor module in Run Function" << endl;
                        }
                        GetXY(src);
                        circle(src, Yc,3,Scalar(0,0,0),1,8,0);
                        circle(src, Gc,3,Scalar(0,0,0),1,8,0);
                        circle(src, final_plan[i][j],3,Scalar(0,0,0),1,8,0);
                        imshow("MyVideo",src);
                        if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
                        {
                            cout << "esc key is pressed by user" << endl; 
                            break; 
                        }
                        ostringstream s1,s2;
                        string s3,s4,s5,s6;
                        double temp = GoToGoal(final_plan[i][j]);
                        if(abs(temp*180/PI)<15)
                        {
                            (*Drive).Bvelocity = 0.405;
                        }
                        else
                        {

                            (*Drive).Bvelocity = 0.025;
                        }
                        //cout<<abs(temp*180/PI)<<endl;
                        double signal = (*Drive).PID_Controller(temp); 
                        (*Drive).Uni2DiffKinematics(signal);
                        s1<<((*Drive).Wvelocities[0]);
                        s2<<((*Drive).Wvelocities[1]);
                        s3 = s1.str();
                        s4 = s2.str();
                        if (s3.length()<3)
                            s3 = "0"+s3;
                        if (s4.length()<3)
                            s4 = "0"+s4; 
                        s5 = "$"+s3+"#"+s4+"!";
                        //cout<<s5<<endl;
                        const char * output_buffer = s5.c_str();
                        (*TCom).node.write(output_buffer,9);
                        if(j==0)
                        {
                            gate = (double(norm(Yc-final_plan[i][j]))>20);
                        }
                        else
                        {
                            gate = (double(norm(Yc-final_plan[i][j]))>40);
                        }
                    }while(gate);
                }
                string s6="$000&000!";
                const char * stop = s6.c_str();
                (*TCom).node.write(stop,9);
                usleep(3000000*t_wait[i]);
            }

        }
};


int main(int argc,char** argv)
{
    cap.open("http://192.168.43.1:8080/video?dummy=param.mjpg");
    //Camera Module Check
    if(!cap.isOpened())
    {
        cerr<<"Camera Check:[FAILED]"<<endl;
        return -1;
    }
    cout<<">>Camera Check:[OK]"<<endl;
    //Serial Communication Check
    SerialComm Comm; 
    if ( ! Comm.node.good() )
    {
        cerr << ">>Serial Check:[FAILED]"<<endl;
        return -1;
    }
    cout<<">>Serial Check:[OK]"<<endl;
    //Creation Of Required Objects
    Bot driver(3.0,0,0); 
    SuperVisor supervisor;
    supervisor.Run(&driver,&Comm);
    return 0;
}

bool BoundsCheck(int x,bool y)
{
    if(y)
    {
        if((x + BOT_SIZE )>480)
            return 0;
        else
            return 1;
    }
    else
    {
        if((x+ BOT_SIZE)>640) 
            return 0;
        else
            return 1;
    }
}

Mat CreateMap(Mat src,vector<Point2i> * map_point)
{ 
    Mat gray,thresholded;
    if(!src.data)
    {
        cerr<<"No Image Supplied"<<endl;
    }
    //Drawing Borders
    line(src, Point(0,0),Point(640,0),Scalar(0, 0, 0),10,8,0);
    line(src, Point(0,0),Point(0,480),Scalar(0, 0, 0),10,8,0);
    line(src, Point(0,480),Point(640,480),Scalar(0, 0, 0),10,8,0);
    line(src, Point(640,0),Point(640,480),Scalar(0, 0, 0),10,8,0);
    Size s = src.size();
    Mat map = Mat::zeros((s.height/BOT_SIZE),(s.width/BOT_SIZE),CV_64F);
    cvtColor(src,gray,COLOR_BGR2GRAY);
    threshold(gray,thresholded,127,255,0);
    erode(thresholded,thresholded,getStructuringElement(MORPH_RECT,Size(55,55)));
    int k=0;
    for(int i=0;i<s.width;i+=BOT_SIZE)
    { 
        int l=0;
        for(int j=0;j<s.height;j+=BOT_SIZE)
        {
            if(BoundsCheck(i,0) && BoundsCheck(j,1))
            {
                Rect rect1(i,j,BOT_SIZE,BOT_SIZE);
                map_point->push_back(Point2i(i,j));
                if(((sum(Mat(thresholded,rect1)))[0]/(255*BOT_SIZE*BOT_SIZE))>0.8)
                {
                    map.at<double>(l,k)=0;
                    rectangle(src,rect1,Scalar(255,255,255),4,8,0);            
                }
                else
                {
                    map.at<double>(l,k)=1;
                    rectangle(src,rect1,Scalar(0,0,0),4,8,0);
                }
            }
            l++;
        }
        k++;
    }
    return map;
}

Point3f img2map(Point temp,Mat src)
{
    if(!src.data)
    {
        cerr<<"No Image Supplied"<<endl;
    }
    Size s = src.size();
    int k=0;
    for(int i=0;i<s.width;i+=BOT_SIZE)
    { 
        int l=0;
        for(int j=0;j<s.height;j+=BOT_SIZE)
        {
            if(BoundsCheck(i,0) && BoundsCheck(j,1))
            {
                Rect rect1(i,j,BOT_SIZE,BOT_SIZE);
                if(rect1.contains(temp))
                {
                    return Point3f(k,0,l);
                }
            }
            l++;
        }
        k++;
    }
    return Point3f(0,0,0);
}

vector<Point3f> RetCenters(Mat src)
{
    Mat gray,thresholded;
    vector<vector<Point> > contours;
    vector<Point3f> circle_vector;
    vector<Point3f> rev_circle_vector;
    vector<Point> approx_curve;
    vector<Vec4i> hierarchy;
    Moments mu;
    erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(src,src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 
    dilate(src,src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 
    erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    cvtColor(src,gray,COLOR_BGR2GRAY); 

    //adaptiveThreshold(gray,thresholded,0,, int thresholdType, int blockSize, double C)
    //adaptiveThreshold(InputArray src, OutputArray dst, double maxValue, int adaptiveMethod, int thresholdType, int blockSize, double C)
    threshold(gray,thresholded,127,255,0);
    
    //adaptiveThreshold(gray,thresholded,0,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,5,3);
    //Check for Optimal Threshold range
    
    /*
    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE);  
    imshow("MyVideo",thresholded);
    waitKey(0);
    */
    //Removing Holes and other random structural anomolies
    findContours(thresholded,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0, 0));
    cout<<contours.size()<<endl;
    for(int i=0;i<contours.size();i++)
    {
        if(contourArea(contours[i])>300)
        {
            
            approxPolyDP(contours[i],approx_curve,0.01*arcLength(contours[i],0),0);
            cout<<approx_curve.size()<<endl;
            if(approx_curve.size()>11)
            {
                Point2f temp;
                float rad;
                minEnclosingCircle(contours[i],temp,rad);
                circle_vector.push_back(Point3f(temp.x,temp.y,rad));
                cout<<temp<<endl;
            }
        }
    }
    /*
     * Alternate Method
    vector<Point3f> bcircles;
    HoughCircles(thresholded, bcircles, CV_HOUGH_GRADIENT,1,10,70,15,1, 100);
    // change the last two parameters
    // (min_radius & max_radius) to detect larger circles
    //vector <Point> Plan;
    cout<<bcircles.size()<<endl;
    for(int i = 0;i < bcircles.size();i++)
    {
        Point3f b_p = bcircles[i];
        cout<<b_p<<endl;
        circle_vector.push_back(b_p);
    }
    */

    for(int i=(circle_vector.size()-1);i>=0;i--)
    {
        rev_circle_vector.push_back(circle_vector[i]);
    }
    return rev_circle_vector;
}


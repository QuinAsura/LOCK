/*
 *
 *  Created on: 01.03.2015
 *      Authors: Sibi Sankar, Sanjay Shreedharan, Adithya S
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

using namespace cv;
using namespace std;

/* Return the angle between two vectors described by the i and j component of the vector*/
double RetAngle(int,int,int,int);

/** @function main */
int main( int argc, char** argv )
{
    /// Load source image and convert it to gray
    VideoCapture cap(1);
    if (!cap.isOpened() )  // if not success, exit program
    {
        cout << "Cannot open the video file" << endl;
        return -1;
    }
    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
    int yLowH = 9;
    int yHighH = 40;
    int yLowS = 100; 
    int yHighS = 255;
    int yLowV = 100;
    int yHighV = 255;
    int gLowH = 153;
    int gHighH = 179;
    int gLowS = 218; 
    int gHighS = 255;
    int gLowV = 86;
    int gHighV = 255;


    while(1)
    {
        Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }
        
        Mat imgHSV,imgThresholded,greenthresh;
        vector<vector<Point> > contours,gcontours;
        vector<Vec4i> hierarchy,ghierarchy;
        Point2f mc(0,0),gmc(0,0);
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
                maxArea = contourArea(gcontours[i]);
                gbestcnt = i;
            }
        }

        if (!((gcontours.size()==0) || (contours.size()==0)))
        {
            mu = moments(contours[bestcnt],false);
            gmu = moments(gcontours[gbestcnt],false);
            mc= Point2f( mu.m10/mu.m00,mu.m01/mu.m00 );
            gmc = Point2f( gmu.m10/gmu.m00,gmu.m01/gmu.m00 );
            circle(frame, mc, 3, Scalar(0,0,0), -1, 8, 0 );
            circle(frame, gmc,3, Scalar(0,0,0), -1, 8, 0 );
            double angle = RetAngle(int(mc.x),int(mc.y),int(gmc.x),int(gmc.y));
            cout<<angle<<endl;
        }
        imshow("MyVideo", frame); //show the original image
        if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl; 
            break; 
        }
    }

    return 0;
}

double RetAngle(int a1,int a2,int b1,int b2)
{

    int x1 = b1-a1;
    int y1 = b2-a2;
    int x2 = b1-0;
    int y2 = b2-0;
    
    double dot = x1*x2 + y1*y2;
    double det = x1*y2 - y1*x2;
    return (atan2(det,dot) * 180.0 / 3.14);
    
}


/*
JAMESON LICENSE
This code is publicly available to anyone who wishes to use it.
If someday you have the fortune to pass by me, and you think it was useful,
I only ask that you pay me a glassh of Jameson.

Code created by Rui P. Alves
Faculty of Engeneering of University of Porto
*/


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <atomic>

// Include opencv libraries
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

// Include gstreamer library
#include <gst/gst.h>
#include <gst/app/app.h>

//include socket libs

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>

using namespace cv;
using namespace std;

#define BUFFER_LENGTH 2041 // Buffer Length
// Share frame between main loop and gstreamer callback
atomic<Mat*> atomicFrame;

//initial min and max HSV filter values.
//this values can be changed with the trackbar and hardcoded here for the color segmentation
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

//Green HSV values 
int G_H_MIN = 45;
int G_H_MAX = 61;
int G_S_MIN = 210;
int G_S_MAX = 256;
int G_V_MIN = 120;
int G_V_MAX = 256;

//Red HSV values
int R_H_MIN = 0;
int R_H_MAX = 21;
int R_S_MIN = 152;
int R_S_MAX = 256;
int R_V_MIN = 10;
int R_V_MAX = 255;

//Yellow HSV values
int Y_H_MIN = 20;
int Y_H_MAX = 50;
int Y_S_MIN = 176;
int Y_S_MAX = 256;
int Y_V_MIN = 165;
int Y_V_MAX = 256;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=3; 
//minimum and maximum object area
const int MIN_OBJECT_AREA = 8*8;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH*0.75;
Ptr<MultiTracker>multiTracker;



const string trackbarWindowName = "Trackbars";
float roll, pitch, yaw;


/*Gstreamer functions to access camera live feed*/
/**
 * @brief Check preroll to get a new frame using callback
 *  https://gstreamer.freedesktop.org/documentation/design/preroll.html
 * @return GstFlowReturn
 */
GstFlowReturn new_preroll(GstAppSink* /*appsink*/, gpointer /*data*/)
{
    return GST_FLOW_OK;
}

/**
 * @brief This is a callback that get a new frame when a preroll exist
 *
 * @param appsink
 * @return GstFlowReturn
 */
GstFlowReturn new_sample(GstAppSink *appsink, gpointer /*data*/)
{
    static int framecount = 0;

    // Get caps and frame
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    const int width = g_value_get_int(gst_structure_get_value(structure, "width"));
    const int height = g_value_get_int(gst_structure_get_value(structure, "height"));

    // Print dot every 30 frames
    if(!(framecount%30)) {
        g_print(".");
    }

    // Show caps on first frame
    if(!framecount) {
        g_print("caps: %s\n", gst_caps_to_string(caps));
    }
    framecount++;

    // Get frame data
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    // Convert gstreamer data to OpenCV Mat
    Mat* prevFrame;
    prevFrame = atomicFrame.exchange(new Mat(Size(width, height), CV_8UC3, (char*)map.data, Mat::AUTO_STEP));
    if(prevFrame) {
        delete prevFrame;
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
}



/**
 * @brief Bus callback
 *  Print important messages
 *
 * @param bus
 * @param message
 * @param data
 * @return gboolean
 */
static gboolean my_bus_callback(GstBus *bus, GstMessage *message, gpointer data)
{
    // Debug message
    //g_print("Got %s message\n", GST_MESSAGE_TYPE_NAME(message));
    switch(GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_ERROR: {
            GError *err;
            gchar *debug;

            gst_message_parse_error(message, &err, &debug);
            g_print("Error: %s\n", err->message);
            g_error_free(err);
            g_free(debug);
            break;
        }
        case GST_MESSAGE_EOS:
            /* end-of-stream */
            break;
        default:
            /* unhandled message */
            break;
    }
    /* we want to be notified again the next time there is a message
     * on the bus, so returning TRUE (FALSE means we want to stop watching
     * for messages on the bus and our callback should not be called again)
     */
    return true;
}


void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed

}
// General purpose functions
string intToString(int number) {
   std:: stringstream ss;
   ss << number;
   return ss.str();
}

string floatToString ( float number)
{
   std:: stringstream ss;
   ss << number;
   return ss.str();
}

//This function creates the trackbars for the HSV parameters
void createTrackbars(){
	//create window for trackbars


    namedWindow(trackbarWindowName,0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is         moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    cv::createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    cv::createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    cv::createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    cv::createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    cv::createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    cv::createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
}

// Define Tracker types
Ptr<Tracker> createTracker(string trackerType) {

    Ptr<Tracker> tracker;
    if (trackerType == "BOOSTING")
        tracker = TrackerBoosting::create();
    else if (trackerType == "MIL")
        tracker = TrackerMIL::create();
    else if (trackerType == "KCF")
        tracker = TrackerKCF::create();
    else if (trackerType == "TLD")
        tracker = TrackerTLD::create();
    else if (trackerType == "MEDIANFLOW")
        tracker = TrackerMedianFlow::create();
    else if (trackerType == "MOSSE")
        tracker = TrackerMOSSE::create();
    else if (trackerType == "CSRT")
        tracker = TrackerCSRT::create();
    else
        throw("not a valid tracker");

    return tracker;
}

//This fucntion draws the Objects detected in the frame
void drawObject(int x, int y, Mat &frame){

	circle(frame,Point(x,y),15,Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

}
// Morphological operations to be applied to the color segmentation
void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 5px by 5px ellipse

	Mat erodeElement = getStructuringElement( MORPH_ELLIPSE,Size(5,5));
    	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_ELLIPSE,Size(8,8));

	//morphological opening 
	erode(thresh,thresh,erodeElement);
	dilate(thresh,thresh,dilateElement);


	//morphological closing  
	dilate(thresh,thresh,dilateElement);
	erode(thresh,thresh,erodeElement);


}

// ###Function for shape detection. returns a flag indicating it found an object and a vector of bounding boxes###
// returns 0 if object  found and -1 if no object was found
int detectFilteredObject(int &x, int &y, Mat threshold, Rect2d &bbox){

	Mat temp;
	threshold.copyTo(temp);
	Rect2d aux;
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) 
		{
		int numObjects = hierarchy.size();
        	//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        	if(numObjects <= MAX_NUM_OBJECTS)
			{
			for (int index = 0; index >= 0; index = hierarchy[index][0]) 
				{
				Moments moment = moments((Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/4 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
               			if(area>MIN_OBJECT_AREA ) //&& area<MAX_OBJECT_AREA && area>refArea)
					{
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
					}
				else objectFound = false;

				}

			//let user know you found an object
			if(objectFound == true)
				{
				//rectangle dimensions for Bounding box
				double radius= sqrt(refArea/3.14);
				bbox.x = x - radius;
				bbox.y = y - radius;
				bbox.width = radius*2;
				bbox.height = bbox.width;
				return 0; 
				//returns 0 indicating it found an object
				}
			}
		}
	return -1;
}


// Main function for the detection. Runs the color detection first and then passes the result to the shape detection
// if the object was already detected and in a X frames range, the fucntion calls the trackers.
void detectObject(Mat &img, vector<Rect2d> &bbox) {

	Mat hsv_image, threshG, threshY, threshR;
	int xg, yg, xy, yy, xr, yr;
	Rect2d aux;
	cvtColor(img, hsv_image, COLOR_BGR2HSV);
	
	//Trackbars detection
	// Use this if you want to adjust the values of segmentation
    //inRange(hsv_image,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);

	// Green color detection
	inRange(hsv_image,Scalar(G_H_MIN,G_S_MIN,G_V_MIN),Scalar(G_H_MAX,G_S_MAX,G_V_MAX),threshG);
	
	//Yellow color detection 
	inRange(hsv_image,Scalar(Y_H_MIN,Y_S_MIN,Y_V_MIN),Scalar(Y_H_MAX,Y_S_MAX,Y_V_MAX),threshY);

	// Red color detection
	inRange(hsv_image,Scalar(R_H_MIN,R_S_MIN,R_V_MIN),Scalar(R_H_MAX,R_S_MAX,R_V_MAX),threshR);
	
	//apply an open followed by a closing operation
	morphOps(threshG);
	morphOps(threshR);
	morphOps(threshY);
	
	//detect the objects and draw
	// the object Bbox are stores in the aux variable. The order goes : 1-Green 2- Yellow 3- Red 
	if(detectFilteredObject(xg,yg,threshG,aux) == 0) 
		{
		drawObject(xg,yg,img);
		bbox.push_back(aux);
		}
	if(detectFilteredObject(xy,yy,threshY,aux) == 0) 
		{
		drawObject(xy,yy,img);
		bbox.push_back(aux);
		}

	if(detectFilteredObject(xr,yr,threshR,aux) == 0) 
		{
		drawObject(xr,yr,img);
		bbox.push_back(aux);
		}
}

//fucntion to start trackers
void initTracker(string trackerType, Mat frame, vector<Rect2d> bbox)  //inicia o tracker
{
	multiTracker = MultiTracker::create();
	for(int i=0; i < bbox.size(); i++)
        		multiTracker->add(createTracker(trackerType), frame, bbox[i]);
	
}


// This  fucntion does the selextion between the detection and the tracking
// Returns false if not all the 3 objects were found and true if they are
//Stores the center of the objects in imgPos

bool ObjectTracking(string trackerType, Mat frame, bool start, vector< vector <Point2f> > &imgPos)
{
	//Create Tracker
	int x, y;
	double radius;
	bool ok=false;
	vector <Rect2d> bbox;
	vector<Point2f> temp;
	// is start is true, then it Runs the Detection fucntion and after starts the Tracker
	if(start==true)
		{
		 detectObject(frame, bbox);
		 initTracker(trackerType, frame, bbox); //calls fucntion to create tracker
		}	
	// If tracker already started and no error or loss of objects has ocurred, updates the positions
	else {
		ok=multiTracker->update(frame, bbox);
		//bbox = multiTracker->getObjects(); //This line is not needed
		}
	// If the number of found obejcts is equal to 3 then the objects are drawn in the frame and the returns a flag indicating the tracking can pocceed
	if( bbox.size()== 3)
		{
		ok= true;
		for( int i=0; i<bbox.size(); i++)
			{
			radius=bbox[i].width/2;
			x= bbox[i].x + radius; //Finds the center
			y = bbox[i].y + radius;
			temp.push_back(Point2f(x,y));    //save the x,y coordinates to a vector to be later stored in .txt file
			drawObject(x,y,frame);
			}
		imgPos.push_back(temp);
		}
	else { imgPos.clear();}
	
	return ok;
}

// Main fucntion
int main(int argc, char** argv )
{
	
	//Legacy verificantion. verifies if all inputs for the code to run are given
    /*if ( argc != 3 )  
    {
        printf("usage: DisplayImage.out  <IMUdata_Path>\n");
        return -1;
    }*/

	// Opens the pipeline and starts GStreamer
    gst_init(&argc,&argv );
    gchar *descr = g_strdup(
        "udpsrc port=4200 "
        "! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 "
        "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert "
        "! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
    );
    
  // Check pipeline
    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(descr, &error);

    if(error) {
        g_print("could not construct pipeline: %s\n", error->message);
        g_error_free(error);
        exit(-1);
    }

  // Get sink
    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");

    /**
     * @brief Get sink signals and check for a preroll
     *  If preroll exists, we do have a new frame
     */
    gst_app_sink_set_emit_signals((GstAppSink*)sink, true);
    gst_app_sink_set_drop((GstAppSink*)sink, true);
    gst_app_sink_set_max_buffers((GstAppSink*)sink, 1);
    GstAppSinkCallbacks callbacks = { nullptr, new_preroll, new_sample };
    gst_app_sink_set_callbacks(GST_APP_SINK(sink), &callbacks, nullptr, nullptr);

    // Declare bus
    GstBus *bus;
    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    gst_bus_add_watch(bus, my_bus_callback, nullptr);
    gst_object_unref(bus);

    gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);

	/*Variables declaration*/
	 
    Mat threshold, K, D; // K and D are the variables for the Intrinsic parameters of the camera
    int YlRdDx, YlRdDy, MarkerCenterX, MarkerCenterY; // Variables For the Extrapolation of the coordinates //YlRdDx = YellowRedDistance (Distance from the Yellow obejct to the Red Object) 
	// MarkerCenterX= X coordinate of the center of the marker 
	float xPos, yPos, zPos, H, tanAlpha;   // xPos, yPos and zPos are the position coordinates in the vehicle referencial to the marker //H and tanalpha are values calculated for the extrapolation of the coordinates
    double focLen, cX, cY; // focLen= focal length of the camera (intrinsic parameter) // cX, cY = (X,Y) coordinates of the center of the camera image plane 
    float lambda, dReal, dIm; // lambda is the scale factor for the distance between 2 points in the camera plane (dIm) and the real distance (dReal) 	
    vector < vector<Point2f> > imgPos; // Vector of Vectors of Coordinate float Points containing the Position of the objects in the marker (1-Green , 2-Yellow, 3-Red)
    dReal=0.36;  //in meters //Real distance between yellow center and green center //needs accurate measurement
    vector<Mat> vec_channels;
    float sideA =0.20;  // Distance between center of the marker and Red object
    float CalculatedYaw, ADistCamera; //Variavles to determine the yaw between the ROV and the marker. Not tested (Commented up ahead)
    string Xr, Yr, Zr; // Strings to send to pyhton script
    //createTrackbars();  // UNCOMMENT FOR CREATING THE TRACKBARS IN HSV PARAMETERS NEED TO BE ADJUSTED
    
    // variables For Socket connection

    char server_reply[BUFFER_LENGTH];
    string positionData;
    int sendMsgSize;
    int sock = socket(AF_INET , SOCK_STREAM , 0);
    struct sockaddr_in server;
    ssize_t recsize;

    /* Connect to python server  PORT 8888 */
	
     server.sin_addr.s_addr =  inet_addr("127.0.0.1");
     server.sin_family = AF_INET; 
     server.sin_port = htons(8888);
    
    //Connect to remote server
 	 if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
  		{
     	 	 printf("connect failed. Error");
 	 	}
 
    // read camera matrix and store for calculations and undistortions
    FileStorage Parameters("Intrinsic.yml", cv::FileStorage::READ);
    Parameters["K"] >> K;
    Parameters["D"] >> D;
    focLen=K.at<double>(0,0);
    cX=K.at<double>(0,2);
    cY=K.at<double>(1,2);
    Parameters.release();
	
    Mat  cameraFeed ; // stores the frame captured by gstreamer
 
	//select type of tracker
    string trackerTypes[7] = {"KCF","MOSSE", "MEDIANFLOW", "CSRT"}; // This were the trackers with best results, but others can be added

    string trackerType=trackerTypes[2]; // Select the Tracker you wish to use

    bool first = true; // Flag indicating the first frame of detection
    int FrameCount=0;  // Counts the Frames passes since the detection begun until a limit. Used to control the tracker. after 50 frames the the program runs the detection again
    bool ok;

	/*Starts The LOOP*/
	while(1)
	{
	
	g_main_iteration(false);   /* Reads the frame from the pipeline*/
    Mat *Feed = atomicFrame.load();

	/*Checks if the Feed is empty*/
	if(Feed){ cout << "Image" << endl;
		}
	else
		{ cout << " No image" << endl;
		 continue;
		 }
	// Get IMU data from socket connection  
 	recsize=recv(sock ,(void*)server_reply , BUFFER_LENGTH , 0);
	if(recsize > 0)
			{		
				istringstream iss(server_reply);		  
				iss >> roll >> pitch >> yaw; //Passes the values from the string to the Float values
			}
	else {
		cout << "No data from IMU" << endl;
		continue;
		}
	

	//Undistort the image from the camera applying the intrinsic parameters
	undistort(atomicFrame.load()[0], cameraFeed,K, D);
	
	//Starts the Detection. If Frame is 0 the program will try to detect the objects using the Detection Algorithm
	if(FrameCount==0)
		{ok=ObjectTracking(trackerType, cameraFeed, true, imgPos);
		 FrameCount++;
		}
	// If The Objects were already detected then use the tracker
	else {
		ok=ObjectTracking(trackerType, cameraFeed, false, imgPos); //the 3rd input indicates if the fucntion uses the tracker or the detection
		FrameCount++;
	      }
	if(ok==false) //if the no 3 objetcs are detected the frame count is reset and the program tries to detect the objects again
		FrameCount=0;
	
	// If the 3 objects are found. Begin to extrapolate coordiantes
	else { 
		//Calculates the Distance between the Green Center and the Red Center 
		dIm=sqrt((imgPos.back()[0].x-imgPos.back()[2].x)*(imgPos.back()[0].x-imgPos.back()[2].x) + (imgPos.back()[0].y-imgPos.back()[2].y)*(imgPos.back()[0].y-imgPos.back()[2].y));  
		//Obtains the scale factor 
		lambda= dReal/dIm;
		YlRdDx=(imgPos.back()[1].x-imgPos.back()[2].x)/2; //X distance from Yellow to Red
		YlRdDy=(imgPos.back()[1].y-imgPos.back()[2].y)/2; //Y distance from Yellow to Red
		MarkerCenterX= imgPos.back()[1].x - YlRdDx; // Defines the Center os The Marker  from the yellow poisition.
		MarkerCenterY= imgPos.back()[1].y - YlRdDy;
		
		/* This Code is supposed to return the Yaw difference between the ROV and the Marker(Never Tested)*/
		/*ADistCamera=sqrt( (YlRdDy*YlRdDy) + (YlRdDx*YlRdDx));
		CalculatedYaw=acos((ADistCamera/sideA));
		cout << "CalculatedYaw1: " << CalculatedYaw;
		if(YlRdDx < 0)
			CalculatedYaw= CalculatedYaw + M_PI ;  //If the marker is seen fro the back, then we are at 180º*/
		yPos=lambda*(MarkerCenterX-cX)*cos(roll)*cos(yaw);   //Calculates the distance from the center of the marker to the center of the camera and slcaes it to the Real World.Applies the IMu info
		zPos=lambda*(MarkerCenterY-cY)*cos(roll)*cos(pitch);  	
		
		tanAlpha=sqrt(((imgPos.back()[2].x -MarkerCenterX)*(imgPos.back()[2].x -MarkerCenterX))+((imgPos.back()[2].y -MarkerCenterY)*(imgPos.back()[2].y -MarkerCenterY)))/focLen; //Finds tanAlpha for H measurement
		H=sideA/tanAlpha;
		xPos=H*cos(pitch)*cos(yaw) + 0.156;	 
		cout << "Zr: " << zPos;  //Couts the values obtained
		cout << " Xr: " << xPos << " Yr: " << yPos << endl;

		//Pass values to string so the can be sent to Position Controler
		Xr=floatToString(xPos);
		Yr=floatToString(yPos);
		Zr=floatToString(zPos);
		ostringstream oss(positionData);
		oss << Xr << " " << Yr << " " << Zr;
		
		positionData= oss.str();
		sendMsgSize= positionData.length();
		positionData[sendMsgSize] = '\0';
				
		send(sock , (void*)positionData.c_str() , sendMsgSize , 0 );  //send values via the socket
		cout << " data sent"<<endl; 
		}
	    
	/*Code to open windows and see the image*/
	//namedWindow("cameraFeed", WINDOW_NORMAL);
	//imshow("cameraFeed",cameraFeed); 
	
	//Used of the program is opening a window. Press Q to exit
	/*if(char(waitKey(5)) == 'q') //If Q pressed, the program shutsdown
		{
		 cout << "Q key pressed by user. Now exiting program... " << endl;
		 break;
		}*/
	if(FrameCount == 50)
		{FrameCount=0;
		/*break;*/	}
	
	memset(server_reply, 0, BUFFER_LENGTH); //clear buffer 
	}	
       
    gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    destroyAllWindows();
    //capture.release();
    Threshold.release();
    Tracking.release();
    CameraRaw.release();
    return 0;
}

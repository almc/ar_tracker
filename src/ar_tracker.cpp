#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <resource_retriever/retriever.h>
#include <ar_tracker/ARMarker.h>

#include <GL/gl.h>
#include <GL/glut.h>
#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>

// #include <stdio.h>
// #include <stdlib.h>
#include <sstream>
#include <signal.h>

// Define the function to be called when ctrl-c (SIGINT) signal is sent to process
void signal_callback_handler(int signum)
{
   printf("Caught signal %d\n",signum);
   // Cleanup and close up stuff here
   // Terminate program
   exit(signum);
}

char           *vconf         = const_cast<char*>("");
char           *cparam_name   = const_cast<char*>("data/camera_para.dat");
int             thresh        = 100;	// 100
int             count         = 0;
int             xsize, ysize;
ARParam         cparam;

#define MIN_CERTAINTY           0.5
#define NMARKERS                6
#define AR_TO_ROS               0.001

// const char     *patt_name[NMARKERS]   = {"data/robot/patt.hiro", "data/robot/patt.kanji", "data/robot/patt.sample1", "data/robot/patt.sample2"};
const char     *patt_name[NMARKERS]   = {"data/multi/patt.letter_a", "data/multi/patt.letter_b", "data/multi/patt.letter_c",
                                         "data/multi/patt.letter_d", "data/multi/patt.letter_f", "data/multi/patt.letter_g"};
int             patt_id[NMARKERS];
double          patt_width            = 80.0; // 80
double          patt_center[2]        = {0.0, 0.0};
double          patt_trans[3][4];
ARMarkerInfo    marker_data[NMARKERS];

static void   init(void);
static void   cleanup(void);
static void   keyEvent(unsigned char key, int x, int y);
static void   mainLoop(void);
static void   draw(void);

int  ros_count            = 0;
bool publishVisualMarkers = false;
bool publishTf            = true;
bool reverseTransform     = false;

ros::Publisher tracker_pub;
ros::Publisher arMarkerPub;
ros::Publisher rvizMarkerPub;
tf::TransformBroadcaster *broadcaster_ptr;
visualization_msgs::Marker rvizMarker;

int main(int argc, char **argv)
{
	// register signal and signal handler
	signal(SIGINT, signal_callback_handler);

	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
    tf::TransformBroadcaster broadcaster;
    broadcaster_ptr = &broadcaster;

	tracker_pub = n.advertise<std_msgs::String>("marker_tracker", 1000);
	arMarkerPub = n.advertise<ar_tracker::ARMarker>("ar_pose_marker", 0);
	if (publishVisualMarkers)
		rvizMarkerPub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

	glutInit(&argc, argv);
	init();

    arVideoCapStart();
    argMainLoop(NULL, keyEvent, mainLoop);

	return (0);
}

static void keyEvent(unsigned char key, int x, int y)
{
    if (key == 0x1b)			// esc
    {
        printf("*** %f (frame/sec)\n", (double)count/arUtilTimer());
        cleanup();
        exit(0);
    }
}

static void mainLoop(void)
{
    ARUint8         *dataPtr;
    ARMarkerInfo    *marker_info;
    int             marker_num;
    int             j, k;

    /* grab a video frame */
    if ((dataPtr = (ARUint8 *)arVideoGetImage()) == NULL)
    {
        arUtilSleep(2);
        return;
    }
    if (count == 0) arUtilTimerReset();
    count++;

    argDrawMode2D();
    argDispImage(dataPtr, 0,0);

    /* detect the markers in the video frame */
    if (arDetectMarker(dataPtr, thresh, &marker_info, &marker_num) < 0)
    {
        cleanup();
        exit(0);
    }

    arVideoCapNext();

    /* check for object visibility */
    k = -1;

    // printf("Found %d number of markers\n", marker_num);

    // double confidence_markers[NMARKERS];
    for (j = 0; j < NMARKERS; j++)
    {
	    marker_data[j].cf = 0.0;
	    // confidence_markers[j] = 0.0;
    }

    // fill up the NMARKERS array with data
    int i = 0;
    for (i = 0; i < NMARKERS; i++)
    {
	    for (j = 0; j < marker_num; j++)
	    {
		    if (patt_id[i] == marker_info[j].id)
		    {
			    arGetTransMat(&marker_info[j], patt_center, patt_width, patt_trans);
			    if (marker_data[i].cf < marker_info[j].cf)
			    {
				    marker_data[i] = marker_info[j];
				    // std::cout << "marker: " << marker_data[i].id
				    //           << " conf: " << marker_data[i].cf << std::endl;
			    }
			    k = 1;
			    /* if (k == -1) k = j; */
			    /* else if (marker_info[k].cf < marker_info[j].cf) k = j; */
		    }
	    }
    }
    // early exit due to no markers found
    if (k == -1)
    {
        argSwapBuffers();
        return;
    }
    // display the cubes on the scree according to the transformation

    for (i = 0; i < NMARKERS; i++)
    {
	    /* get the transformation between the marker and the real camera */
	    if (marker_data[i].cf > MIN_CERTAINTY)
	    {
		    arGetTransMat(&marker_data[i], patt_center, patt_width, patt_trans);
		    draw();
	    }
    }
    argSwapBuffers();

    // publish data in ros topic
    if (ros::ok())
    {
	    // std_msgs::String msg;
	    // std::stringstream ss;
	    for (i = 0; i < NMARKERS; i++)
	    {
		    if (marker_data[i].cf > MIN_CERTAINTY)
		    {
			    arGetTransMat(&marker_data[i], patt_center, patt_width, patt_trans);
			    // ss << "Marker ID: " << marker_data[i].id << " center: " << patt_center[0] << "," << patt_center[1]
			    //    << " width: " << patt_width << " trans(x,y,z): " << patt_trans[0][3] << "/" << patt_trans[1][3] << std::endl;
			    // std::cout << "Marker ID: " << marker_data[i].id << " center: " << patt_center[0] << "," << patt_center[1]
			    //           << " width: " << patt_width << " trans(x,y,z): " << patt_trans[0][3] << "/" << patt_trans[1][3] << std::endl;

			    double arQuat[4], arPos[3];
			    arUtilMat2QuatPos (patt_trans, arQuat, arPos);

			    double quat[4], pos[3];

			    pos[0] = arPos[0] * AR_TO_ROS;
			    pos[1] = arPos[1] * AR_TO_ROS;
			    pos[2] = arPos[2] * AR_TO_ROS;

			    quat[0] = -arQuat[0];
			    quat[1] = -arQuat[1];
			    quat[2] = -arQuat[2];
			    quat[3] = arQuat[3];


			    ROS_INFO(" QUAT: Pos x: %3.5f  y: %3.5f  z: %3.5f", pos[0], pos[1], pos[2]);
			    ROS_INFO("     Quat qx: %3.5f qy: %3.5f qz: %3.5f qw: %3.5f", quat[0], quat[1], quat[2], quat[3]);

			    // **** publish the marker --------------------
			    ar_tracker::ARMarker ar_pose_marker;
			    ar_pose_marker.header.frame_id = i;                // image_msg->header.frame_id;
			    ar_pose_marker.header.stamp    = ros::Time::now(); // image_msg->header.stamp;
			    ar_pose_marker.id              = marker_data[i].id;

			    ar_pose_marker.pose.pose.position.x = pos[0];
			    ar_pose_marker.pose.pose.position.y = pos[1];
			    ar_pose_marker.pose.pose.position.z = pos[2];

			    ar_pose_marker.pose.pose.orientation.x = quat[0];
			    ar_pose_marker.pose.pose.orientation.y = quat[1];
			    ar_pose_marker.pose.pose.orientation.z = quat[2];
			    ar_pose_marker.pose.pose.orientation.w = quat[3];

			    ar_pose_marker.confidence = marker_data[i].cf;

			    arMarkerPub.publish(ar_pose_marker);
			    ROS_INFO ("Published ar_single marker");
			    // **** publish the marker --------------------


			    // **** publish transform between camera and marker --------------------
			    tf::Quaternion rotation (quat[0], quat[1], quat[2], quat[3]);
			    tf::Vector3 origin (pos[0], pos[1], pos[2]);
			    tf::Transform t (rotation, origin);
			    if (publishTf)
			    {
				    if (reverseTransform)
				    {
					    tf::StampedTransform markerToCam (t.inverse(), ros::Time::now(), patt_name[i], patt_name[i]); //image_msg->header.frame_id);
					    broadcaster_ptr->sendTransform(markerToCam);
				    } else {
					    std::string name("world_coordinate_frame");
					    tf::StampedTransform camToMarker (t          , ros::Time::now(), name.c_str(), patt_name[i]); // (frame_id: child_frame_id: patt.letter_)
					    broadcaster_ptr->sendTransform(camToMarker);
				    }
			    }
			    // **** publish transform between camera and marker --------------------


			    // **** publish visual marker --------------------
			    if (publishVisualMarkers)
			    {
				    tf::Vector3 markerOrigin (0, 0, 0.25 * patt_width * AR_TO_ROS);
				    tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
				    tf::Transform markerPose = t * m;           // marker pose in the camera frame

				    tf::poseTFToMsg(markerPose, rvizMarker.pose);

				    rvizMarker.header.frame_id = patt_name[i];  // image_msg->header.frame_id;
				    rvizMarker.header.stamp = ros::Time::now(); // image_msg->header.stamp;
				    rvizMarker.id = 1;

				    rvizMarker.scale.x = 1.0 * patt_width * AR_TO_ROS;
				    rvizMarker.scale.y = 1.0 * patt_width * AR_TO_ROS;
				    rvizMarker.scale.z = 0.5 * patt_width * AR_TO_ROS;
				    rvizMarker.ns = "basic_shapes";
				    rvizMarker.type = visualization_msgs::Marker::CUBE;
				    rvizMarker.action = visualization_msgs::Marker::ADD;
				    rvizMarker.color.r = 0.0f;
				    rvizMarker.color.g = 1.0f;
				    rvizMarker.color.b = 0.0f;
				    rvizMarker.color.a = 1.0;
				    rvizMarker.lifetime = ros::Duration(1.0);

				    // rvizMarkerPub.publish(rvizMarker);
				    ROS_INFO ("Published visual marker");
			    }
			    // **** publish visual marker --------------------


			    // msg.data = ss.str();
			    // ROS_INFO("%s", msg.data.c_str());
			    // tracker_pub.publish(msg);
			    // ros::spinOnce();
			    ++ros_count;
		    }
	    }
    }
}

static void init(void)
{
    ARParam  wparam;

    /* open the video path */
    if (arVideoOpen(vconf) < 0) exit(0);
    /* find the size of the window */
    if (arVideoInqSize(&xsize, &ysize) < 0) exit(0);
    printf("Image size (x,y) = (%d,%d)\n", xsize, ysize);

    /* set the initial camera parameters */
    if (arParamLoad(cparam_name, 1, &wparam) < 0)
    {
        printf("Camera parameter load error !!\n");
        exit(0);
    }
    arParamChangeSize(&wparam, xsize, ysize, &cparam);
    arInitCparam(&cparam);
    printf("*** Camera Parameter ***\n");
    arParamDisp(&cparam);

    int i = 0;
    for (i = 0; i < NMARKERS; i++)
    {
	    if ((patt_id[i]=arLoadPatt(patt_name[i])) < 0)
	    {
		    printf("pattern load error !!\n");
		    exit(0);
	    }
    }

    /* open the graphics window */
    argInit(&cparam, 1.0, 0, 0, 0, 0);
}

/* cleanup function called when program exits */
static void cleanup(void)
{
    arVideoCapStop();
    arVideoClose();
    argCleanup();
}

static void draw(void)
{
    double    gl_para[16];
    GLfloat   mat_ambient[]     = {0.0, 0.0, 1.0, 1.0};
    GLfloat   mat_flash[]       = {0.0, 0.0, 1.0, 1.0};
    GLfloat   mat_flash_shiny[] = {50.0};
    GLfloat   light_position[]  = {100.0,-200.0,200.0,0.0};
    GLfloat   ambi[]            = {0.1, 0.1, 0.1, 0.1};
    GLfloat   lightZeroColor[]  = {0.9, 0.9, 0.9, 0.1};

    argDrawMode3D();
    argDraw3dCamera(0, 0);
    glClearDepth(1.0);
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    /* load the camera transformation matrix */
    argConvGlpara(patt_trans, gl_para);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(gl_para);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambi);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_flash);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_flash_shiny);
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMatrixMode(GL_MODELVIEW);
    glTranslatef(0.0, 0.0, 25.0);
    glutSolidCube(50.0);
    glDisable(GL_LIGHTING);

    glDisable(GL_DEPTH_TEST);
}


// typedef struct {
// 	int     area;
// 	int     id;
// 	int     dir;
// 	double  cf;
// 	double  pos[2];
// 	double  line[4][3];
// 	double  vertex[4][2];
// } ARMarkerInfo;

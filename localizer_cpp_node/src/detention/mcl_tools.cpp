/*
 *
 *		mcl_tools.cpp
 *
 */ 

/* Mulithreading */
#include <pthread.h>

/* C++ */
#include <iostream>
#include <string>
#include <unistd.h>
using namespace std;

/* OpenCV */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

/* OpenGl*/
#include <GL/glut.h>
#include "util.h"
#include "mcl_tools.h"

#include "raycaster.h"
IplImage *image;
raycaster rayImage;

const string imagePath("./src/tom_2d_pf/hallway.png");
extern vector<particle> particles;
//============== Internal Data ========================================//
particle real_pose(0,0,0,0), best_pose(0,0,0,0);

void randomizeSomeParticles(void);
void moveForward(geometry_msgs::Twist *U);
void moveCCW(geometry_msgs::Twist *U);
void moveCW(geometry_msgs::Twist *U);
#if 0
void policy(geometry_msgs::Twist *U, sensor_msgs::LaserScan *Z)
{
  static u32_t sequence = 0;

  if(sequence++ < FW_TIME )
    moveForward(U);
  else if (sequence < TOTAL_TIME) 
    moveCCW(U);
  else
    sequence=0;
}
#endif
void policy(geometry_msgs::Twist *U, sensor_msgs::LaserScan *Z)
{
  static u32_t resample_counter = 50;
  /* If we're getting close to the right wall */
  if(!Z->ranges.empty() && (Z->ranges[0] < 1.0f || Z->ranges[1] < 1.5f))  
    moveCCW(U);
  /* If we're getting close to the left wall */
  else if(!Z->ranges.empty() && (Z->ranges[4] < 1.0f || Z->ranges[3] < 1.5f) )
    moveCW(U);
  else
    moveForward(U);
}

void moveForward(geometry_msgs::Twist *U)
{
  U->linear.x = 1.0f;
  U->linear.y = 0.0f;
  U->angular.z = 0.0f;
}
void moveCCW(geometry_msgs::Twist *U)
{
  U->linear.x = 0.0f;
  U->linear.y = 0.0f;
  U->angular.z = 1.0f;
}
void moveCW(geometry_msgs::Twist *U)
{
  U->linear.x = 0.0f;
  U->linear.y = 0.0f;
  U->angular.z = -1.0f;
}

void cleanBadParticles(void)
{
	vector<particle>::iterator it;
	u32_t p = 0;

	for(it = particles.begin(); it != particles.end(); it++, p++)
	{
		it->w = 0;	/* Also clear weights */
		while(map_hit(it->x, it->y))
		{
			particles[p] = particle();
		}
	}

	randomizeSomeParticles();
}

void randomizeSomeParticles(void)
{
	static u32_t counter = 200;
	if(counter-- == 0)
	{	
		vector<particle>::iterator it;
		u32_t p = 0;
		for(it = particles.begin(); it != particles.begin()+50; it++, p++)
		{
			particles[p] = particle();

			while(map_hit(it->x, it->y))
			{
				particles[p] = particle();
			}
		}
		counter = 200;
	}
}
/* Particl Filter Tools */
vector<particle> resample( vector<particle> oldParticles, vector<f64_t> cdf_arr)
{
   f64_t randVal;
   vector<particle> newParticles(oldParticles);
   vector<particle>::iterator oldP_it, newP_it;
   vector<f64_t>::iterator cdf_it;
   
   /* Re-sample every particle */
   for(newP_it = newParticles.begin(); newP_it != newParticles.end(); newP_it++)
   {
      randVal = ((f64_t)(rand()%1000))/1000.0;
      /* Linear search CDF array for new particle */
      for(cdf_it = cdf_arr.begin(), oldP_it = oldParticles.begin(); cdf_it != cdf_arr.end(); )
      {
         if(randVal > *cdf_it)
         {
            cdf_it++; oldP_it++;
         }
         else
         {
            *newP_it = *oldP_it;
            break;
         }
      }
   }

   return newParticles;
};

vector<f64_t> norm_cdf_vector(vector<particle> arr)
{
   f64_t sum;
   vector<particle>::iterator in_it;   /* Double iterator*/
   vector<f64_t> cdf_arr;

   /* Normalize Vector */
   for(sum = 0.0, in_it = arr.begin(); in_it != arr.end(); in_it++)
      sum += in_it->w;

   for(in_it = arr.begin(); in_it < arr.end(); in_it++)
      in_it->w /=sum;

   /* Create CDF Vector */
   for(sum=0.0, in_it = arr.begin(); in_it < arr.end(); in_it++)
   {
      sum+= in_it->w;
      cdf_arr.push_back(sum);
   }
   return cdf_arr;
}

/* 
 * 	Conversion Operations
 *    World <---> Image 
 *		   Tools
 */
inline u32_t clamp(u32_t x, u32_t lower, u32_t upper)
{
	return max(lower, min(x, upper));
}
inline u32_t wx_to_px(f32_t wx)
{
	return clamp((wx/MAP_WIDTH) * (image->width), 0, image->width - 1u);
}
inline u32_t wy_to_py(f32_t wy)
{
	return clamp((wy/MAP_HEIGHT) * (image->height), 0, image->height - 1u);
}
inline f32_t px_to_wx(u32_t px)
{
	return ((f32_t)px)/(image->width) * MAP_WIDTH;
}
inline f32_t py_to_wy(u32_t py)
{
	return ((image->height) - (f32_t)py)/(image->height) * MAP_HEIGHT;
}
inline f64_t wt_to_pt(f64_t pt)
{
	f64_t x = cos(pt); 	f64_t y = sin(pt);
	return atan2(-y, x);
}

/* 
 * 		
 *  Raycasting 
 *
 */
u16_t map_hit(f64_t wx, f64_t wy)
{
	s32_t px = wx_to_px(wx);
	s32_t py = wy_to_py(wy);

	u32_t wall = raycaster_test_pixel(&rayImage, px, py);
	return (wall == 1);
}

f64_t map_range(Pos_t particle, f64_t phi)
{
	u32_t px = wx_to_px(particle.x);
	u32_t py = wy_to_py(particle.y);
	f64_t pt = wt_to_pt(particle.t + phi);
	return raycaster_cast(&rayImage, px, py, pt);
}


/* 
 * 		OpenGL
 *  Visual Utilities 
 *
 */
/* To do, add particle iterator */
void gl_idle(void)
{    
	glutPostRedisplay();
    //time.sleep(0.05)
    sleep(0.05);
}
void gl_display(void)
{
	u32_t i;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Draw background.
    glEnable( GL_TEXTURE_2D );
    glColor3f(0.5f, 0.5f, 0.5f);

    glBegin(GL_QUADS);
    glTexCoord2f(0, 1); glVertex2f(0, 1);
    glTexCoord2f(0, 0);	glVertex2f(0, 0);
    glTexCoord2f(1, 0);	glVertex2f(1, 0);
    glTexCoord2f(1, 1);	glVertex2f(1, 1);
    glEnd();

    // Draw particles.
    glDisable( GL_TEXTURE_2D );
    glColor3f(0.0, 1.0, 0.0);
    glPointSize(1.0);
    glBegin(GL_POINTS);
    for(i=0; i<NUM_PARTICLES; i++)
    	glVertex2f(particles.at(i).x/MAP_WIDTH, particles.at(i).y / MAP_HEIGHT);
    glEnd();

    // Draw poses
    glPointSize(5.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex2f(real_pose.x / MAP_WIDTH, real_pose.y / MAP_HEIGHT);
    glColor3f(0.0f, 0.3f, 1.0f);
    glVertex2f(best_pose.x / MAP_WIDTH, best_pose.y / MAP_HEIGHT);
    glEnd();

    glutSwapBuffers();
}
void gl_click(int button, int state, int x, int y)
{
	if(button != 0 || state != 1)	return;

	f32_t wx = px_to_wx(x);
	f32_t wy = py_to_wy(y);

	cout << "Click at world= " << wx << ", " << wy << "; image = " << x << ", " << y/* <<"; Hit =  << map_hit(wx,wy)*/ <<endl;
}
void initMclTools(int argc, char **argv)
{
		  /* Get Image */
	image = cvLoadImage(imagePath.c_str(), CV_LOAD_IMAGE_COLOR);
	
	if(image == NULL) cout << "Cannot open image " << endl;

	rayImage.image = image;
	//cout << "width: " << image->width << "height: " << image->height << endl;
	cvFlip(image, image, 0);
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(image->width, image->height);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Iancovici's 2D Particle Filter");

	gluOrtho2D(0.0, 1.0, 0.0, 1.0);

	glEnable( GL_POINT_SMOOTH );
	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	glPointSize( 1.0 );

	glClearColor ( 0, 0, 0, 0 );
	glShadeModel( GL_SMOOTH );
	glDisable( GL_LIGHTING );
	//glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	//glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, image->widthStep/3);
	int align=1;
	for (int s=image->widthStep; (align<8) && !(s&1); s>>=1)
    	align<<=1;
	glPixelStorei(GL_UNPACK_ALIGNMENT, align);
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
  	glTexImage2D( GL_TEXTURE_2D, 0, 3, image->width, image->height, 0, GL_RGB, GL_UNSIGNED_BYTE, image->imageData);
    //            map_img.tostring("raw", "RGB", 0, -1));
  	glEnable( GL_TEXTURE_2D );

  	glutDisplayFunc(&gl_display);
  	glutIdleFunc(&gl_idle);
	glutMouseFunc(&gl_click);
}

void *glutTh(void* )
{
	glutMainLoop();
}

void mcl_run_viz(void)
{

	pthread_t glutThId;
	//const char* thread_id = "Glut Thread";
	pthread_create(&glutThId, NULL, glutTh, NULL);
}
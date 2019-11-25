/*
 * ROS
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>

/*
 * EXTERN
 */
extern "C" {
#include "sdbackground_wrapper.h"
#include "extern/def.h"
#include "extern/nrutil.h"
}

int gAmpliFactor = 1;

image_transport::Publisher pub;

// Image variables
uint i0, i1,h0, h1;
uint width, height,bord;

// Image variables
unsigned char **I, **M, **V, **E, **D;

void imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    //For syntax keeping
    int N = gAmpliFactor;

    // created shared pointer Image with copy
    sensor_msgs::Image::Ptr new_image =
            boost::make_shared<sensor_msgs::Image>();

    width   = uint(image->width);
    height  = uint(image->height);

    i0 = 0; i1 = height - 1;
    h0 = 0;
    h1 = width - 1;

    byte v_min = 2;
    byte v_max = 255;
    //int p_m = 1; // update period of the background model
    //int p_v = 1; // update period of the background variance

    if(I == nullptr)
    {
        I = bmatrix(i0-bord, i1+bord, h0-bord, h1+bord); // absolute difference
        D = bmatrix(i0-bord, i1+bord, h0-bord, h1+bord); // absolute difference
        M = bmatrix(i0-bord, i1+bord, h0-bord, h1+bord); // current SD-background
        V = bmatrix(i0-bord, i1+bord, h0-bord, h1+bord); // current SD-variance
        E = bmatrix(i0-bord, i1+bord, h0-bord, h1+bord); // current SD-variance
    }

    //Copy of the image into buffer I
    for(uint i=0;i<width;i++)
    {
        for(uint j=0;j<height;j++)
        {
            I[i][j]=byte(image->data[i*width+j]);
        }
    }

    // for time measure
    long size;
    double  t1, t2, dt;

    // Here the callback use the external code of Antoine
    size = (h1-h0+1)*(i1-i0+1);
    CHRONO(routine_UpdateM(I,i0,i1,h0,h1,M), "MAJ Moyenne");
    CHRONO(routine_AbsoluteDifference(I,M,i0,i1,h0,h1,D), "Calcul Difference");
    CHRONO(routine_UpdateV(D,i0,i1,h0,h1,N,v_min,v_max,V), "MAJ Variance");
    CHRONO(routine_ComputeFore(D,V,i0,i1,h0,h1,E), "Calcul Label");

    //Copy from the buffer E to new image
    for(uint i=0;i<width;i++)
    {
        for(uint j=0;j<height;j++)
        {
            new_image->data[i*width+j] = E[i][j];
        }
    }

    //Publish
    pub.publish(new_image);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sdbackground");
    ros::NodeHandle nh;

    int rate;

    nh.param("amplification factor", gAmpliFactor, 1);
    nh.param("rate", rate, int(40));

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("in_image", 1, imageCallback);
    pub = it.advertise("out_image", 1);

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);

    // Main loop.
    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    free_bmatrix(I, i0-bord, h1+bord, h0-bord);
    free_bmatrix(M, i0-bord, h1+bord, h0-bord);
    free_bmatrix(V, i0-bord, h1+bord, h0-bord);
    free_bmatrix(E, i0-bord, h1+bord, h0-bord);
    free_bmatrix(D, i0-bord, h1+bord, h0-bord);

    return 0;
}

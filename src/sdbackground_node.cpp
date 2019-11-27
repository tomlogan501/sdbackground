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

//Debug
#include <iostream>     // std::cout, std::end

static int gAmpliFactor = 1;

static image_transport::Publisher pub;

// Image variables
static int width, height;
static const byte v_min = 2;
static const byte v_max = 125;

// Image variables
static byte **I, **M, **V, **E, **D;
static sensor_msgs::Image::Ptr pubImage;

void imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    if(image->encoding == "mono8")
    {
        //For syntax keeping
        int N = gAmpliFactor;

        // copy image properties
        pubImage->header       = image->header;
        pubImage->height       = image->height ;
        pubImage->width        = image->width ;
        pubImage->encoding     = image->encoding;
        pubImage->is_bigendian = image->is_bigendian;
        pubImage->step         = image->step;

        pubImage->data.resize(pubImage->width * pubImage->height);

        //Copy of the image into buffer I
        for(uint i=0;i<uint(height);i++)
        {
            for(uint j=0;j<uint(width);j++)
            {
                I[i][j]=byte(image->data[i*uint(width)+j]);
            }
        }

        // Here the callback use the external code of Antoine
        routine_AbsoluteDifference(I,M,0, height, 0, width,D);
        routine_UpdateV(D,0, height, 0, width,N,v_min,v_max,V);
        routine_ComputeFore(D,V,0, height, 0, width,E);

        //Copy from the buffer E to new image
        uint new_index = 0;
        for(uint i=0;i<uint(height);i++)
        {
            for(uint j=0;j<uint(width);j++)
            {
                new_index = i*uint(width)+j;
                pubImage->data[new_index] = E[i][j];
                //FOR TESTING PURPOSE //
                //pubImage->data[new_index] = i*255/height;
            }
        }
        routine_UpdateM(I,0, height, 0, width,M);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sdbackground");
    ros::NodeHandle nh;

    int rate;

    nh.param("amplification_factor", gAmpliFactor, 10);
    nh.param("rate", rate, int(10));
    nh.param("width", width, int(640));
    nh.param("height", height, int(480));

    I = bmatrix(0, height, 0, width); // absolute difference
    D = bmatrix(0, height, 0, width); // absolute difference
    M = bmatrix(0, height, 0, width); // current SD-background
    V = bmatrix(0, height, 0, width); // current SD-variance
    E = bmatrix(0, height, 0, width); // current SD-variance

    Routine_Put_Initial_Value(0,height,0,width,v_min,V);
    Routine_Put_Initial_Value(0,height,0,width,v_min,M);

    // created shared pointer Image
    pubImage = boost::make_shared<sensor_msgs::Image>();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("in_image", 1, imageCallback);
    pub = it.advertise("out_image", 1);

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);

    // Main loop.
    while (nh.ok())
    {
        ros::spinOnce();
        //Publish
        pub.publish(pubImage);
        r.sleep();
    }

    free_bmatrix(I, 0, width, 0);
    free_bmatrix(M, 0, width, 0);
    free_bmatrix(V, 0, width, 0);
    free_bmatrix(E, 0, width, 0);
    free_bmatrix(D, 0, width, 0);

    return 0;
}

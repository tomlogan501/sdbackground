// Copyright [2019] <Thomas SIMON>

/*
 * ROS
 */
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sdbackground/sdbgConfig.h>

/*
 * EXTERN
 */
extern "C"
{
#include <sdbackground_wrapper.h>
}

// Global parameters for dynamic use
static int gAmpliFactor = 10;
static int gRateFreq = 30;
// Output select
static int gSelectedMatrix = 4;

static image_transport::Publisher pub;

// Image variables
static int width, height;
static const byte v_min = 2;
static const byte v_max = 255;

// Image variables
static byte **I, **M, **V, **E, **D;
static sensor_msgs::Image::Ptr pubImage;


///
/// \brief callback : Callback when parameters are changed with the gui
/// \param config : the structure config define in the cfg file
/// \param level : N/A
///
void callback(sdbackground::sdbgConfig &config, uint32_t level)
{
  //  ROS_INFO("Reconfigure Request: %d: Facteur d'amplification, %d: Frequence
  //  de traitement (Hz)",
  //            config.ampli_factor, config.ana_rate);
  gAmpliFactor = config.ampli_factor;
  gRateFreq = config.ana_rate;
  gSelectedMatrix = config.selected_matrix;
}

///
/// \brief imageCallback : Callback to treat the image and call the code through
/// the wrapper \param image : the image send
///
void imageCallback(const sensor_msgs::ImageConstPtr &image)
{
  // Only mono for this package
  if (image->encoding == "mono8")
  {
    // For syntax keeping
    int N = gAmpliFactor;

    // copy image properties
    pubImage->header = image->header;
    pubImage->height = image->height;
    pubImage->width = image->width;
    pubImage->encoding = image->encoding;
    pubImage->is_bigendian = image->is_bigendian;
    pubImage->step = image->step;

    pubImage->data.resize(pubImage->width * pubImage->height);

    // Copy of the image into buffer I
    for (uint i = 0; i < uint(height); i++)
    {
      for (uint j = 0; j < uint(width); j++)
      {
        I[i][j] = byte(image->data[i * uint(width) + j]);
      }
    }

    // Here the callback use the external code of Antoine
    routine_AbsoluteDifference(I, M, 0, height, 0, width, D);
    routine_UpdateV(D, 0, height, 0, width, N, v_min, v_max, V);
    routine_ComputeFore(D, V, 0, height, 0, width, E);

    // Copy from the buffer E to new image
    uint new_index = 0;
    for (uint i = 0; i < uint(height); i++)
    {
      for (uint j = 0; j < uint(width); j++)
      {
        new_index = i * uint(width) + j;

        switch (gSelectedMatrix)
        {
        case 1:
          pubImage->data[new_index] = M[i][j];
          break;
        case 2:
          pubImage->data[new_index] = D[i][j];
          break;
        case 3:
          pubImage->data[new_index] = V[i][j];
          break;
        case 4:
          pubImage->data[new_index] = E[i][j];
          break;
        default:
          pubImage->data[new_index] = E[i][j];
          break;
        }
        // FOR TESTING PURPOSE //
        // pubImage->data[new_index] = i*255/height;
      }
    }
    routine_UpdateM(I, 0, height, 0, width, M);
  }
  else
  {
    ROS_INFO("Only mono8 image");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sdbackground");
  ros::NodeHandle nh;

  nh.param("amplification_factor", gAmpliFactor, 10);
  nh.param("rate", gRateFreq, static_cast<int>(30));
  nh.param("width", width, static_cast<int>(640));
  nh.param("height", height, static_cast<int>(480));

  nh.param("selected_matrix", gSelectedMatrix, static_cast<int>(4));

  // Dynamic parameters initilisation
  dynamic_reconfigure::Server<sdbackground::sdbgConfig> server;
  dynamic_reconfigure::Server<sdbackground::sdbgConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  I = bmatrix(0, height, 0, width);  // absolute difference
  D = bmatrix(0, height, 0, width);  // absolute difference
  M = bmatrix(0, height, 0, width);  // current SD-background
  V = bmatrix(0, height, 0, width);  // current SD-variance
  E = bmatrix(0, height, 0, width);  // current SD-variance

  Routine_Put_Initial_Value(0, height, 0, width, v_min, V);
  Routine_Put_Initial_Value(0, height, 0, width, v_min, M);

  // created shared pointer Image
  pubImage = boost::make_shared<sensor_msgs::Image>();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("in_image", 1, imageCallback);
  pub = it.advertise("out_image", 1);

  // Main loop.
  while (nh.ok())
  {
    // Tell ROS how fast to run this node.
    ros::Rate Rate(gRateFreq);
    ros::spinOnce();
    // Publish
    pub.publish(pubImage);
    Rate.sleep();
  }

  free_bmatrix(I, 0, width, 0);
  free_bmatrix(M, 0, width, 0);
  free_bmatrix(V, 0, width, 0);
  free_bmatrix(E, 0, width, 0);
  free_bmatrix(D, 0, width, 0);

  return 0;
}

Evaluation program for Sigma-Delta background subtraction

Provided as-is, without warranty of any kind

Please cite the following papers if you use this code:
- Basics Sigma-Delta (SD_basics.c, prog1):
Antoine Manzanera & Julien C. Richefeu 
A robust and computationally efficient motion detection 
algorithm based on Sigma-Delta background estimation. 
Indian Conference on Computer Vision, Graphics and Image 
Processing (ICVGIP'04). Kolkata - India. dec.2004. pp. 46-51.
- Zipfian Sigma-Delta (SD_zipf.c, prog2):
Antoine Manzanera
Sigma-Delta Background Subtraction and the Zipf Law. 
Progress in Pattern Recognition, Image Analysis and 
Applications (CIARP'07). Viña del Mar-Valparaíso, Chile. 
nov.2007. pp. 42-51.  

You have to adjust the input parameters to your own data:
(main functions)
// Image sequence parameters
  src_path = "../car3/";
  filename = "car_3";
  dst_path = "./SD1/"; 
  ndigit   = 3;
  width    = 320;
  height   = 240;

You may also want to adjust the tunable parameters:
- Defaut values, Basic version:
  int N = 1; // Number of deviation for the label decision criterion
  byte v_min = 2;
  byte v_max = 255; 
  int p_m = 1; // update period of the background model
  int p_v = 1; // update period of the background variance
- Defaut values, Zipfian version:
  float max_fg_rate = 0.5; // Maximal rate of foreground points (N increment condition)
  float max_fg_isolated_rate = 0.01; // Maximal rate of isolated foreground points (N increment condition)
  byte v_min = 2;
  byte v_max = 255; 
  int p_v = 8; // update period of the background variance

For more information just look at the comments in the C code.

Antoine Manzanera

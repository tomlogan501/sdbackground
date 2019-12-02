#include <gtest/gtest.h>
#include <string>
#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;

//Link to extern
extern "C"
{

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include "sdbackground_wrapper_test.h"

static float Freq;

/* ------------------ */
long long readTSC (void)
/* ------------------ */
{
    long long t;
    asm volatile (".byte 0x0f,0x31" : "=A" (t));

    return t;
}
/* ------------ */
double dtime(void)
/* ------------ */
{
    return double(readTSC());
}
/* ----------------- */
void setFreq(float f)
/* --------------- */
{
    Freq = f;
#ifdef DEBUG
    printf("Freq = %.0f MHz\n", Freq/1e6);
#endif
}
/* ---------------------------------------- */
void printCPP(char *str, double dt,  int size)
/* ---------------------------------------- */
{
    float cpp, ms, df;

    df = (float) dt;
    cpp = (float) dt / (float) size;
    ms = 1000.0f * (float) dt / (float) Freq;

#ifdef DEBUG
    printf("%20s : ", str);
    printf("cycles = %10.0f ;", df);
    printf("cpp = %6.2f c/p ;", cpp);
    printf("t = %6.2f ms\n", ms);
#endif
}
/* --------------------------------------------------------------------------------------------------------------------------------- */
void generate_path_filename_k_ndigit_extension(char *path, char *filename, int k, int ndigit, char *extension, char *complete_filename)
/* --------------------------------------------------------------------------------------------------------------------------------- */
{
    char *format = "%s%s%d.%s";

    switch(ndigit) {
    case 0 : format = "%s%s%d.%s";   break;
    case 1 : format = "%s%s%01d.%s"; break;
    case 2 : format = "%s%s%02d.%s"; break;
    case 3 : format = "%s%s%03d.%s"; break;
    case 4 : format = "%s%s%04d.%s"; break;
    case 5 : format = "%s%s%05d.%s"; break;
    case 6 : format = "%s%s%06d.%s"; break;
    case 7 : format = "%s%s%07d.%s"; break;
    case 8 : format = "%s%s%08d.%s"; break;
    case 9 : format = "%s%s%09d.%s"; break;
    }
    sprintf(complete_filename, format, path, filename, k, extension);
}

byte sign(byte b0, byte b1) {
    if (b0<b1) return (byte)-1;
    else if (b0>b1) return 1;
    else return 0;
}

#ifdef DEBUG
#define CHRONO(X, str) t1=dtime(); X; t2 = dtime(); dt = t2-t1; printCPP(str, dt, size);
#else
#define CHRONO(X, str) t1=dtime(); X; t2 = dtime(); dt = t2-t1;;
#endif

///
/// \brief generatePictures : generate the set of image to be compared to ground truth
/// \param src_path : source path
/// \param dst_path : destination path
/// \todo  add other arguments ?
///
void generatePictures(const char *src_path,const char *dst_path)
{
    setFreq(2.4e9);
    char *filename;

    char cfilenameI[1024];
    char cfilenameO[1024];

    int i0, i1, j0, j1;
    int width, height,bord;
    int ndigit;

    // Image variables
    byte **I, **M, **V, **E, **D;
    int t, tstart, tstop, tstep;

    // Image sequence parameters
    filename = "car_3";
    ndigit   = 3;
    width    = 320;
    height   = 240;
    // No spatial processing in this version
    bord     = 0;

    // Tunable parameters
    int N = 10; // Number of deviation for the label decision criterion
    byte v_min = 2;
    byte v_max = 255;
    int p_m = 1; // update period of the background model
    int p_v = 1; // update period of the background variance

    // for time measure
    long size;
    double  t1, t2, dt;

#ifdef DEBUG
    printf("src_path = %s\n", src_path);
    printf("filename = %s\n", filename);
    printf("dst_path = %s\n", dst_path);
#endif

    i0 = 0; i1 = height;
    j0 = 0; j1 = width;

    // images are allocated outside the loop
    // do not allocate inside
    // malloc with borders to tackle border problem
    I = bmatrix(0, height, 0, width); // absolute difference
    D = bmatrix(0, height, 0, width); // absolute difference
    M = bmatrix(0, height, 0, width); // current SD-background
    V = bmatrix(0, height, 0, width); // current SD-variance
    E = bmatrix(0, height, 0, width); // current SD-variance

    Routine_Put_Initial_Value(0,height,0,width,v_min,I);
    Routine_Put_Initial_Value(0,height,0,width,v_min,D);
    Routine_Put_Initial_Value(0,height,0,width,v_min,M);
    Routine_Put_Initial_Value(0,height,0,width,v_min,V);
    Routine_Put_Initial_Value(0,height,0,width,v_min,E);

    tstart = 0;
    tstop  = 199;
    tstep  = 1;

    generate_path_filename_k_ndigit_extension((char*)src_path, filename, tstart, ndigit, "pgm", cfilenameI);

#ifdef DEBUG
    puts(cfilenameI);
#endif

    // SD parameters initialization
    MLoadPGM_bmatrix (cfilenameI, i0, i1, j0, j1, M); // Multiple-Load without allocating matrix
    Routine_Put_Initial_Value(i0, i1, j0, j1, v_min, V);

    for(t=tstart+1; t<=tstop; t+=tstep) {
        generate_path_filename_k_ndigit_extension((char*)src_path, filename, t, ndigit, "pgm", cfilenameI); // generate filename of current image
        MLoadPGM_bmatrix (cfilenameI, i0, i1, j0, j1, I);  // load current image

        // routine of computation
        if (t<tstop) {
            if (t % p_m == 0) routine_UpdateM(I,i0,i1,j0,j1,M);
            routine_AbsoluteDifference(I,M,i0,i1,j0,j1,D);
            if (t % p_v == 0) routine_UpdateV(D,i0,i1,j0,j1,N,v_min,v_max,V);
            routine_ComputeFore(D,V,i0,i1,j0,j1,E);
        } else {
            size = (j1-j0+1)*(i1-i0+1);
            CHRONO(routine_UpdateM(I,i0,i1,j0,j1,M), "MAJ Moyenne");
            CHRONO(routine_AbsoluteDifference(I,M,i0,i1,j0,j1,D), "Calcul Difference");
            CHRONO(routine_UpdateV(D,i0,i1,j0,j1,N,v_min,v_max,V), "MAJ Variance");
            CHRONO(routine_ComputeFore(D,V,i0,i1,j0,j1,E), "Calcul Label");
        }

        // Put here the results you wish to save:
        // generate filename for output
        generate_path_filename_k_ndigit_extension((char*)dst_path, "E_", t, ndigit, "pgm", cfilenameO);
        // write your stuff
        SavePGM_bmatrix(E, i0, i1, j0, j1, cfilenameO);
    }

    free_bmatrix(I, 0, width, 0);
    free_bmatrix(M, 0, width, 0);
    free_bmatrix(V, 0, width, 0);
    free_bmatrix(E, 0, width, 0);
    free_bmatrix(D, 0, width, 0);
}

} //Extern C

std::size_t number_of_files_in_directory(std::filesystem::path path)
{
    using std::filesystem::directory_iterator;
    using fp = bool (*)( const std::filesystem::path&);
    return std::count_if(directory_iterator(path), directory_iterator{}, (fp)std::filesystem::is_regular_file);
}

///
/// \brief checkImage : the basic function to compare 2 pictures
/// \param srcImage1 : first image
/// \param srcImage2 : second image
/// \param width
/// \param height
///
void checkImage(byte **srcImage1, byte **srcImage2,unsigned int width,unsigned int height)
{
    for(unsigned int i=0; i<=height; i++) {
        for(unsigned int j=0; j<=width; j++) {
            ASSERT_EQ(srcImage1[i][j],srcImage2[i][j]);
        }
    }
}

///
/// \brief checkResults : Function to test set of picture with another set (ground truth)
/// \param src_path : Source of the image set to test
/// \param gt_path : Path for the image set used as ground truth
///
void checkResults(const char *src_path,const char *gt_path)
{
    int iSizeSrc,iSizeGT=0;

    iSizeSrc = int(number_of_files_in_directory(src_path));
    iSizeGT = int(number_of_files_in_directory(gt_path));

    if(iSizeGT != iSizeSrc)
    {
        std::cout << "Different size of images sets: Source " <<  iSizeSrc << " : Ground truth "  <<iSizeGT << std::endl;
        FAIL();
    }

    unsigned int width    = 320;
    unsigned int height   = 240;

    byte **srcImage, **gtImage;
    srcImage =  bmatrix(0, height, 0, width); // absolute difference
    gtImage =   bmatrix(0, height, 0, width); // absolute difference


    for (const auto & entry : fs::directory_iterator(src_path))
    {
        MLoadPGM_bmatrix((char*)entry.path().c_str(), 0, height, 0, width, srcImage);
        std::string gdFile = std::string(gt_path) +"/"+ std::string(entry.path().filename().c_str());
        MLoadPGM_bmatrix((char*)gdFile.c_str(), 0, height, 0, width, gtImage);

        //std::cout << gdFile.c_str() << "::" <<entry.path().c_str() << std::endl;

        checkImage(srcImage,gtImage,width,height);
    }

    free_bmatrix(srcImage, 0, width, 0);
    free_bmatrix(gtImage, 0, width, 0);
}

TEST(UnitTest, testUnit1)
{
    generatePictures("dataset/","result_images/");
    checkResults("result_images/","ground_truth/");
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

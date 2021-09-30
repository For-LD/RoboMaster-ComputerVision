#ifndef IMAGE_OPENCV_HPP
#define IMAGE_OPENCV_HPP
#ifdef OPENCV
  #include<darknet.h>
  #include<opencv2/opencv.hpp>
  using namespace cv;
  IplImage *image_to_ipl(image im);
  image ipl_to_image(IplImage* src);
  Mat image_to_mat(image im);
  image mat_to_image(Mat m);
  //void *open_video_stream(const char *f, int c, int w, int h, int fps);
  //image get_image_from_stream(void *p);
  //image load_image_cv(char *filename, int channels);
  //int show_image_cv(image im, const char* name, int ms);
  //void make_window(char *name, int w, int h, int fullscreen);
#endif
#endif

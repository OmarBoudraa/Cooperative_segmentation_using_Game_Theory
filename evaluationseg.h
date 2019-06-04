#ifndef EVALUATIONSEG_H
#define EVALUATIONSEG_H
#include "principale.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <fstream>
#include "evaluationseg.h"
//Max Value Iplimage
double MAXIPIMAGE(IplImage *img);
//Mean Value Iplimage
double MEANIPLIMAGE(IplImage *img, CvSeq* seq);
//Frontière Externe Iplimage
CvSeq *FRONTIEREIPLIMAGE(IplImage *img);
//Mesure de Zéboudj
double ZEBOUDJ(IplImage *isrc_gray, CvSeq *region);
//Critère de Borsotti
double borsotti(IplImage *isrc_gray, CvSeq *region );

#endif // EVALUATIONSEG_H

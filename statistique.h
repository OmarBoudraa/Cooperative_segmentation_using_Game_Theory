#ifndef STATISTIQUE_H
#define STATISTIQUE_H
#include "principale.h"

 IplImage*  TadRGB2GRIS(IplImage *src);
 IplImage*   TadEtireHist(IplImage *src);
 double TadMoyenne(IplImage *src, int x, int y, int width, int height);
 QImage TadIplImage2QImage(IplImage *cvimage);
 IplImage* TadQImage2IplImage(QImage *qimg);
 bool TadBinImg(IplImage *src);
 double TadVariance(IplImage *src, int x, int y, int width, int height);

#endif // STATISTIQUE_H

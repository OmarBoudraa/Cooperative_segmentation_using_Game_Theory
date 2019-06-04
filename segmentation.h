#ifndef SEGMENTATION_H
#define SEGMENTATION_H
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
typedef struct{double distance;
      CvPoint *point;
      } pntstruct;
IplImage *PPROEO(IplImage *imgbenaire);
//Canny
IplImage * Tadcanny(IplImage *isrc_gray);

void onChange(int pos);
int getsmin();
int getsmax();
int getaperature();
//Snakes
CvPoint *Tadsnake(IplImage *isrc_gray,CvPoint* seed, long iteration,IplImage *isrc_canny);
IplImage *getimgsnakes();
long getlength();
CvSeq *getlstcanny();
IplImage *getimgcanny();
//Region growing
IplImage *Tadregiongrowing(IplImage *isrc_gray, CvPoint* seedp, double  lamda, long iteration);
void Tadregiongrowing(IplImage *isrc_gray,CvPoint* seedp,CvPoint* pnt,  double lamda, long iteration);
 IplImage *getimgrbin();
 CvSeq *getlistreg();
 CvSeq *getlistbruit();
//Segmentation coopérative
//CvPoint To CvSeq
CvSeq * CVPOINT2CVSEQ (CvPoint pnt[], long taille);
//Inside CvSeq

CvSeq * INSIDECVSEQ (IplImage *image_gray, CvSeq * seq);
//CvSeq To IplImage
IplImage * CVSEQ2IPLIMAGE (IplImage *image_gray, CvSeq *seq);

//Existe CvPoint
bool EXISTECVPOINT(CvSeq * seq,CvPoint *point);
//Add CvPoint to CvSeq
void ADDCVPOINT2CVSEQ (CvSeq * seq,CvPoint *pnt);
//Remove CvPoint from CvSeq
void REMOVECVPOINTFCVSEQ (CvSeq * seq,CvPoint *point);
//Contourner par des blancs
bool WHITECONTOURNED(IplImage *img, CvPoint *point);
// CvSeq1 N Cvseq2

CvSeq * CVSEQ1NCVSEQ2(CvSeq *seq1,CvSeq *seq2);
//CvSeq inf from Iplimage
CvSeq *COUNTOURFIMG(CvSeq *seq, CvSeq *imgseq);
// Mesure d'Abdou et Pratt
double MAP(CvSeq *seq1, CvSeq *seq2, double a);
//Best Direction
long BESTDIRECT(IplImage *img, CvPoint *pnt,IplImage *nimg);
//Find outer contour
CvSeq * FINDOUTSIDECONTOUR(IplImage *imagebin);
//Calculate first part F1

double CFPF1(IplImage *image_gray,IplImage *image_bin, double lamda, CvSeq *region);
//FIND ALL REGION GROWING
IplImage * ALLREGIONGROWING(IplImage *image_gray, double lamda);

//IplImage AND CvSeq
IplImage * IPLIMAGEANDCVSEQ(IplImage *image, CvSeq *seq, double value);
//Plus proche Point
CvPoint * PPP(CvSeq *seq,CvSeq *contour);
//Calculate second part F1

double CSPF1(IplImage *image_bin, CvSeq *mask,  CvSeq *front);

//Iplimage to CvSeq
CvSeq *IPLIMAGE2CVSEQ(IplImage *img);

//Fill the Gaps
IplImage *FILLGAPS(IplImage *img);

//Get Mask of Region
IplImage *getrmask();
//Get Mask of Snakes
IplImage *getsmask();
//Inverser Iplimage
IplImage *INVIPLIMAGE(IplImage* img);
//Ordonner une liste
CvSeq *ORDREDLIST(CvSeq *seq, CvSeq *contour);
//Clonage de CvSeq
CvSeq *CLONECVSEQ(CvSeq *seq);
//Laisser une seule région
IplImage *LAISSER1SEULE(IplImage *imagebin);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Segmentation coopérative en utilisant la théorie des jeux

void SIUGT(IplImage *isrc_gray,CvPoint *seed, long siteration,IplImage *isrc_canny, long riteration, double lamda, double alpha, double beta, double a);

//Print Resultat de segmentation coopérative
IplImage *printresgt(IplImage *isrc_gray,IplImage *mask);


#endif // SEGMENTATION_H

#include "segmentation.h"
#include "QDebug"
#define APERTURE_SIZE 3
#define LOW_THRESHOLD 50
#define HIGH_THRESHOLD 150
using namespace std;
static int Thresholdness = 141;
static int ialpha = 10;
static int ibeta=50;
static int igamma=40;

//Paramètres de la méthode Snakes
static IplImage *image_snakes = 0 ;
static long length=0;
static CvSeq* listcanny=0;
//Paramètres de la méthode Canny
static IplImage *image_gris = 0 ;
static IplImage *image_canny = 0 ;
static int smin=50;
static int smax=150;
static int aperature=3;
//Paramètres croissance de region
static long iter=0;
static IplImage *image_rbin=0;
static CvSeq* listreg=0;
static CvSeq* listbruit=0;
static bool** visited;
//Paramètres de la segmentation coopérative
static CvSeq* LRNS;
static CvSeq* LSNR;
static IplImage *image_region=0;
static IplImage *image_rbina=0;
static IplImage *image_rmask=0;
static IplImage *image_smask=0;
IplImage *PPROEO(IplImage *imgbenaire){
//cloner trois images de l'image benaire d'entree: -imgcopie: image utilisee pour extraire les contour -imgrect: image des rectangles orientees -imgfinal: image des rectangles orientees remplises
IplImage *imgcopie=cvCloneImage(imgbenaire);
IplImage *imgrect=cvCloneImage(imgbenaire),*imgfinal=cvCloneImage(imgbenaire);
// initialiser les images imgrect et imgfinal à noir (0)
cvZero(imgrect);cvZero(imgfinal);
CvMemStorage* storage1=cvCreateMemStorage(0),* storage2=cvCreateMemStorage(0),* storage3=cvCreateMemStorage(0);
CvSeq *contours1=NULL,*contours2=NULL;
//Extraire les contours des composantes connexes
cvFindContours( imgcopie, storage1, &contours1, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
CvSeq *ptr;
CvBox2D boundbox;
cv::RotatedRect rotatedrectangle;
for (ptr = contours1; ptr != NULL; ptr = ptr->h_next) {
// Finds minimum area rotated rectangle bounding a set of points
        boundbox = cvMinAreaRect2(ptr, storage2);
        rotatedrectangle=boundbox;
        cv::Point2f vertices[4];
        rotatedrectangle.points(vertices);
//Dessiner les rectangles orientee
for (int i = 0; i < 4; i++) cvLine(imgrect, vertices[i], vertices[(i+1)%4], cvScalarAll(255.0));}
cvFindContours( imgrect, storage3, &contours2, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
//Remplir l'interieur des rectangles orientees
for (ptr = contours2; ptr != NULL; ptr = ptr->h_next) {
cvDrawContours( imgfinal, ptr,cvScalarAll(255.0) ,cvScalarAll(255.0) , ptr->total, CV_FILLED);}
//liberer l'espace occupee par les deux images imgcopie et imgrect
cvReleaseImage(&imgcopie);
cvReleaseImage(&imgrect);
return imgfinal;
}
void onChange(int pos){
    if (!(aperature ==3||aperature ==5||aperature ==7)){aperature=3; return;}else{
    cvCanny( image_gris, image_canny, smin, smax, aperature );
    listcanny=IPLIMAGE2CVSEQ(image_canny);
    cvShowImage("Segmentation par la methode canny", image_canny);}

}
IplImage * Tadcanny(IplImage *isrc_gray){

cvNamedWindow( "Segmentation par la methode canny" );
/// Convert the image to grayscale
/*image_gris = cvCreateImage( cvSize(isrc_gray->width,isrc_gray->height),
                                              isrc_gray->depth, isrc_gray->nChannels );*/
image_gris=cvCloneImage(isrc_gray);
image_canny=cvCloneImage(isrc_gray);
/// Reduce noise with a kernel 3x3
//blur( src_gray, detected_edges, cvSize(3,3) );
//cvSmooth(src_gray, detected_edges, CV_GAUSSIAN, 3, 3);

/// Canny detector

cvNamedWindow("Segmentation par la methode canny",0);
 cvCreateTrackbar("Seuil Min", "Segmentation par la methode canny", &smin, 255, onChange);
 cvCreateTrackbar("Seuil Max", "Segmentation par la methode canny", &smax, 255, onChange);
 cvCreateTrackbar("Aperature", "Segmentation par la methode canny", &aperature, 7, onChange);
 onChange(0);
cvShowImage("Segmentation par la methode canny", image_canny);

cvWaitKey(0);
return image_canny;
//cvReleaseImage( &detected_canny );
//cvDestroyWindow( "Segmentation par la methode canny" );

}

int getsmin(){return smin;}
int getsmax(){return smax;}
int getaperature(){return aperature;}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CvPoint* Tadsnake(IplImage *isrc_gray,CvPoint* seed, long iteration,IplImage *isrc_canny){
  /*  image_gris=cvCreateImage( cvSize(isrc_gray->width,isrc_gray->height),
                                     isrc_gray->depth, isrc_gray->nChannels );*/
image_gris=cvCloneImage(isrc_gray);
    image_snakes = cvCreateImage( cvSize(isrc_gray->width,isrc_gray->height),
                                                  isrc_gray->depth, 3 );
    //detected_snakes=cvCloneImage(src_gray);
    cvCvtColor(image_gris,image_snakes,CV_GRAY2RGB);
//image binaire

    //image canny
 /*  image_canny = cvCreateImage( cvSize(image_gris->width,image_gris->height),
                                                  image_gris->depth, image_gris->nChannels );*/
 if(isrc_canny==0){
    image_canny=cvCloneImage(image_gris);
    /// Reduce noise with a kernel 3x3
    //blur( src_gray, detected_edges, cvSize(3,3) );
    //cvSmooth(src_gray, detected_edges, CV_GAUSSIAN, 3, 3);

    /// Canny detector
    cvCanny( image_canny, image_canny, smin, smax, aperature );
listcanny=IPLIMAGE2CVSEQ(image_canny);}
 /*   IplImage    *src_epouse=cvCreateImage( cvSize(isrc_gray->width,isrc_gray->height),
                                         isrc_gray->depth, isrc_gray->nChannels );
*/
    CvPoint* point;
 CvMemStorage* storage = cvCreateMemStorage(1048576);
 CvSeq* contours = NULL;
  // if(lreg==0){

   IplImage *src_epouse=cvCloneImage(isrc_gray);
       cvThreshold(src_epouse,src_epouse,Thresholdness,255,CV_THRESH_OTSU);
       //cvAdaptiveThreshold(src_epouse,src_epouse,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C);

    if(cvGetReal2D(src_epouse,seed->y,seed->x)==0.0) {src_epouse=INVIPLIMAGE(src_epouse);}


    cvFindContours( src_epouse, storage, &contours, sizeof(CvContour),
                    CV_RETR_LIST
     , CV_CHAIN_APPROX_NONE);

   /*}
    else{src_epouse=CVSEQ2IPLIMAGE(isrc_gray,lreg);
        cvFindContours( src_epouse, storage, &contours, sizeof(CvContour),
                        CV_RETR_EXTERNAL
         , CV_CHAIN_APPROX_NONE);}*/

IplImage *img_epouse;

   if(!contours||contours==NULL) return 0;

    CvSeqReader reader;
    CvPoint pt= cvPoint(0,0);

    CvSeq *ptr=contours,*ptri=contours;
    CvRect boundbox;
      length=0;
     //CvPoint* point;
    double smallest_area=image_canny->height*image_canny->width;
    for (ptr = contours; ptr != NULL; ptr = ptr->h_next) {
        boundbox = cvBoundingRect(ptr, 0);
      if( cvContourArea(ptr)<=smallest_area&&seed->x>=boundbox.x&&seed->x<boundbox.x+boundbox.width&&
              seed->y>=boundbox.y&&seed->y<boundbox.y+boundbox.height){

          ptri=ptr;
         // listcanny=ptri;
        length = ptri->total;
        point = new CvPoint[length];
        cvStartReadSeq(ptri, &reader);
        for (int i = 0; i < length; i++)
        {
        CV_READ_SEQ_ELEM(pt, reader);
        point[i]=pt;
        }
      smallest_area=cvContourArea(ptr);
     // img_epouse=CVSEQ2IPLIMAGE(src_epouse,INSIDECVSEQ(src_epouse,ptr));
      img_epouse=CVSEQ2IPLIMAGE(src_epouse,ptr);
      }  //..do sth with ptr
      }

    //IplImage *imgci=cvCloneImage(img_epouse);
  /* for(int x=0;x<image_snakes->width;x++){
        for(int y=0;y<image_snakes->height;y++){
            cvSetReal2D(imgci,y,x,255);
        }
    }*/
   /* CvMemStorage* storageci = cvCreateMemStorage(1048576);
    CvSeq* contoursi = NULL;
    CvSeqReader readerci;

       cvFindContours( imgci, storageci, &contoursi, sizeof(CvContour),
                       CV_RETR_EXTERNAL
        , CV_CHAIN_APPROX_NONE);
       length = contoursi->total;
       point = new CvPoint[length];
       cvStartReadSeq(contoursi, &readerci);
       for (int i = 0; i < length; i++)
       {
       CV_READ_SEQ_ELEM(pt, readerci);
       point[i]=cvPoint(pt.x,pt.y);
       }
*/
    //qDebug()<<length;qDebug()<<i;
    if (length==0) return point;


float alpha=ialpha/100.0f;
    float beta=ibeta/100.0f;
    float gamma=igamma/100.0f;

    CvSize size;
    size.width=21;
    size.height=21;
    CvTermCriteria criteria;
    criteria.type=CV_TERMCRIT_ITER;
    criteria.max_iter=iteration;
    criteria.epsilon=0.1;
    cvSnakeImage( img_epouse,
    point,length,&alpha,&beta,&gamma,CV_VALUE,size,criteria,0);
    for(int i=0;i<length;i++)
    {
    int j = (i+1)%length;
    cvLine( image_snakes, point[i],point[j],CV_RGB( 255, 255, 0 ),1,8,0 );
    }
    //delete []point;
//Get All points
    length=0;
   for(int i=0;i<image_snakes->height;i++){
       for(int j=0;j<image_snakes->width;j++){CvScalar s=cvGet2D(image_snakes,i,j);
        if (s.val[0]==0.0&&s.val[1]==255.0&&s.val[2]==255.0){length++;}
 } }
   point = new CvPoint[length];

   int index=0;
   for(int i=0;i<image_snakes->height;i++){
       for(int j=0;j<image_snakes->width;j++){CvScalar s=cvGet2D(image_snakes,i,j);
           if (s.val[0]==0.0&&s.val[1]==255.0&&s.val[2]==255.0){
           point[index]=cvPoint(j,i) ;

            index++;}}
   }

return point;
/*  cvFindContours( src_bins, storage, &contours, sizeof(CvContour),
                  CV_RETR_LIST,
  CV_CHAIN_APPROX_NONE );*/
}
IplImage *getimgsnakes(){return image_snakes;}
long getlength(){return length;}
CvSeq *getlstcanny(){return listcanny;}
IplImage *getimgcanny(){return image_canny;}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
IplImage *Tadregiongrowing(IplImage *isrc_gray, CvPoint* seedp, double lamda, long iteration)
{
 /*   src_gray=cvCreateImage( cvSize(isrc_gray->width,isrc_gray->height),
                                     isrc_gray->depth, isrc_gray->nChannels );*/

 /*   image_rbin= cvCreateImage( cvSize(isrc_gray->width,isrc_gray->height),isrc_gray->depth, isrc_gray->nChannels );
image_rbin=cvCloneImage(isrc_gray);
 cvThreshold(isrc_gray,image_rbin,Thresholdness,255,CV_THRESH_OTSU);*/
    image_rbin=ALLREGIONGROWING(isrc_gray,lamda);

 //bool visited[isrc_gray->height][isrc_gray->width];
 long height=isrc_gray->height; long width=isrc_gray->width;
visited = new bool* [width];
    for (int i = 0; i < width; i++){visited[i] = new bool[height];}
 for(int i=0;i<isrc_gray->width;i++){for(int j=0;j<isrc_gray->height;j++){visited[i][j]=false;}}

 CvMemStorage* storagea = cvCreateMemStorage(1048576);
 CvMemStorage* storageb = cvCreateMemStorage(1048576);

 listreg = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storagea );
 listbruit = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storageb );
 iter=0;
 Tadregiongrowing(isrc_gray,seedp,seedp,lamda,iteration);

 IplImage *img_rg = cvCreateImage( cvSize(isrc_gray->width,isrc_gray->height),
                                                       isrc_gray->depth, 3 );
        // cvCloneImage(image_source);
                cvCvtColor(isrc_gray, img_rg,CV_GRAY2RGB);
              //  CvScalar s;s.val[0]=;s=cvGet2D(img_dst1,i,j);cvSet2D(img_dst1,i,j,s);

                for(int i=0;i<listreg->total;i++){CvPoint *pt[1];
   pt[0]=(CvPoint *) cvGetSeqElem(listreg,i);
   CvScalar s;s.val[0]=0;s.val[1]=0;s.val[2]=255;cvSet2D(img_rg,pt[0]->y,pt[0]->x,s);}

                for(int i=0;i<listbruit->total;i++){CvPoint *pt[1];
   pt[0]=(CvPoint *) cvGetSeqElem(listbruit,i);
   CvScalar s;s.val[0]=255;s.val[1]=0;s.val[2]=0;cvSet2D(img_rg,pt[0]->y,pt[0]->x,s);}

                return img_rg;
}

void Tadregiongrowing(IplImage *isrc_gray,CvPoint* seedp,CvPoint* pnt,double lamda, long iteration){

 if(!visited[pnt->x][pnt->y]&&iter<iteration){

/*
double V0=0.0,V255=0.0;
V0=pow(cvGetReal2D(isrc_gray,pnt->y,pnt->x)-0.0,2)
       +lamda*(((pnt->x-1>=0)?pow(cvGetReal2D(image_rbin,pnt->y,pnt->x-1)-0.0,2):0.0)
 +((pnt->y-1>=0)?pow(cvGetReal2D(image_rbin,pnt->y-1,pnt->x)-0.0,2):0.0)
+((pnt->x+1<isrc_gray->width)?pow(cvGetReal2D(image_rbin,pnt->y,pnt->x+1)-0.0,2):0.0)
+((pnt->y+1<isrc_gray->height)?pow(cvGetReal2D(image_rbin,pnt->y+1,pnt->x)-0.0,2):0.0));
V255=pow(cvGetReal2D(isrc_gray,pnt->y,pnt->x)-255.0,2)
      +lamda*(((pnt->x-1>=0)?pow(cvGetReal2D(image_rbin,pnt->y,pnt->x-1)-255.0,2):0.0)
 +((pnt->y-1>=0)?pow(cvGetReal2D(image_rbin,pnt->y-1,pnt->x)-255.0,2):0.0)
+((pnt->x+1<isrc_gray->width)?pow(cvGetReal2D(image_rbin,pnt->y,pnt->x+1)-255.0,2):0.0)
+((pnt->y+1<isrc_gray->height)?pow(cvGetReal2D(image_rbin,pnt->y+1,pnt->x)-255.0,2):0.0));
if(V255<=V0){cvSetReal2D(image_rbin,pnt->y,pnt->x,255.0);}else{cvSetReal2D(image_rbin,pnt->y,pnt->x,0.0);}*/

 visited[pnt->x][pnt->y]=true;
iter++;

if(cvGetReal2D(image_rbin,pnt->y,pnt->x)==cvGetReal2D(image_rbin,seedp->y,seedp->x)){

   cvSeqPush(listreg, pnt );


    if(pnt->x-1>=0){
        CvPoint *pt= new CvPoint();
        pt->x=pnt->x-1;pt->y=pnt->y;
        Tadregiongrowing(isrc_gray,seedp,pt,lamda,iteration);

        free(pt);
    }
    if(pnt->y-1>=0)
   {CvPoint *pt= new CvPoint();
        pt->x=pnt->x;pt->y=pnt->y-1;
       Tadregiongrowing(isrc_gray,seedp,pt, lamda,iteration);
       free(pt);
    }
    if(pnt->x+1<isrc_gray->width){
        CvPoint *pt= new CvPoint();
        pt->x=pnt->x+1;pt->y=pnt->y;
      Tadregiongrowing(isrc_gray,seedp,pt, lamda,iteration);
      free(pt);
    }
    if(pnt->y+1<isrc_gray->height){
        CvPoint *pt= new CvPoint();
        pt->x=pnt->x;pt->y=pnt->y+1;
       // Sleep(1);
       Tadregiongrowing(isrc_gray,seedp,pt,  lamda,iteration);
       free(pt);
    }
   //  free(src_binr);
}else{
cvSeqPush( listbruit, pnt );
}

}
}
 IplImage *getimgrbin(){
     return image_rbin;
 }
 CvSeq *getlistreg(){
     return listreg;}
 CvSeq *getlistbruit(){
     return listbruit;}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Segmentation copérative
//CvPoint To CvSeq
 CvSeq * CVPOINT2CVSEQ (CvPoint pnt[], long taille){
     CvMemStorage* storage = cvCreateMemStorage(1048576);
   CvSeq *  cvsequence = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );
   for(int i=0;i<taille;i++){cvSeqPush( cvsequence, &pnt[i] );}
   return cvsequence;
 }
 //Inside CvSeq

 CvSeq * INSIDECVSEQ (IplImage *image_gray, CvSeq * seq){
   /*  IplImage *   imageC2I= cvCreateImage( cvSize(image_gray->width,image_gray->height),
                                         image_gray->depth, image_gray->nChannels );
     cvSetZero(imageC2I);

     CvMemStorage* storage = cvCreateMemStorage(1048576);
   CvSeq *  cvsequence = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );
   for(int i=0;i<seq->total;i++){CvPoint *pnt=(CvPoint *) cvGetSeqElem(seq,i);
       cvSetReal2D(imageC2I,pnt->y,pnt->x,255.0);
   }
   for(int i=0;i<imageC2I->height;i++){
       for(int j=0;j<imageC2I->width;j++){
           CvPoint *pt= new CvPoint();
               pt->x=j;pt->y=i;
               if (WHITECONTOURNED(imageC2I,pt)){
                   cvSetReal2D(imageC2I,pt->y,pt->x,255.0);
               }

         free(pt);}}
//cvScalarAll(255)
 //  cvDrawContours( imageC2I, seq,cvRealScalar(255.0) , cvRealScalar(255.0) , seq->total, CV_FILLED);

   for(int i=0;i<imageC2I->height;i++){
       for(int j=0;j<imageC2I->width;j++){double s=cvGetReal2D(imageC2I,i,j);
           if (s==255.0){CvPoint *pt= new CvPoint();
               pt->x=j;pt->y=i;
               cvSeqPush( cvsequence, pt );
         }}}*/
IplImage *   imageC2I=CVSEQ2IPLIMAGE(image_gray,seq);
IplImage *   fimageC2I=FILLGAPS(imageC2I);
CvSeq * cvsequence=IPLIMAGE2CVSEQ(fimageC2I);
cvReleaseImage(&imageC2I);cvReleaseImage(&fimageC2I);
   return cvsequence;
 }
 //CvSeq To IplImage
 IplImage * CVSEQ2IPLIMAGE (IplImage *image_gray,CvSeq *seq){
  IplImage *   imageC2I= cvCreateImage( cvSize(image_gray->width,image_gray->height),
                                      image_gray->depth, image_gray->nChannels );
  cvSetZero(imageC2I);
   for(int i=0;i<seq->total;i++){CvPoint *pnt=(CvPoint *) cvGetSeqElem(seq,i);
       cvSetReal2D(imageC2I,pnt->y,pnt->x,255.0);
   }
   return imageC2I;
 }

//Existe CvPoint
 bool EXISTECVPOINT(CvSeq * seq,CvPoint *point)
 {
 bool exist=false;
 for(int i=0;i<seq->total;i++){CvPoint *pnt=(CvPoint *) cvGetSeqElem(seq,i);
  if(pnt->x==point->x&&pnt->y==point->y){exist=true;break;}
 }
   return exist;
 }
 //Add CvPoint to CvSeq
void ADDCVPOINT2CVSEQ (CvSeq * seq,CvPoint *pnt){
 if (!EXISTECVPOINT (seq,pnt)){
               cvSeqPush( seq, pnt );
 }
  // return seq;
 }
 //Remove CvPoint from CvSeq
void REMOVECVPOINTFCVSEQ (CvSeq * seq,CvPoint *point){
     for(int i=0;i<seq->total;i++){CvPoint *pnt=(CvPoint *) cvGetSeqElem(seq,i);
      if(pnt->x==point->x&&pnt->y==point->y){cvSeqRemove(seq,i);break;
      }
     }
  // return seq;
 }
 //Contourner par des blancs
 bool WHITECONTOURNED(IplImage *img, CvPoint *point){
    bool left=false,right=false,top=false,down=false;
     int x=point->x;
     int y=point->y;
     while (x>=0){
     if(cvGetReal2D(img,y,x)==255.0) {left=true;break;}
     x=x-1;
     }

     x=point->x;
         y=point->y;
          while (y>=0){
          if(cvGetReal2D(img,y,x)==255.0) {top=true;break;}
          y=y-1;
          }
          x=point->x;
              y=point->y;
               while (y<img->height){
               if(cvGetReal2D(img,y,x)==255.0) {down=true;break;}
               y=y+1;
               }
               x=point->x;
                   y=point->y;
                    while (x<img->width){
                    if(cvGetReal2D(img,y,x)==255.0) {right=true;break;}
                    x=x+1;
                    }

                    return (left&&right&&top&&down);

 }
// CvSeq1 N Cvseq2

 CvSeq * CVSEQ1NCVSEQ2(CvSeq *seq1,CvSeq *seq2){

     CvMemStorage* storage = cvCreateMemStorage(1048576);
   CvSeq *  cvsequence = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );

     for(int i=0;i<seq1->total;i++){CvPoint *pnt=(CvPoint *) cvGetSeqElem(seq1,i);
       if(!EXISTECVPOINT(seq2,pnt))//&&!WHITECONTOURNED(mask, pnt))
       {
      cvSeqPush( cvsequence, pnt );
       }
     }
     //cvReleaseMemStorage(&storage);
 return cvsequence;
 }
 //CvSeq inf from Iplimage
CvSeq *COUNTOURFIMG(CvSeq *seq, CvSeq *imgseq){
    CvMemStorage* storage = cvCreateMemStorage(1048576);
  CvSeq *  cvsequence = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );

  CvPoint *pnti= new CvPoint(); pnti->x=0;pnti->y=0;
  int height=image_canny->height,width=image_canny->width;
  double si=pow(pow(width,2.0)+pow(height,2.0),0.5);
  double mindist,dist;
  CvPoint *pnt,*pntimgseq;
  //CvSeq *imgseq=IPLIMAGE2CVSEQ(img);
  for(int t=0;t<seq->total;t++){pnt=(CvPoint *) cvGetSeqElem(seq,t);
      mindist=si;
      for(int k=0;k<imgseq->total;k++){pntimgseq=(CvPoint *) cvGetSeqElem(imgseq,k);
              dist=pow(pow(pnt->x-pntimgseq->x,2.0)+pow(pnt->y-pntimgseq->y,2.0),0.5);
              if (mindist>dist){
                  mindist=dist;
                  //pnti= cvPoint(pntimgseq->x,pntimgseq->y);
                  pnti->x=pntimgseq->x;pnti->y=pntimgseq->y;
            }}

ADDCVPOINT2CVSEQ (cvsequence,pnti);
//free(pnti);
  // cvSeqPush( cvsequence, pnti );
    }

  //cvReleaseMemStorage(&(imgseq->storage));
return cvsequence;


}
// Mesure d'Abdou et Pratt : contour ideal seq1
double MAP(CvSeq *seq1, CvSeq *seq2, double a){
    long N=max(seq1->total,seq2->total);
double mesure=0.0;
double mindist=1000000000.0;
    if(N==seq1->total){
        for(int i=0;i<seq1->total;i++){
            CvPoint *pnt1=(CvPoint *) cvGetSeqElem(seq1,i);
        for(int j=0;j<seq2->total;j++){
            CvPoint *pnt2=(CvPoint *) cvGetSeqElem(seq2,j);
        if(pow(pow(pnt1->x-pnt2->x,2.0)+pow(pnt1->y-pnt2->y,2.0),0.5)<mindist){
          mindist=pow(pow(pnt1->x-pnt2->x,2.0)+pow(pnt1->y-pnt2->y,2.0),0.5);
        }
        }

        mesure=mesure+(1.0/(1.0+a*mindist));
        }
        mesure=mesure/N;
    }
    else{
        for(int i=0;i<seq2->total;i++){
            CvPoint *pnt2=(CvPoint *) cvGetSeqElem(seq2,i);
        for(int j=0;j<seq1->total;j++){
            CvPoint *pnt1=(CvPoint *) cvGetSeqElem(seq1,j);
        if(pow(pow(pnt1->x-pnt2->x,2.0)+pow(pnt1->y-pnt2->y,2.0),0.5)<mindist){
          mindist=pow(pow(pnt1->x-pnt2->x,2.0)+pow(pnt1->y-pnt2->y,2.0),0.5);
        }
        }

        mesure=mesure+(1.0/(1.0+a*mindist));
         }
        mesure=mesure/N;
    }
//qDebug()<<mesure;
return mesure;


}
//Best Direction
long BESTDIRECT(IplImage *img, CvPoint *pnt,IplImage *nimg){
    long direct=-1;
    long distl=0,distr=0,distt=0,distd=0;
    int x=pnt->x;
    int y=pnt->y;

    while(x-1>=0&&cvGetReal2D(img,y,x-1)==255.0){
        if(cvGetReal2D(nimg,y,x-1)==255.0){distl=1000000;break;}
 distl++;
 x=x-1;
}

    x=pnt->x;
    y=pnt->y;
    while(x+1<img->width&&cvGetReal2D(img,y,x+1)==255.0){
   if(cvGetReal2D(nimg,y,x+1)==255.0){distr=1000000;break;}
        distr++;
 x=x+1;
}
    x=pnt->x;
    y=pnt->y;
    while(y-1>=0&&cvGetReal2D(img,y-1,x)==255.0){
 if(cvGetReal2D(nimg,y-1,x)==255.0){distt=1000000;break;}
 distt++;
 y=y-1;
}
    x=pnt->x;
    y=pnt->y;
    while(y+1<img->height&&cvGetReal2D(img,y+1,x)==255.0){
 if(cvGetReal2D(nimg,y+1,x)==255.0){distd=1000000;break;}
        distd++;
 y=y+1;
}
    if(distd==distl&&distl==distr&&distr==distt&&distt==0){return direct;}
    if(distd==distl&&distl==distr&&distr==distt&&distt==1000000){qDebug()<<"kokokokoko";
        return direct;}
    long bestdist=min(min(min(distl,distr),distt),distd);
 if(bestdist==distl){direct=0;}
 else if(bestdist==distr){direct=2;}
 else if (bestdist==distt){direct=1;}
 else if (bestdist==distd) {direct=3;}
return direct;


}
//Find outer contour
CvSeq * FINDOUTSIDECONTOUR(IplImage *imagebin){
IplImage *img=cvCloneImage(imagebin);
    CvMemStorage* storage = cvCreateMemStorage(1048576);
    CvSeq* contours = NULL;
    cvFindContours( img, storage, &contours, sizeof(CvContour),CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    // cvReleaseMemStorage(&storage);
    cvReleaseImage(&img);
    return contours;
}
//Calculate first part  F1

double CFPF1(IplImage *image_gray,IplImage *image_bin, double lamda, CvSeq *region){
    double V=0.0;
  /*  for(int y=0;y<image_gray->height;y++){
        for(int x=0;x<image_gray->width;x++){
    V=V+pow(cvGetReal2D(image_gray,y,x)-cvGetReal2D(image_bin,y,x),2)
           +lamda*(((x-1>=0)?pow(cvGetReal2D(image_bin,y,x-1)-cvGetReal2D(image_bin,y,x),2):0.0)
     +((y-1>=0)?pow(cvGetReal2D(image_bin,y-1,x)-cvGetReal2D(image_bin,y,x),2):0.0)
    +((x+1<image_gray->width)?pow(cvGetReal2D(image_bin,y,x+1)-cvGetReal2D(image_bin,y,x),2):0.0)
    +((y+1<image_gray->height)?pow(cvGetReal2D(image_bin,y+1,x)-cvGetReal2D(image_bin,y,x),2):0.0));

}}*/
    int x,y;
    for(int i=0;i<region->total;i++){
        CvPoint *pnt=(CvPoint *) cvGetSeqElem(region,i);
        x=pnt->x;y=pnt->y;

        V=V+pow(cvGetReal2D(image_gray,y,x)-cvGetReal2D(image_bin,y,x),2.0)
               +lamda*(((x-1>=0)?pow(cvGetReal2D(image_bin,y,x-1)-cvGetReal2D(image_bin,y,x),2.0):0.0)
         +((y-1>=0)?pow(cvGetReal2D(image_bin,y-1,x)-cvGetReal2D(image_bin,y,x),2.0):0.0)
        +((x+1<image_gray->width)?pow(cvGetReal2D(image_bin,y,x+1)-cvGetReal2D(image_bin,y,x),2.0):0.0)
        +((y+1<image_gray->height)?pow(cvGetReal2D(image_bin,y+1,x)-cvGetReal2D(image_bin,y,x),2.0):0.0));
    }

return V;}
//FIND ALL REGION GROWING
IplImage * ALLREGIONGROWING(IplImage *image_gray, double lamda){
/*  IplImage *  image_bin= cvCreateImage( cvSize(image_gray->width,image_gray->height),
                                     image_gray->depth, image_gray->nChannels );*/
 IplImage *image_bin=cvCloneImage(image_gray);
 cvThreshold(image_gray,image_bin,Thresholdness,255,CV_THRESH_OTSU);
/*double V255,V0;
for(int y=0;y<image_gray->height;y++){
    for(int x=0;x<image_gray->width;x++){
cvSetReal2D(image_bin,y,x,0.0);
V0=CFPF1(image_gray,image_bin,lamda);
cvSetReal2D(image_bin,y,x,255.0);
V255=CFPF1(image_gray,image_bin,lamda);
if(V255<=V0){cvSetReal2D(image_bin,y,x,255.0);}else{cvSetReal2D(image_bin,y,x,0.0);}
    }
}*/

 double V0=0.0,V255=0.0;
 for(int y=0;y<image_gray->height;y++){
     for(int x=0;x<image_gray->width;x++){
 V0=pow(cvGetReal2D(image_gray,y,x)-0.0,2.0)
        +lamda*(((x-1>=0)?pow(cvGetReal2D(image_bin,y,x-1)-0.0,2.0):0.0)
  +((y-1>=0)?pow(cvGetReal2D(image_bin,y-1,x)-0.0,2.0):0.0)
 +((x+1<image_gray->width)?pow(cvGetReal2D(image_bin,y,x+1)-0.0,2.0):0.0)
 +((y+1<image_gray->height)?pow(cvGetReal2D(image_bin,y+1,x)-0.0,2.0):0.0));
 V255=pow(cvGetReal2D(image_gray,y,x)-255.0,2.0)
  +lamda*(((x-1>=0)?pow(cvGetReal2D(image_bin,y,x-1)-255.0,2.0):0.0)
  +((y-1>=0)?pow(cvGetReal2D(image_bin,y-1,x)-255.0,2.0):0.0)
 +((x+1<image_gray->width)?pow(cvGetReal2D(image_bin,y,x+1)-255.0,2.0):0.0)
 +((y+1<image_gray->height)?pow(cvGetReal2D(image_bin,y+1,x)-255.0,2.0):0.0));
 if(V255<=V0){cvSetReal2D(image_bin,y,x,255.0);}else{cvSetReal2D(image_bin,y,x,0.0);}}}

return image_bin;
}

//IplImage AND CvSeq
IplImage * IPLIMAGEANDCVSEQ(IplImage *image, CvSeq *seq, double value){
/*  IplImage *  imagea= cvCreateImage( cvSize(image->width,image->height),
                                     image->depth, image->nChannels );*/
IplImage * imagea=cvCloneImage(image);
for(int i=0;i<seq->total;i++){
    CvPoint *pnt=(CvPoint *) cvGetSeqElem(seq,i);
    cvSetReal2D(imagea,pnt->y,pnt->x,value);
}
return imagea;
}
//Plus proche Point
CvPoint * PPP(CvSeq *seq,CvSeq *contour){
    double mindist=1000000000.0,dist;
 CvPoint * pnti=new CvPoint();
  for(int i=0;i<seq->total;i++){CvPoint *pnts=(CvPoint *) cvGetSeqElem(seq,i);

     for(int j=0;j<contour->total;j++){CvPoint *pntc=(CvPoint *) cvGetSeqElem(contour,j);
         dist=pow(pow(pnts->x-pntc->x,2.0)+pow(pnts->y-pntc->y,2.0),0.5);
         if (mindist>dist){
             mindist=dist;
             pnti->x=pnts->x; pnti->y=pnts->y;
     }
  }

    }
  return pnti;
}
//Calculate second part  F1

double CSPF1(IplImage *image_bin, CvSeq *mask,  CvSeq *front){
 /*   double mi=0,me=0;

    for(int y=0;y<image_bin->height;y++){
        for(int x=0;x<image_bin->width;x++){
             CvPoint *pnt=new CvPoint();
             pnt->x=x;pnt->y=y;
             if (EXISTECVPOINT(mask,pnt)){mi=mi+cvGetReal2D(image_bin,y,x);}else{
                 me=me+cvGetReal2D(image_bin,y,x);
             }
        free(pnt);}}
    mi=mi/mask->total;me=me/((image_bin->height*image_bin->width)-mask->total);
    double V=0.0;
    for(int y=0;y<image_bin->height;y++){
        for(int x=0;x<image_bin->width;x++){

            CvPoint *pnt=new CvPoint();
            pnt->x=x;pnt->y=y;
            if (EXISTECVPOINT(mask,pnt)){ V=V+pow(cvGetReal2D(image_bin,y,x)-mi,2);}else{
                V=V+pow(cvGetReal2D(image_bin,y,x)-me,2);
            }
free(pnt);
}
    }
return V;*/
       double mi=0.0,me=0.0;double V=0.0;
    for(int i=0;i<mask->total;i++){CvPoint *pnts=(CvPoint *) cvGetSeqElem(mask,i);
    mi=mi+cvGetReal2D(image_bin,pnts->y,pnts->x);}

    for(int i=0;i<front->total;i++){CvPoint *pnts=(CvPoint *) cvGetSeqElem(front,i);
    me=me+cvGetReal2D(image_bin,pnts->y,pnts->x);}

    mi=mi/mask->total;me=me/front->total;

    for(int i=0;i<mask->total;i++){CvPoint *pnts=(CvPoint *) cvGetSeqElem(mask,i);
    V=V+pow(cvGetReal2D(image_bin,pnts->y,pnts->x)-mi,2.0);}

    for(int i=0;i<front->total;i++){CvPoint *pnts=(CvPoint *) cvGetSeqElem(front,i);
     V=V+pow(cvGetReal2D(image_bin,pnts->y,pnts->x)-me,2.0);}
   return V;}
//Iplimage to CvSeq
CvSeq *IPLIMAGE2CVSEQ(IplImage *img){
  CvMemStorage* storage = cvCreateMemStorage(1048576);
  CvSeq *  cvsequence = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );

  for(int i=0;i<img->height;i++){for(int j=0;j<img->width;j++){
          if(cvGetReal2D(img,i,j)==255.0){
              CvPoint *pnt=new CvPoint();
              pnt->x=j;pnt->y=i;
              cvSeqPush(cvsequence,pnt);
              free(pnt);
          }

  }}


    return cvsequence;
}
//Fill the Gaps
IplImage *FILLGAPS(IplImage *imgbenaire){
    IplImage *imgcopie=cvCloneImage(imgbenaire);
IplImage *imgrect=cvCloneImage(imgbenaire),*imgfinal=cvCloneImage(imgbenaire);
cvZero(imgrect);cvZero(imgfinal);

    CvMemStorage* storage1=cvCreateMemStorage(0),* storage2=cvCreateMemStorage(0),* storage3=cvCreateMemStorage(0);
    CvSeq *contours1=NULL,*contours2=NULL;

    cvFindContours( imgcopie, storage1, &contours1, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

    CvSeq *ptr;
    CvBox2D boundbox;
    cv::RotatedRect rotatedrectangle;
    for (ptr = contours1; ptr != NULL; ptr = ptr->h_next) {
        boundbox = cvMinAreaRect2(ptr, storage2);
        rotatedrectangle=boundbox;
        cv::Point2f vertices[4];
        rotatedrectangle.points(vertices);
        for (int i = 0; i < 4; i++)
 cvLine(imgrect, vertices[i], vertices[(i+1)%4], cvScalarAll(255.0));}
    cvFindContours( imgrect, storage3, &contours2, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
    for (ptr = contours2; ptr != NULL; ptr = ptr->h_next) {
        cvDrawContours( imgfinal, ptr,cvScalarAll(255.0) ,cvScalarAll(255.0) , ptr->total, CV_FILLED);}


    cvReleaseImage(&imgcopie);
    cvReleaseImage(&imgrect);

return imgfinal;
}
//Get Mask of Region
IplImage *getrmask(){

    return image_rmask;

}
//Get Mask of Snakes
IplImage *getsmask(){

    return image_smask;

}
//Inverser Iplimage
IplImage *INVIPLIMAGE(IplImage* img){

    IplImage *invimg=cvCloneImage(img);
    for(int i=0;i<img->height;i++){
        for(int j=0;j<img->width;j++){
            cvSetReal2D(invimg,i,j,255.0-cvGetReal2D(img,i,j));

        }
    }
    cvReleaseImage(&img);
    return invimg;
}
//Ordonner une liste
CvSeq *ORDREDLIST(CvSeq *seq, CvSeq *contour){
 long n=seq->total;CvPoint  *opoint = new CvPoint();
    //CvPoint  *opoint = new CvPoint[n];
 CvMemStorage* storage = cvCreateMemStorage(1048576);
CvSeq *  cvsequence = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );

 for(int i=0;i<n;i++){opoint=PPP(seq,contour);
     cvSeqPush(cvsequence,opoint);
     REMOVECVPOINTFCVSEQ(seq,opoint);
 free(opoint);}
 return cvsequence;

}
//Clonage de CvSeq
CvSeq *CLONECVSEQ(CvSeq *seq){
 CvMemStorage* storage = cvCreateMemStorage(1048576);
CvSeq *  cvsequence = cvCloneSeq(seq,storage);

 return cvsequence;

}
//Laisser une seule région
IplImage *LAISSER1SEULE(IplImage *imagebin){
    IplImage *img=cvCloneImage(imagebin);
        CvMemStorage* storage = cvCreateMemStorage(1048576);
        CvSeq* contours = NULL;
        cvFindContours( img, storage, &contours, sizeof(CvContour),CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        CvSeq *ptr=contours;
        double biggest_area=0,area=0;
        CvPoint *pnts;
        for (ptr = contours; ptr != NULL; ptr = ptr->h_next) {
            area = cvContourArea(ptr);
          if( biggest_area<area){biggest_area=area;}
        }
        for (ptr = contours; ptr != NULL; ptr = ptr->h_next) {
            area = cvContourArea(ptr);
          if( biggest_area!=area){for(int i=0;i<ptr->total;i++){pnts=(CvPoint *) cvGetSeqElem(ptr,i);
              cvSetReal2D(imagebin,pnts->y,pnts->x,0.0);}
        }}
         cvReleaseMemStorage(&storage);
        cvReleaseImage(&img);
        return imagebin;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Segmentation coopérative en utilisant la théorie des jeux

void SIUGT(IplImage *isrc_gray,CvPoint *seed, long siteration,IplImage *isrc_canny, long riteration, double lamda, double alpha, double beta, double a){
//
image_region=Tadregiongrowing(isrc_gray, seed, lamda, riteration);
CvSeq *listregion=IPLIMAGE2CVSEQ(FILLGAPS(CVSEQ2IPLIMAGE(isrc_gray,listreg)));

CvPoint *spoints=Tadsnake(isrc_gray,seed,siteration,isrc_canny);
CvSeq *listsnake=INSIDECVSEQ(isrc_gray,CVPOINT2CVSEQ(spoints,length));

//
image_smask=CVSEQ2IPLIMAGE(isrc_gray,listsnake);
image_rmask=CVSEQ2IPLIMAGE(isrc_gray,listregion);
//cvShowImage("snake",image_smask); cvWaitKey(0);
//cvShowImage("region",image_rmask); cvWaitKey(0);
//
LSNR=CVSEQ1NCVSEQ2(listsnake,listregion);
LRNS=CVSEQ1NCVSEQ2(listregion,listsnake);

//
image_rbina= cvCloneImage(image_rbin);

//image_rbina= ALLREGIONGROWING(isrc_gray, lamda);
double seedvalue=cvGetReal2D(image_rbin,seed->y,seed->x);
//image_rbina=IPLIMAGEANDCVSEQ(image_rbina,listregion,seedvalue);
//
CvSeq *osmask,*ormask,*sarea,*oscanny,*nscanny,*rarea,*sfront;

/////////////////////////////
long nb=LRNS->total;
image_rmask=IPLIMAGEANDCVSEQ(image_rmask,LRNS,0.0);
image_rbina=IPLIMAGEANDCVSEQ(image_rbina,LRNS,255.0-seedvalue);
CvSeq *outsidesnake=FINDOUTSIDECONTOUR(image_smask);
oscanny=COUNTOURFIMG(outsidesnake,listcanny);
LRNS=ORDREDLIST(LRNS,outsidesnake);
CvPoint *point;
qDebug() << "Liste des points qui appartient à la région d'interêt situés à l'exterieur du contour";
//while(LRNS->total!=0){
    while(nb>0){
    qDebug() << nb;
point=(CvPoint *) cvGetSeqElem(LRNS,0);
      int x=point->x;
      int y=point->y;

      double E1,E2;
             osmask=FINDOUTSIDECONTOUR(image_smask);
              ormask=FINDOUTSIDECONTOUR(image_rmask);
             sarea=IPLIMAGE2CVSEQ(image_smask);
            rarea=IPLIMAGE2CVSEQ(image_rmask);
              //oscanny=COUNTOURFIMG(osmask,listcanny);
             sfront=FRONTIEREIPLIMAGE(image_smask);


      E1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
      E2=MAP(oscanny,osmask,a) +beta*MAP(osmask,ormask,a);
              cvReleaseMemStorage(&(osmask->storage));
              cvReleaseMemStorage(&(ormask->storage));
               cvReleaseMemStorage(&(sarea->storage));
               cvReleaseMemStorage(&(rarea->storage));
                //cvReleaseMemStorage(&(oscanny->storage));
                cvReleaseMemStorage(&(sfront->storage));
//qDebug() << E1;qDebug() << E2;
cvSetReal2D(image_smask,y,x,255.0);
cvSetReal2D(image_rmask,y,x,255.0);
cvSetReal2D(image_rbina,y,x,seedvalue);

double EI1,EI2;
    osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);
    sarea=IPLIMAGE2CVSEQ(image_smask);nscanny=COUNTOURFIMG(osmask,listcanny);
    rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);
           EI1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
           EI2=MAP(nscanny,osmask,a)  +beta*MAP(osmask,ormask,a);
    double E=((E1-EI1)/E1)+((EI2-E2)/E2);
//qDebug() << EI1;qDebug() << EI2;qDebug() << E;
    cvReleaseMemStorage(&(osmask->storage));
    cvReleaseMemStorage(&(ormask->storage));
    cvReleaseMemStorage(&(sarea->storage));
     //cvReleaseMemStorage(&(nscanny->storage));
     cvReleaseMemStorage(&(rarea->storage));
     cvReleaseMemStorage(&(sfront->storage));
if(E<0){
    cvSetReal2D(image_smask,y,x,0.0);
    cvSetReal2D(image_rmask,y,x,0.0);
    cvSetReal2D(image_rbina,y,x,255.0-seedvalue);
    cvReleaseMemStorage(&(nscanny->storage));
}else{cvReleaseMemStorage(&(oscanny->storage));
    oscanny=CLONECVSEQ(nscanny);
    cvReleaseMemStorage(&(nscanny->storage));
}
REMOVECVPOINTFCVSEQ(LRNS, point);
nb--;
   image_smask=LAISSER1SEULE(image_smask);
   image_rmask=LAISSER1SEULE(image_rmask);
    }

//free(point);
cvReleaseMemStorage(&(outsidesnake->storage));
    cvReleaseMemStorage(&(LRNS->storage));
//////////////////////////////////////////////////////////////////////////////////

    nb=LSNR->total;

image_smask=IPLIMAGEANDCVSEQ(image_smask,LSNR,0.0);
//image_rbina=IPLIMAGEANDCVSEQ(image_rbina,LSNR,255.0-seedvalue);
CvSeq *outsideregion=FINDOUTSIDECONTOUR(image_rmask);
outsidesnake=FINDOUTSIDECONTOUR(image_smask);
oscanny=COUNTOURFIMG(outsidesnake,listcanny);
LSNR=ORDREDLIST(LSNR,outsideregion);
qDebug() << "Liste des points qui n'appartient pas à la région d'interêt situés à l'interieur du contour";
   // while(LSNR->total!=0){

    while(nb>0){
    qDebug() << nb;
point=(CvPoint *) cvGetSeqElem(LSNR,0);
      int x=point->x;
      int y=point->y;

      double E1,E2;
             osmask=FINDOUTSIDECONTOUR(image_smask);
              ormask=FINDOUTSIDECONTOUR(image_rmask);
             sarea=IPLIMAGE2CVSEQ(image_smask);
            rarea=IPLIMAGE2CVSEQ(image_rmask);
              //oscanny=COUNTOURFIMG(osmask,listcanny);
             sfront=FRONTIEREIPLIMAGE(image_smask);


      E1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
      E2=MAP(oscanny,osmask,a) +beta*MAP(osmask,ormask,a);
              cvReleaseMemStorage(&(osmask->storage));
              cvReleaseMemStorage(&(ormask->storage));
               cvReleaseMemStorage(&(sarea->storage));
               cvReleaseMemStorage(&(rarea->storage));
                //cvReleaseMemStorage(&(oscanny->storage));
                cvReleaseMemStorage(&(sfront->storage));

cvSetReal2D(image_smask,y,x,255.0);
cvSetReal2D(image_rmask,y,x,255.0);
cvSetReal2D(image_rbina,y,x,seedvalue);

double EI1,EI2;
    osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);
    sarea=IPLIMAGE2CVSEQ(image_smask);nscanny=COUNTOURFIMG(osmask,listcanny);
    rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);
           EI1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
           EI2=MAP(nscanny,osmask,a)  +beta*MAP(osmask,ormask,a);
    double E=((E1-EI1)/E1)+((EI2-E2)/E2);

    cvReleaseMemStorage(&(osmask->storage));
    cvReleaseMemStorage(&(ormask->storage));
    cvReleaseMemStorage(&(sarea->storage));
     //cvReleaseMemStorage(&(nscanny->storage));
     cvReleaseMemStorage(&(rarea->storage));
     cvReleaseMemStorage(&(sfront->storage));
if(E<0){
    cvSetReal2D(image_smask,y,x,0.0);
    cvSetReal2D(image_rmask,y,x,0.0);
    cvSetReal2D(image_rbina,y,x,255.0-seedvalue);
cvReleaseMemStorage(&(nscanny->storage));
}
else{cvReleaseMemStorage(&(oscanny->storage));
    oscanny=CLONECVSEQ(nscanny);
    cvReleaseMemStorage(&(nscanny->storage));
}
REMOVECVPOINTFCVSEQ(LSNR, point);
nb--;
image_smask=LAISSER1SEULE(image_smask);
image_rmask=LAISSER1SEULE(image_rmask);
    }

//free(point);
cvReleaseMemStorage(&(outsideregion->storage));
cvReleaseMemStorage(&(outsidesnake->storage));
cvReleaseMemStorage(&(LSNR->storage));
//
//image_rmask=cvCloneImage(image_smask);
}
//Print Resultat de segmentation coopérative
IplImage *printresgt(IplImage *isrc_gray,IplImage *mask){

    IplImage *img = cvCreateImage( cvSize(isrc_gray->width,isrc_gray->height),
                                                          isrc_gray->depth, 3 );
                  cvCvtColor(isrc_gray, img,CV_GRAY2RGB);
               CvSeq *seq=IPLIMAGE2CVSEQ(mask);
               CvSeq *oseq=FINDOUTSIDECONTOUR(mask);
                   for(int i=0;i<seq->total;i++){CvPoint *pt[1];
      pt[0]=(CvPoint *) cvGetSeqElem(seq,i);
      CvScalar s;s.val[0]=0;s.val[1]=0;s.val[2]=255;cvSet2D(img,pt[0]->y,pt[0]->x,s);}

                   for(int i=0;i<oseq->total;i++){CvPoint *pt[1];
      pt[0]=(CvPoint *) cvGetSeqElem(oseq,i);
      CvScalar s;s.val[0]=0;s.val[1]=255;s.val[2]=255;cvSet2D(img,pt[0]->y,pt[0]->x,s);}
   cvReleaseMemStorage(&(seq->storage));
   cvReleaseMemStorage(&(oseq->storage));
                   return img;
}
//Segmentation coopérative en utilisant la théorie des jeux
/*
void SIUGT(IplImage *isrc_gray,CvPoint *seed, long siteration, long riteration, double lamda, double alpha, double beta, double a){
//    
image_region=Tadregiongrowing(isrc_gray, seed, lamda, riteration);
CvSeq *listregion=IPLIMAGE2CVSEQ(FILLGAPS(CVSEQ2IPLIMAGE(isrc_gray,listreg)));

CvPoint *spoints=Tadsnake(isrc_gray,seed,siteration);
CvSeq *listsnake=INSIDECVSEQ(isrc_gray,CVPOINT2CVSEQ(spoints,length));

//
image_smask=CVSEQ2IPLIMAGE(isrc_gray,listsnake);
image_rmask=CVSEQ2IPLIMAGE(isrc_gray,listregion);
//
LSNR=CVSEQ1NCVSEQ2(listsnake,listregion);
LRNS=CVSEQ1NCVSEQ2(listregion,listsnake);

//
image_rbina= cvCloneImage(image_rbin);

//image_rbina= ALLREGIONGROWING(isrc_gray, lamda);
double seedvalue=cvGetReal2D(image_rbin,seed->y,seed->x);
//image_rbina=IPLIMAGEANDCVSEQ(image_rbina,listregion,seedvalue);
//
CvSeq *osmask,*ormask,*sarea,*scanny,*rarea,*sfront;
/////////////////////////////
long nb=LRNS->total;
qDebug() << "Liste des points qui appartient à la région d'interêt situés à l'exterieur du contour";
//while(LRNS->total!=0){
    while(nb>0){
        CvSeq *outsidesnake=FINDOUTSIDECONTOUR(image_smask);
  CvPoint *point= PPP(LRNS,outsidesnake);
  long direct=  BESTDIRECT(image_rmask,point,image_smask);
 // qDebug() << direct;
    qDebug() << nb;
  if(direct==-1){
      REMOVECVPOINTFCVSEQ(LRNS,point);
      nb--;
      cvSetReal2D(image_rmask,point->y,point->x,0.0);
      cvSetReal2D(image_rbina,point->y,point->x,255.0-cvGetReal2D(image_rbina,point->y,point->x));
  }

  else if(direct==0){

      int x=point->x;
      int y=point->y;
  long nbpoint=1;
      while(x-1>=0&&cvGetReal2D(image_rmask,y,x-1)==255.0){
   nbpoint++;
   x=x-1;
  }
     // qDebug() << nbpoint;
      double *energ= new double[nbpoint];
    //  double *lastvalue= new double[nbpoint];
     //
       x=point->x;
       y=point->y;
    //  for(int i=0;i<nbpoint;i++){lastvalue[i]=cvGetReal2D(image_rbina,y,x-i);}
      double E1,E2;
             osmask=FINDOUTSIDECONTOUR(image_smask);
              ormask=FINDOUTSIDECONTOUR(image_rmask);
             sarea=IPLIMAGE2CVSEQ(image_smask);
            rarea=IPLIMAGE2CVSEQ(image_rmask);
              scanny=COUNTOURFIMG(osmask,image_canny);
             sfront=FRONTIEREIPLIMAGE(image_smask);


      E1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
      E2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
              cvReleaseMemStorage(&(osmask->storage));
              cvReleaseMemStorage(&(ormask->storage));
               cvReleaseMemStorage(&(sarea->storage));
               cvReleaseMemStorage(&(rarea->storage));
                cvReleaseMemStorage(&(scanny->storage));
                cvReleaseMemStorage(&(sfront->storage));

      // qDebug() << osmask->storage->free_space;
      //qDebug()<<E1;
       //qDebug()<<E2;

for(int i=0;i<nbpoint;i++){
cvSetReal2D(image_smask,y,x-i,255.0);}
for(int i=nbpoint-1;i>=0;i--){double EI1,EI2;
    osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);sarea=IPLIMAGE2CVSEQ(image_smask);scanny=COUNTOURFIMG(osmask,image_canny);rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);
           EI1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
           EI2=MAP(scanny,osmask,a)  +beta*MAP(osmask,ormask,a);
    energ[i]=((E1-EI1)/E1)+((EI2-E2)/E2);

   cvSetReal2D(image_rmask,y,x-i,0.0);
    cvSetReal2D(image_smask,y,x-i,0.0);
    cvSetReal2D(image_rbina,y,x-i,255.0-cvGetReal2D(image_rbina,y,x-i));
    cvReleaseMemStorage(&(osmask->storage));
    cvReleaseMemStorage(&(ormask->storage));
    cvReleaseMemStorage(&(sarea->storage));
     cvReleaseMemStorage(&(scanny->storage));cvReleaseMemStorage(&(rarea->storage));cvReleaseMemStorage(&(sfront->storage));
}

double maxi=0.0;
int index=-1;
for(int i=0;i<nbpoint;i++){
    maxi=max(maxi,energ[i]);}
for(int i=0;i<nbpoint;i++){
    if(maxi==energ[i]) {index=i;break;}}

for(int i=0;i<=index;i++){
    cvSetReal2D(image_rmask,y,x-i,255.0);
    cvSetReal2D(image_smask,y,x-i,255.0);
    cvSetReal2D(image_rbina,y,x-i,seedvalue);

}
for(int i=0;i<nbpoint;i++){CvPoint *pnt=new CvPoint();
    pnt->x=x-i;pnt->y=y;
    REMOVECVPOINTFCVSEQ(LRNS, pnt);
nb--;//free(pnt);
}

  }
  else if(direct==1){
      int x=point->x;
      int y=point->y;
  long nbpoint=1;
      while(y-1>=0&&cvGetReal2D(image_rmask,y-1,x)==255.0){
   nbpoint++;
   y=y-1;
  }
      double *energ= new double[nbpoint];
      //double *lastvalue= new double[nbpoint];
     //
       x=point->x;
       y=point->y;
      //for(int i=0;i<nbpoint;i++){lastvalue[i]=cvGetReal2D(image_rbina,y-i,x);}
       double E1,E2;
       osmask=FINDOUTSIDECONTOUR(image_smask);
        ormask=FINDOUTSIDECONTOUR(image_rmask);
       sarea=IPLIMAGE2CVSEQ(image_smask);
      rarea=IPLIMAGE2CVSEQ(image_rmask);
        scanny=COUNTOURFIMG(osmask,image_canny);
       sfront=FRONTIEREIPLIMAGE(image_smask);

       E1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
       E2=MAP(scanny,osmask,a)+beta*MAP(osmask,ormask,a);
      cvReleaseMemStorage(&(osmask->storage));
      cvReleaseMemStorage(&(ormask->storage));
       cvReleaseMemStorage(&(sarea->storage));
       cvReleaseMemStorage(&(rarea->storage));
        cvReleaseMemStorage(&(scanny->storage));
        cvReleaseMemStorage(&(sfront->storage));
 for(int i=0;i<nbpoint;i++){
cvSetReal2D(image_smask,y-i,x,255.0);}

for(int i=nbpoint-1;i>=0;i--){double EI1,EI2;
    osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);sarea=IPLIMAGE2CVSEQ(image_smask);scanny=COUNTOURFIMG(osmask,image_canny);rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);
           EI1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
           EI2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
    energ[i]=((E1-EI1)/E1)+((EI2-E2)/E2);
    cvSetReal2D(image_rmask,y-i,x,0.0);
    cvSetReal2D(image_smask,y-i,x,0.0);
    cvSetReal2D(image_rbina,y-i,x,255.0-cvGetReal2D(image_rbina,y-i,x));
    cvReleaseMemStorage(&(osmask->storage));
    cvReleaseMemStorage(&(ormask->storage));
    cvReleaseMemStorage(&(sarea->storage));
     cvReleaseMemStorage(&(scanny->storage));cvReleaseMemStorage(&(rarea->storage));cvReleaseMemStorage(&(sfront->storage));
}
double maxi=0.0;
int index=-1;
for(int i=0;i<nbpoint;i++){
    maxi=max(maxi,energ[i]);}
for(int i=0;i<nbpoint;i++){
    if(maxi==energ[i]) {index=i;break;}}

for(int i=0;i<=index;i++){
    cvSetReal2D(image_rmask,y-i,x,255.0);
    cvSetReal2D(image_smask,y-i,x,255.0);
    cvSetReal2D(image_rbina,y-i,x,seedvalue);

}
for(int i=0;i<nbpoint;i++){CvPoint *pnt=new CvPoint();
    pnt->x=x;pnt->y=y-i;
    REMOVECVPOINTFCVSEQ(LRNS, pnt);
nb--;//free(pnt);
}
}
  else if(direct==2){

      int x=point->x;
      int y=point->y;
  long nbpoint=1;
      while(x+1<isrc_gray->width&&cvGetReal2D(image_rmask,y,x+1)==255.0){
   nbpoint++;
   x=x+1;
  }
      double *energ= new double[nbpoint];
      //double *lastvalue= new double[nbpoint];
     //
       x=point->x;
       y=point->y;
      //for(int i=0;i<nbpoint;i++){lastvalue[i]=cvGetReal2D(image_rbina,y,x+i);}
      double E1,E2;
      osmask=FINDOUTSIDECONTOUR(image_smask);
       ormask=FINDOUTSIDECONTOUR(image_rmask);
      sarea=IPLIMAGE2CVSEQ(image_smask);
     rarea=IPLIMAGE2CVSEQ(image_rmask);
       scanny=COUNTOURFIMG(osmask,image_canny);
      sfront=FRONTIEREIPLIMAGE(image_smask);

             E1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
             E2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
       cvReleaseMemStorage(&(osmask->storage));
       cvReleaseMemStorage(&(ormask->storage));
        cvReleaseMemStorage(&(sarea->storage));
        cvReleaseMemStorage(&(rarea->storage));
         cvReleaseMemStorage(&(scanny->storage));
         cvReleaseMemStorage(&(sfront->storage));
 for(int i=0;i<nbpoint;i++){
cvSetReal2D(image_smask,y,x+i,255.0);}

for(int i=nbpoint-1;i>=0;i--){double EI1,EI2;
   osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);sarea=IPLIMAGE2CVSEQ(image_smask);scanny=COUNTOURFIMG(osmask,image_canny);rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);
           EI1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
           EI2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
    energ[i]=((E1-EI1)/E1)+((EI2-E2)/E2);
    cvSetReal2D(image_rmask,y,x+i,0.0);
    cvSetReal2D(image_smask,y,x+i,0.0);
    cvSetReal2D(image_rbina,y,x+i,255.0-cvGetReal2D(image_rbina,y,x+i));
    cvReleaseMemStorage(&(osmask->storage));
    cvReleaseMemStorage(&(ormask->storage));
    cvReleaseMemStorage(&(sarea->storage));
     cvReleaseMemStorage(&(scanny->storage));cvReleaseMemStorage(&(rarea->storage));cvReleaseMemStorage(&(sfront->storage));
}
double maxi=0.0;
int index=-1;
for(int i=0;i<nbpoint;i++){
    maxi=max(maxi,energ[i]);}
for(int i=0;i<nbpoint;i++){
    if(maxi==energ[i]) {index=i;break;}}

for(int i=0;i<=index;i++){
    cvSetReal2D(image_rmask,y,x+i,255.0);
    cvSetReal2D(image_smask,y,x+i,255.0);
    cvSetReal2D(image_rbina,y,x+i,seedvalue);

}
for(int i=0;i<nbpoint;i++){CvPoint *pnt=new CvPoint();
    pnt->x=x+i;pnt->y=y;
    REMOVECVPOINTFCVSEQ(LRNS, pnt);
nb--;//free(pnt);
}

  }
  else {
      int x=point->x;
      int y=point->y;
  long nbpoint=1;
      while(y+1<isrc_gray->height&&cvGetReal2D(image_rmask,y+1,x)==255.0){
   nbpoint++;
   y=y+1;
  }
      double *energ= new double[nbpoint];
     // double *lastvalue= new double[nbpoint];
     //
       x=point->x;
       y=point->y;
    //  for(int i=0;i<nbpoint;i++){lastvalue[i]=cvGetReal2D(image_rbina,y+i,x);}
      double E1,E2;
      osmask=FINDOUTSIDECONTOUR(image_smask);
       ormask=FINDOUTSIDECONTOUR(image_rmask);
      sarea=IPLIMAGE2CVSEQ(image_smask);
     rarea=IPLIMAGE2CVSEQ(image_rmask);
       scanny=COUNTOURFIMG(osmask,image_canny);
      sfront=FRONTIEREIPLIMAGE(image_smask);

             E1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
             E2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
       cvReleaseMemStorage(&(osmask->storage));
       cvReleaseMemStorage(&(ormask->storage));
        cvReleaseMemStorage(&(sarea->storage));
        cvReleaseMemStorage(&(rarea->storage));
         cvReleaseMemStorage(&(scanny->storage));
         cvReleaseMemStorage(&(sfront->storage));
 for(int i=0;i<nbpoint;i++){
cvSetReal2D(image_smask,y+i,x,255.0);}

for(int i=nbpoint-1;i>=0;i--){double EI1,EI2;
    osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);sarea=IPLIMAGE2CVSEQ(image_smask);scanny=COUNTOURFIMG(osmask,image_canny);rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);
           EI1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
           EI2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
           energ[i]=((E1-EI1)/E1)+((EI2-E2)/E2);
    cvSetReal2D(image_rmask,y+i,x,0.0);
    cvSetReal2D(image_smask,y+i,x,0.0);
    cvSetReal2D(image_rbina,y+i,x,255.0-cvGetReal2D(image_rbina,y+i,x));
    cvReleaseMemStorage(&(osmask->storage));
    cvReleaseMemStorage(&(ormask->storage));
    cvReleaseMemStorage(&(sarea->storage));
     cvReleaseMemStorage(&(scanny->storage));cvReleaseMemStorage(&(rarea->storage));cvReleaseMemStorage(&(sfront->storage));
}
double maxi=0.0;
int index=-1;
for(int i=0;i<nbpoint;i++){
    maxi=max(maxi,energ[i]);}
for(int i=0;i<nbpoint;i++){
    if(maxi==energ[i]) {index=i;break;}}

for(int i=0;i<=index;i++){
    cvSetReal2D(image_rmask,y+i,x,255.0);
    cvSetReal2D(image_smask,y+i,x,255.0);
    cvSetReal2D(image_rbina,y+i,x,seedvalue);

}
for(int i=0;i<nbpoint;i++){CvPoint *pnt=new CvPoint();
    pnt->x=x;pnt->y=y+i;
    REMOVECVPOINTFCVSEQ(LRNS, pnt);
nb--;//free(pnt);
}

  }
//free(point);
cvReleaseMemStorage(&(outsidesnake->storage));
}
    cvReleaseMemStorage(&(LRNS->storage));
//////////////////////////////////////////////////////////////////////////////////

    nb=LSNR->total;
qDebug() << "Liste des points qui n'appartient pas à la région d'interêt situés à l'intérieur du contour";
   // while(LSNR->total!=0){
while(nb>0){CvSeq *outsideregion=FINDOUTSIDECONTOUR(image_rmask);
  CvPoint *point= PPP(LSNR,outsideregion);
  long direct=  BESTDIRECT(image_smask,point,image_rmask);
    qDebug() << nb;
  if(direct==-1){
      REMOVECVPOINTFCVSEQ(LSNR,point);
      nb--;
      cvSetReal2D(image_smask,point->y,point->x,0.0);
      //cvSetReal2D(image_rbina,point->y,point->x,255.0-cvGetReal2D(image_rbina,point->y,point->x));

  }
  else if(direct==0){

      int x=point->x;
      int y=point->y;
  long nbpoint=1;
      while(x-1>=0&&cvGetReal2D(image_smask,y,x-1)==255.0){
   nbpoint++;
   x=x-1;
  }
      double *energ= new double[nbpoint];
     // double *lastvalue= new double[nbpoint];
     //
       x=point->x;
       y=point->y;
     // for(int i=0;i<nbpoint;i++){lastvalue[i]=cvGetReal2D(image_rbina,y,x-i);}
      double E1,E2;
      osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);sarea=IPLIMAGE2CVSEQ(image_smask);scanny=COUNTOURFIMG(osmask,image_canny);rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);

             E1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
             E2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
             cvReleaseMemStorage(&(osmask->storage));
             cvReleaseMemStorage(&(ormask->storage));
             cvReleaseMemStorage(&(sarea->storage));
              cvReleaseMemStorage(&(scanny->storage));cvReleaseMemStorage(&(rarea->storage));cvReleaseMemStorage(&(sfront->storage));
for(int i=0;i<nbpoint;i++){
cvSetReal2D(image_rmask,y,x-i,255.0);
cvSetReal2D(image_rbina,y,x-i,seedvalue);}

for(int i=nbpoint-1;i>=0;i--){double EI1,EI2;
    osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);sarea=IPLIMAGE2CVSEQ(image_smask);scanny=COUNTOURFIMG(osmask,image_canny);rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);
           EI1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
           EI2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
           energ[i]=((E1-EI1)/E1)+((EI2-E2)/E2);
    cvSetReal2D(image_rmask,y,x-i,0.0);
    cvSetReal2D(image_smask,y,x-i,0.0);
    cvSetReal2D(image_rbina,y,x-i,255.0-cvGetReal2D(image_rbina,y,x-i));
    cvReleaseMemStorage(&(osmask->storage));
    cvReleaseMemStorage(&(ormask->storage));
    cvReleaseMemStorage(&(sarea->storage));
     cvReleaseMemStorage(&(scanny->storage));
     cvReleaseMemStorage(&(rarea->storage));
     cvReleaseMemStorage(&(sfront->storage));
}

double maxi=0.0;
int index=-1;
for(int i=0;i<nbpoint;i++){
    maxi=max(maxi,energ[i]);}
for(int i=0;i<nbpoint;i++){
    if(maxi==energ[i]) {index=i;break;}}

for(int i=0;i<=index;i++){
    cvSetReal2D(image_rmask,y,x-i,255.0);
    cvSetReal2D(image_smask,y,x-i,255.0);
    cvSetReal2D(image_rbina,y,x-i,seedvalue);

}
for(int i=0;i<nbpoint;i++){CvPoint *pnt=new CvPoint();
    pnt->x=x-i;pnt->y=y;
    REMOVECVPOINTFCVSEQ(LSNR, pnt);
nb--;//free(pnt);
}


  }
  else if(direct==1){
      int x=point->x;
      int y=point->y;
  long nbpoint=1;
      while(y-1>=0&&cvGetReal2D(image_smask,y-1,x)==255.0){
   nbpoint++;
   y=y-1;
  }
      double *energ= new double[nbpoint];
      //double *lastvalue= new double[nbpoint];
     //
       x=point->x;
       y=point->y;
      //for(int i=0;i<nbpoint;i++){lastvalue[i]=cvGetReal2D(image_rbina,y-i,x);}
      double E1,E2;
      osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);sarea=IPLIMAGE2CVSEQ(image_smask);scanny=COUNTOURFIMG(osmask,image_canny);rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);
      E1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
      E2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
      cvReleaseMemStorage(&(osmask->storage));
      cvReleaseMemStorage(&(ormask->storage));
      cvReleaseMemStorage(&(sarea->storage));
       cvReleaseMemStorage(&(scanny->storage));cvReleaseMemStorage(&(rarea->storage));cvReleaseMemStorage(&(sfront->storage));

      for(int i=0;i<nbpoint;i++){
cvSetReal2D(image_rmask,y-i,x,255.0);
cvSetReal2D(image_rbina,y-i,x,seedvalue);}

for(int i=nbpoint-1;i>=0;i--){double EI1,EI2;
    osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);sarea=IPLIMAGE2CVSEQ(image_smask);scanny=COUNTOURFIMG(osmask,image_canny);rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);
           EI1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
           EI2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
           energ[i]=((E1-EI1)/E1)+((EI2-E2)/E2);
    cvSetReal2D(image_rmask,y-i,x,0.0);
    cvSetReal2D(image_smask,y-i,x,0.0);
    cvSetReal2D(image_rbina,y-i,x,255.0-cvGetReal2D(image_rbina,y-i,x));
    cvReleaseMemStorage(&(osmask->storage));
    cvReleaseMemStorage(&(ormask->storage));
    cvReleaseMemStorage(&(sarea->storage));
     cvReleaseMemStorage(&(scanny->storage));cvReleaseMemStorage(&(rarea->storage));cvReleaseMemStorage(&(sfront->storage));
}
double maxi=0.0;
int index=-1;
for(int i=0;i<nbpoint;i++){
    maxi=max(maxi,energ[i]);}
for(int i=0;i<nbpoint;i++){
    if(maxi==energ[i]) {index=i;break;}}

for(int i=0;i<=index;i++){
    cvSetReal2D(image_rmask,y-i,x,255.0);
    cvSetReal2D(image_smask,y-i,x,255.0);
    cvSetReal2D(image_rbina,y-i,x,seedvalue);

}
for(int i=0;i<nbpoint;i++){CvPoint *pnt=new CvPoint();
    pnt->x=x;pnt->y=y-i;
    REMOVECVPOINTFCVSEQ(LSNR, pnt);
nb--;//free(pnt);
}
}
  else if(direct==2){

      int x=point->x;
      int y=point->y;
  long nbpoint=1;
      while(x+1<isrc_gray->width&&cvGetReal2D(image_smask,y,x+1)==255.0){
   nbpoint++;
   x=x+1;
  }
      double *energ= new double[nbpoint];
      //double *lastvalue= new double[nbpoint];
     //
       x=point->x;
       y=point->y;
      //for(int i=0;i<nbpoint;i++){lastvalue[i]=cvGetReal2D(image_rbina,y,x+i);}
      double E1,E2;
     osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);sarea=IPLIMAGE2CVSEQ(image_smask);scanny=COUNTOURFIMG(osmask,image_canny);rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);

             E1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
             E2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
             cvReleaseMemStorage(&(osmask->storage));
             cvReleaseMemStorage(&(ormask->storage));
             cvReleaseMemStorage(&(sarea->storage));
              cvReleaseMemStorage(&(scanny->storage));cvReleaseMemStorage(&(rarea->storage));cvReleaseMemStorage(&(sfront->storage));
for(int i=0;i<nbpoint;i++){
cvSetReal2D(image_rmask,y,x+i,255.0);
cvSetReal2D(image_rbina,y,x+i,seedvalue);}

for(int i=nbpoint-1;i>=0;i--){double EI1,EI2;
    osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);sarea=IPLIMAGE2CVSEQ(image_smask);scanny=COUNTOURFIMG(osmask,image_canny);rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);
           EI1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
           EI2=MAP(scanny,osmask,a)   +beta*MAP(osmask,ormask,a);
           energ[i]=((E1-EI1)/E1)+((EI2-E2)/E2);
    cvSetReal2D(image_rmask,y,x+i,0.0);
    cvSetReal2D(image_smask,y,x+i,0.0);
    cvSetReal2D(image_rbina,y,x+i,255.0-cvGetReal2D(image_rbina,y,x+i));
    cvReleaseMemStorage(&(osmask->storage));
    cvReleaseMemStorage(&(ormask->storage));
    cvReleaseMemStorage(&(sarea->storage));
     cvReleaseMemStorage(&(scanny->storage));cvReleaseMemStorage(&(rarea->storage));cvReleaseMemStorage(&(sfront->storage));
}
double maxi=0.0;
int index=-1;
for(int i=0;i<nbpoint;i++){
    maxi=max(maxi,energ[i]);}
for(int i=0;i<nbpoint;i++){
    if(maxi==energ[i]) {index=i;break;}}

for(int i=0;i<=index;i++){
    cvSetReal2D(image_rmask,y,x+i,255.0);
    cvSetReal2D(image_smask,y,x+i,255.0);
    cvSetReal2D(image_rbina,y,x+i,seedvalue);

}
for(int i=0;i<nbpoint;i++){CvPoint *pnt=new CvPoint();
    pnt->x=x+i;pnt->y=y;
    REMOVECVPOINTFCVSEQ(LSNR, pnt);
nb--;//free(pnt);
}

  }
  else {
      int x=point->x;
      int y=point->y;
  long nbpoint=1;
      while(y+1<isrc_gray->height&&cvGetReal2D(image_smask,y+1,x)==255.0){
   nbpoint++;
   y=y+1;
  }
      double *energ= new double[nbpoint];
      //double *lastvalue= new double[nbpoint];
     //
       x=point->x;
       y=point->y;
      //for(int i=0;i<nbpoint;i++){lastvalue[i]=cvGetReal2D(image_rbina,y+i,x);}
      double E1,E2;
     osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);sarea=IPLIMAGE2CVSEQ(image_smask);scanny=COUNTOURFIMG(osmask,image_canny);rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);
             E1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
          E2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
          cvReleaseMemStorage(&(osmask->storage));
          cvReleaseMemStorage(&(ormask->storage));
          cvReleaseMemStorage(&(sarea->storage));
           cvReleaseMemStorage(&(scanny->storage));cvReleaseMemStorage(&(rarea->storage));cvReleaseMemStorage(&(sfront->storage));

for(int i=0;i<nbpoint;i++){
cvSetReal2D(image_rmask,y+i,x,255.0);
cvSetReal2D(image_rbina,y+i,x,seedvalue);}

for(int i=nbpoint-1;i>=0;i--){double EI1,EI2;
    osmask=FINDOUTSIDECONTOUR(image_smask);ormask=FINDOUTSIDECONTOUR(image_rmask);sarea=IPLIMAGE2CVSEQ(image_smask);scanny=COUNTOURFIMG(osmask,image_canny);rarea=IPLIMAGE2CVSEQ(image_rmask);sfront=FRONTIEREIPLIMAGE(image_smask);
           EI1=CFPF1(isrc_gray,image_rbina,lamda,rarea)+alpha*CSPF1(image_rbina,sarea,sfront);
           EI2=MAP(scanny,osmask,a) +beta*MAP(osmask,ormask,a);
           energ[i]=((E1-EI1)/E1)+((EI2-E2)/E2);
    cvSetReal2D(image_rmask,y+i,x,0.0);
    cvSetReal2D(image_smask,y+i,x,0.0);
    cvSetReal2D(image_rbina,y+i,x,255.0-cvGetReal2D(image_rbina,y+i,x));
    cvReleaseMemStorage(&(osmask->storage));
    cvReleaseMemStorage(&(ormask->storage));
    cvReleaseMemStorage(&(sarea->storage));
     cvReleaseMemStorage(&(scanny->storage));cvReleaseMemStorage(&(rarea->storage));cvReleaseMemStorage(&(sfront->storage));
}
double maxi=0.0;
int index=-1;
for(int i=0;i<nbpoint;i++){
    maxi=max(maxi,energ[i]);}
for(int i=0;i<nbpoint;i++){
    if(maxi==energ[i]) {index=i;break;}}

for(int i=0;i<=index;i++){
    cvSetReal2D(image_rmask,y+i,x,255.0);
    cvSetReal2D(image_smask,y+i,x,255.0);
    cvSetReal2D(image_rbina,y+i,x,seedvalue);

}
for(int i=0;i<nbpoint;i++){CvPoint *pnt=new CvPoint();
    pnt->x=x;pnt->y=y+i;
    REMOVECVPOINTFCVSEQ(LSNR, pnt);
nb--;//free(pnt);
}  }
  //free(point);
cvReleaseMemStorage(&(outsideregion->storage));
}
cvReleaseMemStorage(&(LSNR->storage));
//
//image_rmask=cvCloneImage(image_smask);
}*/





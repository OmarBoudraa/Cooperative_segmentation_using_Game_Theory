#include "statistique.h"
IplImage*  TadRGB2GRIS(IplImage *src){IplImage* img_nvg=cvCreateImage(cvGetSize(src), src->depth,1);
                   int flip=0;
                   if(src->origin!=IPL_ORIGIN_TL){
                       flip=CV_CVTIMG_FLIP;
                   }
                   cvConvertImage(src, img_nvg, flip);
                                   return img_nvg;}
IplImage*  TadEtireHist(IplImage *src){
IplImage* imgHistogram =0;
         IplImage* gray= 0;
         CvHistogram* hist;
             int bins = 256;
             int hsize[] = {bins};
             //max and min value of the histogram
           float max_value = 0, min_value = 0;int max_idx,min_idx;
             //value and normalized value
            float value;
             int normalized;
             //ranges - grayscale 0 to 256
             float xranges[] = { 0, 256 };
             float* ranges[] = { xranges };
             //create an 8 bit single channel image to hold a
             //grayscale version of the original picture
             gray = cvCreateImage( cvGetSize(src), 8, 1 );
             gray =TadRGB2GRIS(src);
             //Create 3 windows to show the results
             //cvNamedWindow("original",1);
             //cvNamedWindow("gray",1);
             //planes to obtain the histogram, in this case just one
             IplImage* planes[] = { gray };
             //get the histogram and some info about it
             hist = cvCreateHist( 1, hsize, CV_HIST_ARRAY, ranges,1);
             cvCalcHist( planes, hist, 0, NULL);
             cvGetMinMaxHistValue( hist, &min_value, &max_value,&min_idx,&max_idx);
             //create an 8 bits single channel image to hold the histogram
             //paint it white
             imgHistogram = cvCreateImage(cvSize(bins, 300),8,1);
            cvRectangle(imgHistogram, cvPoint(0,0), cvPoint(256,50), CV_RGB(255,255,255),-1);
             //draw the histogram
             for(int i=0; i < bins; i++){
                     value = cvQueryHistValue_1D( hist, i);
                     normalized = cvRound((value-min_value)*50/(max_value-min_value));
                     cvLine(imgHistogram,cvPoint(i,50), cvPoint(i,50-normalized), CV_RGB(0,0,0));}
             for( int i = 0; i <imgHistogram->width; i++)
                         {for(int  j = 50; j < imgHistogram->height; j++)
                             {
                         cvSetReal2D(imgHistogram,j,i,255);}}

  int size=gray->width*gray->height;
             QString temp="*||QUELQUES INFORMATIONS||*";
  CvFont font;
     cvInitFont(&font, 5, 0.7, 0.7, 0, 1, CV_AA);
     cvPutText(imgHistogram, temp.toUtf8(), cvPoint(5, 90), &font, cvScalar(0,0, 0, 0));
     temp="-Inf=";
    QString  val=QString::number(min_idx);
   temp+=val;
  cvInitFont(&font, 5, 0.7, 0.7, 0, 1, CV_AA);
  cvPutText(imgHistogram, temp.toUtf8(), cvPoint(5, 120), &font, cvScalar(0,0, 0, 0));
  temp="-Sup=";
   val=QString::number(max_idx);
  temp+=val;
  cvInitFont(&font, 5, 0.7, 0.7, 0, 1, CV_AA);
  cvPutText(imgHistogram, temp.toUtf8(), cvPoint(5, 140), &font, cvScalar(0,0, 0, 0));
      temp="-Moyenne=";
      val=QString::number(TadMoyenne(gray, 0, 0, gray->width, gray->height));
    temp+=val;
  cvInitFont(&font, 5, 0.7, 0.7, 0, 1, CV_AA);
  cvPutText(imgHistogram, temp.toUtf8(), cvPoint(5, 160), &font, cvScalar(0,0, 0, 0));
  temp="-Variance=";
  val=QString::number(TadVariance(gray, 0, 0, gray->width, gray->height));
  temp+=val;
  cvInitFont(&font, 5, 0.7, 0.7, 0, 1, CV_AA);
  cvPutText(imgHistogram, temp.toUtf8(), cvPoint(5, 180), &font, cvScalar(0,0, 0, 0));
  temp="-Valeur minimale=";
  val=QString::number(min_value);
  temp+=val;
  cvInitFont(&font, 5, 0.7, 0.7, 0, 1, CV_AA);
  cvPutText(imgHistogram, temp.toUtf8(), cvPoint(5, 200), &font, cvScalar(0,0, 0, 0));
  temp="-Valeur maximale=";
  val=QString::number(max_value);
  temp+=val;
  cvInitFont(&font, 5, 0.7, 0.7, 0, 1, CV_AA);
  cvPutText(imgHistogram, temp.toUtf8(), cvPoint(5, 220), &font, cvScalar(0,0, 0, 0));
  temp="-Largeur=";
  val=QString::number(src->width);
  temp+=val;
  cvInitFont(&font, 5, 0.7, 0.7, 0, 1, CV_AA);
  cvPutText(imgHistogram, temp.toUtf8(), cvPoint(5, 240), &font, cvScalar(0,0, 0, 0));
  temp="-Hauteur=";
  val=QString::number(src->height);
  temp+=val;
  cvInitFont(&font, 5, 0.7, 0.7, 0, 1, CV_AA);
  cvPutText(imgHistogram, temp.toUtf8(), cvPoint(5, 260), &font, cvScalar(0,0, 0, 0));
  temp="-Pixels=";
  val=QString::number(size);
  temp+=val;
  cvInitFont(&font, 5, 0.7, 0.7, 0, 1, CV_AA);
  cvPutText(imgHistogram, temp.toUtf8(), cvPoint(5, 280), &font, cvScalar(0,0, 0, 0));
return imgHistogram;}
double TadMoyenne(IplImage *src, int x, int y, int width, int height){
  int size=width*height;
  float mean=0;
  for( int i = x; i <x+width; i++)
          {for(int  j = y; j < y+height; j++)
              { mean+=cvGetReal2D(src,j,i);}}
  return (double) (mean/size);
}
QImage TadIplImage2QImage(IplImage *cvimage)
{

    int h = cvimage->height;
    int w = cvimage->width;
    int channels = cvimage->nChannels;
    QImage qimg(w, h, QImage::Format_ARGB32);
    char *data =   cvimage->imageData;

    for (int y = 0; y < h; y++, data += cvimage->widthStep)
    {
    for (int x = 0; x < w; x++)
    {
    char r=0, g=0, b=0, a =0;
    if (channels == 1)
    {
    r = data[x * channels];
    g = data[x * channels];
    b = data[x * channels];
    }
    else if (channels == 3 || channels == 4)
    {
    r = data[x * channels + 2];
    g = data[x * channels + 1];
    b = data[x * channels];
    }

    if (channels == 4)
    {
    a = data[x * channels + 3];
    qimg.setPixel(x, y, qRgba(r, g, b, a));
    }
    else
    {
    qimg.setPixel(x, y, qRgb(r, g, b));
    }
    }
    }
    return qimg;


}

    IplImage* TadQImage2IplImage(QImage *qimg)
    {

    IplImage *imgHeader = cvCreateImageHeader( cvSize(qimg->width(), qimg->height()), IPL_DEPTH_8U, 4);
    imgHeader->imageData = (char*) qimg->bits();

    uchar* newdata = (uchar*) malloc(sizeof(uchar) * qimg->byteCount());
    memcpy(newdata, qimg->bits(), qimg->byteCount());
    imgHeader->imageData = (char*) newdata;
    //cvClo
    return imgHeader;
    }
    bool TadBinImg(IplImage *src){
        bool bin=true;
    for( int i = 0; i <src->width; i++)
              {for(int  j = 0; j < src->height; j++)
                  {if(cvGetReal2D(src,j,i)!=0&&cvGetReal2D(src,j,i)!=255) bin=false;}}return bin;}

    double TadVariance(IplImage *src, int x, int y, int width, int height){
     int size=width*height;
   float mean=TadMoyenne(src, x, y, width, height);
     float variance=0;
    for( int i = x; i <x+width; i++)
             {for(int  j = y; j < y+height; j++)
                 { variance+=pow(cvGetReal2D(src,j,i),2);}}
     variance/=size;
     variance-=pow(mean,2);
     return variance;}

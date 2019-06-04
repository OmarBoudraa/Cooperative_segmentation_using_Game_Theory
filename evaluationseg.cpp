#include "evaluationseg.h"
//Max Value Iplimage
double MAXIPIMAGE(IplImage *img){
    double maxvalue=0.0;
    for(int i=0;i<img->height;i++){
        for(int j=0;j<img->width;j++){
            if(cvGetReal2D(img,i,j)>maxvalue){maxvalue=cvGetReal2D(img,i,j);}

        }}
    return maxvalue;
}
//Mean Value Iplimage
double MEANIPLIMAGE(IplImage *img, CvSeq* seq){
    double meanvalue=0.0;
    for(int i=0;i<seq->total;i++){CvPoint *pnt;
pnt=(CvPoint *) cvGetSeqElem(seq,i);
    meanvalue=meanvalue+cvGetReal2D(img,pnt->y,pnt->x);
    //free(pnt);
    }
    meanvalue=meanvalue/seq->total;
    return meanvalue;
}
//Frontière Externe Iplimage
CvSeq *FRONTIEREIPLIMAGE(IplImage *img){
    CvSeq *outsidecontour=FINDOUTSIDECONTOUR(img);
 //IplImage *cp=CVSEQ2IPLIMAGE(img,outsidecontour);
 CvMemStorage* storage = cvCreateMemStorage(1048576);
 CvSeq *  cvsequence = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );
 int x,y;CvPoint *pnt;CvPoint *pnti=new CvPoint();
 for(int i=0;i<outsidecontour->total;i++){
pnt=(CvPoint *) cvGetSeqElem(outsidecontour,i);
 x=pnt->x;y=pnt->y;
 if(x-1>=0){pnti->x=x-1;pnti->y=y;
    // if(!WHITECONTOURNED(cp,pnti)){
     if(cvGetReal2D(img,pnti->y,pnti->x)==0.0){
         ADDCVPOINT2CVSEQ(cvsequence,pnti);
     }
     //free(pnti);
     }
 if(y-1>=0){pnti->x=x;pnti->y=y-1;
     if(cvGetReal2D(img,pnti->y,pnti->x)==0.0){
         ADDCVPOINT2CVSEQ(cvsequence,pnti);
     }
     //free(pnti);
     }
 if(x+1<img->width){pnti->x=x+1;pnti->y=y;
     if(cvGetReal2D(img,pnti->y,pnti->x)==0.0){
         ADDCVPOINT2CVSEQ(cvsequence,pnti);
     }
//free(pnti);
     }
 if(y+1<img->height){pnti->x=x;pnti->y=y+1;
     if(cvGetReal2D(img,pnti->y,pnti->x)==0.0){
         ADDCVPOINT2CVSEQ(cvsequence,pnti);
     }
         //free(pnti);
     }
 //free(pnt);
 }
 //cvReleaseImage(&cp);
 cvReleaseMemStorage(&(outsidecontour->storage));


 return cvsequence;
 }
//Mesure de Zéboudj
double ZEBOUDJ(IplImage *isrc_gray, CvSeq *region){
    double L=MAXIPIMAGE(isrc_gray);
    double zvalue=0;
//Contraste interne
double ci=0.0;
double maxintraregion=0.0;
long x,y;double m;
    for(int i=0;i<region->total;i++){CvPoint *pnt;
   pnt=(CvPoint *) cvGetSeqElem(region,i);
    x=pnt->x;y=pnt->y;
    maxintraregion=0.0;
    if(x-1>=0){CvPoint *pnti=new CvPoint();pnti->x=x-1;pnti->y=y;
        if(EXISTECVPOINT(region,pnti)){
            m=abs(cvGetReal2D(isrc_gray,pnti->y,pnti->x)-cvGetReal2D(isrc_gray,pnt->y,pnt->x))/(L-1.0);
           if(maxintraregion<m){maxintraregion=m;
        }

        }free(pnti);
    }
    if(y-1>=0){CvPoint *pnti=new CvPoint();pnti->x=x;pnti->y=y-1;
        if(EXISTECVPOINT(region,pnti)){
             m=abs(cvGetReal2D(isrc_gray,pnti->y,pnti->x)-cvGetReal2D(isrc_gray,pnt->y,pnt->x))/(L-1.0);
           if(maxintraregion<m){maxintraregion=m;
        }

        }free(pnti);
    }
    if(x+1<isrc_gray->width){CvPoint *pnti=new CvPoint();pnti->x=x+1;pnti->y=y;
        if(EXISTECVPOINT(region,pnti)){
             m=abs(cvGetReal2D(isrc_gray,pnti->y,pnti->x)-cvGetReal2D(isrc_gray,pnt->y,pnt->x))/(L-1.0);
           if(maxintraregion<m){maxintraregion=m;
        }

        }free(pnti);
    }
    if(y+1<isrc_gray->height){CvPoint *pnti=new CvPoint();pnti->x=x;pnti->y=y+1;
        if(EXISTECVPOINT(region,pnti)){
             m=abs(cvGetReal2D(isrc_gray,pnti->y,pnti->x)-cvGetReal2D(isrc_gray,pnt->y,pnt->x))/(L-1.0);
           if(maxintraregion<m){maxintraregion=m;
        }

        }
         free(pnti);

    } //free(pnt);
    ci=ci+maxintraregion;}
    ci=ci/region->total;
//Contraste externe
    IplImage *img_bin=CVSEQ2IPLIMAGE(isrc_gray,region);
    CvSeq *frontiereinterne=FINDOUTSIDECONTOUR(img_bin);
    CvSeq *frontiereexterne=FRONTIEREIPLIMAGE(img_bin);

    /////

    double ce=0.0;
    double maxinterregion=0.0;
        for(int i=0;i<frontiereinterne->total;i++){CvPoint *pnt;
       pnt=(CvPoint *) cvGetSeqElem(frontiereinterne,i);
        x=pnt->x; y=pnt->y;
        maxinterregion=0.0;
        if(x-1>=0){CvPoint *pnti=new CvPoint();pnti->x=x-1;pnti->y=y;
            if(EXISTECVPOINT(frontiereexterne,pnti)){
                 m=abs(cvGetReal2D(isrc_gray,pnti->y,pnti->x)-cvGetReal2D(isrc_gray,pnt->y,pnt->x))/(L-1.0);
               if(maxinterregion<m){maxinterregion=m;
            }
            free(pnti);
            }}
        if(y-1>=0){CvPoint *pnti=new CvPoint();pnti->x=x;pnti->y=y-1;
            if(EXISTECVPOINT(frontiereexterne,pnti)){
                 m=abs(cvGetReal2D(isrc_gray,pnti->y,pnti->x)-cvGetReal2D(isrc_gray,pnt->y,pnt->x))/(L-1.0);
               if(maxinterregion<m){maxinterregion=m;
            }
            free(pnti);
            }}
        if(x+1<isrc_gray->width){CvPoint *pnti=new CvPoint();pnti->x=x+1;pnti->y=y;
            if(EXISTECVPOINT(frontiereexterne,pnti)){
                 m=abs(cvGetReal2D(isrc_gray,pnti->y,pnti->x)-cvGetReal2D(isrc_gray,pnt->y,pnt->x))/(L-1.0);
               if(maxinterregion<m){maxinterregion=m;
            }
       free(pnti);
            }}
        if(y+1<isrc_gray->height){CvPoint *pnti=new CvPoint();pnti->x=x;pnti->y=y+1;
            if(EXISTECVPOINT(frontiereexterne,pnti)){
                 m=abs(cvGetReal2D(isrc_gray,pnti->y,pnti->x)-cvGetReal2D(isrc_gray,pnt->y,pnt->x))/(L-1.0);
               if(maxinterregion<m){maxinterregion=m;
            }
                free(pnti);
            }

        }
       // free(pnt);
        ce=ce+maxinterregion;}
        ce=ce/frontiereinterne->total;
        /////////////
        if(ci>0.0&&ci<ce){zvalue=1.0-(ci/ce);

        }
        else if(ci==0.0){zvalue=ce;}
        else{zvalue=0.0;}
        cvReleaseImage(&img_bin);
        cvReleaseMemStorage(&frontiereinterne->storage);
        cvReleaseMemStorage(&frontiereexterne->storage);

    return zvalue;
}
//Critère de Borsotti
double borsotti(IplImage *isrc_gray, CvSeq *region ){
double bvalue=0.0;
double mvalue=MEANIPLIMAGE(isrc_gray,region);
for(int i=0;i<region->total;i++){CvPoint *pnt;
pnt=(CvPoint *) cvGetSeqElem(region,i);
bvalue=bvalue+(pow(cvGetReal2D(isrc_gray,pnt->y,pnt->x)-mvalue,2.0));
//free(pnt);
}
bvalue=bvalue/(1.0+log(region->total));
bvalue=bvalue+(1.0/pow(region->total,2.0));

bvalue=bvalue/(10000.0*isrc_gray->width*isrc_gray->height);
//bvalue=(bvalue*1000000)/(region->total);
return bvalue;


}

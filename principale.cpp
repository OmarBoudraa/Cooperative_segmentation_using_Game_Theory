#include "principale.h"
#include "ui_principale.h"
#include <QtWidgets>
#ifndef QT_NO_PRINTER
#include <QPrintDialog>
#endif
principale::principale(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::principale)
{
    ui->setupUi(this);
        ui->imageLabel = new QLabel;
        ui->imageLabel->setBackgroundRole(QPalette::Base);
        ui->imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        ui->imageLabel->setScaledContents(true);

        ui->scrollArea = new QScrollArea;
        ui->scrollArea->setBackgroundRole(QPalette::Dark);
        ui->scrollArea->setWidget(ui->imageLabel);
        setCentralWidget(ui->scrollArea);
        setWindowTitle(tr("Ségmentation coopérative en utilisant la théorie des jeux"));
        resize(500, 400);


   /*     QVariant t=(event->pos().x());QVariant g=(event->pos().y());
            QMessageBox::about(this, tr(t.toString().toStdString().c_str()),
                              tr(g.toString().toStdString().c_str()));
        printf("gg");*/

}

principale::~principale()
{
    delete ui;
}
void principale::mousePressEvent(QMouseEvent *event)
{

    if (event->button() == Qt::LeftButton) {if(ui->actionCroissance_de_r_gion->isEnabled()){
            QPoint dragPosition;
               if(ui->actionAjuster_la_fen_tre->isChecked()){
      dragPosition = (event->pos()-QPoint(1,(21)));
   double facteurx,facteury;
   facteurx=dragPosition.x();
   facteurx/=(this->width()-1);
   facteurx*=this->image_source->width;

   facteury=dragPosition.y();
   facteury/=(this->height()-21);
   facteury*=image_source->height;

   event->accept();dragPosition.setX(round(facteurx));dragPosition.setY(round(facteury));
   }

               else{
      dragPosition = (event->pos()-QPoint(1,(21))+QPoint(ui->scrollArea->horizontalScrollBar()->value(),ui->scrollArea->verticalScrollBar()->value()))/scaleFactor;
     event->accept();}
               if(dragPosition.y()<image_source->height&&dragPosition.x()<image_source->width){
this->seed->x= dragPosition.x();
this->seed->y= dragPosition.y();

       }

       }else if((event->pos().y()>(20))){QMessageBox::information(this, tr("Ségmentation coopérative en utilisant la théorie des jeux"),
                                      tr("Quelques tâches nécessaires non encore faites."));}
       }
}


void principale::on_actionOuvrir_triggered()
{



    fileName = QFileDialog::getOpenFileName(this,tr("Ouvrir une image"), QDir::currentPath(),tr("Images (*.png *.tiff *.bmp *.jpg *.dib *.jpeg *.jpe *.pbm *.pgm *.sr *.ras *.tiff *.tif)"));

    if (!fileName.isEmpty()) {

        image_source=cvLoadImage(fileName.toUtf8(), CV_LOAD_IMAGE_COLOR);   // Read the file

        QImage image(fileName);
       if (image.isNull()&&!image_source) {
            QMessageBox::information(this, tr("Ségmentation coopérative en utilisant la théorie des jeux"),
                                     tr("On peut pas charger %1.").arg(fileName));
           return;
        }
        if (image.isNull()) {
image=TadIplImage2QImage(image_source);
       }
       if (!image_source) {
image_source=TadQImage2IplImage(&image);
       }

//! [2] //! [3]
       ui->imageLabel->setPixmap(QPixmap::fromImage(image));
       scaleFactor = 1.0;
       ui->actionSauvegarder->setEnabled(true);
       ui->actionImprimer->setEnabled(true);
       ui->actionGris->setEnabled(true);
ui->actionHistogramme->setEnabled(false);

ui->actionBruitage_Bruit_Gaussien_ajout->setEnabled(false);
ui->actionBruitage_Bruit_de_type_poivre_et_sel_5->setEnabled(false);
ui->actionD_bruitage_Filtre_Moyenneur_3x3->setEnabled(false);
ui->actionD_bruitage_Filtre_Gaussien_3x3->setEnabled(false);
ui->actionD_bruitage_Filtre_M_dian_3x3->setEnabled(false);
ui->actionCanny->setEnabled(false);
ui->actionContours_Actifs->setEnabled(false);
 ui->actionCroissance_de_r_gion->setEnabled(false);
 ui->actionS_gmentation_coop_rative_TG->setEnabled(false);
ui->actionEvaluation_des_r_sultats_de_segmentaion->setEnabled(false);
       ui->actionAjuster_la_fen_tre->setEnabled(true);

listreg=0;listbruit=0;
image_rmask=0;
image_smask=0;
image_canny=0;


       updateActions();

       if (!ui->actionAjuster_la_fen_tre->isChecked())
           ui->imageLabel->adjustSize();

   this->seed = new CvPoint();
       this->seed->x=image_source->width/2;
       this->seed->y=image_source->height/2;




    }
}

void principale::on_actionFermer_triggered()

{
    exit(0);
}


void principale::on_actionImprimer_triggered()
{
    Q_ASSERT(ui->imageLabel->pixmap());
#if !defined(QT_NO_PRINTER) && !defined(QT_NO_PRINTDIALOG)
//! [6] //! [7]
    QPrintDialog dialog(&printer, this);
//! [7] //! [8]
    if (dialog.exec()) {
        QPainter painter(&printer);
        QRect rect = painter.viewport();
        QSize size = ui->imageLabel->pixmap()->size();
        size.scale(rect.size(), Qt::KeepAspectRatio);
        painter.setViewport(rect.x(), rect.y(), size.width(), size.height());
        painter.setWindow(ui->imageLabel->pixmap()->rect());
        painter.drawPixmap(0, 0, *ui->imageLabel->pixmap());
    }
#endif
 }
void principale::updateActions()
//! [21] //! [22]
{
   ui->actionZoomer_25->setEnabled(!ui->actionAjuster_la_fen_tre->isChecked());
    ui->actionD_zoomer_25->setEnabled(!ui->actionAjuster_la_fen_tre->isChecked());
    ui->actionTaille_normale->setEnabled(!ui->actionAjuster_la_fen_tre->isChecked());
}

void principale::on_actionSauvegarder_triggered()
{
QFileDialog dialogsave;QString fileSave=NULL;
     fileSave =dialogsave.getSaveFileName(this,tr("Sauvegarder une image"),QDir::currentPath());
    QFileInfo info(fileName);
           QString extension =info.suffix();
           //fileSave+=info.fileName();
           if(fileSave.isNull()){QMessageBox::information(this, tr("Ségmentation coopérative en utilisant la théorie des jeux"),
                                                          tr("Image non sauvegardée.")); return;  }
           fileSave+="."+extension;

           bool Save=false;
               Save=ui->imageLabel->pixmap()->save(fileSave);

    (!Save)? QMessageBox::information(this, tr("Ségmentation coopérative en utilisant la théorie des jeux"),
                                     tr("On peut pas sauvegarder %1.").arg(fileSave)):QMessageBox::information(this, tr("Ségmentation coopérative en utilisant la théorie des jeux"),
                                                                                                  tr("Image Sauvegardée %1.").arg(fileSave));

}
void principale::on_actionA_propos_de_QT_triggered()
{
    qApp->aboutQt();
}

void principale::on_actionA_propos_de_SCG_triggered()
{
     QMessageBox::about(this, tr("A propos de Segmentation coopérative en utilisant la théorie des jeux"),
             tr("<p>La <b> segmentation d’image </b> est un problème central en traitement d’image. Cela consiste à extraire des objets d’une image (partitionner l’image en plusieurs régions). Il existe dans la littérature plusieurs approches dont la segmentation par région et celle par contour. La segmentation par région consiste à regrouper les pixels similaires en régions, quant à la segmentation par contour, elle identifie les changements entre régions (contours). Dans ces deux types de segmentation, nous trouvons plusieurs méthodes qui peuvent être classées selon leur mode de fonctionnement. Elles présentent toutes des avantages et des inconvénients. Ces dernières années, un nouveau type de segmentation a vu le jour. Les chercheurs se sont intéressés à la coopération des méthodes de segmentation par région et par contour dans le but de pallier à leurs inconvénients respectifs. Dans ce travail, nous nous intéressons à la coopération mutuelle région-contour en utilisant la théorie des jeux.</p>"
));

}
void principale::on_actionZoomer_25_triggered()
{
   scaleImage(1.25);
}
void principale::scaleImage(double factor)
//! [23] //! [24]
{
    Q_ASSERT(ui->imageLabel->pixmap());
    scaleFactor *= factor;
    ui->imageLabel->resize(scaleFactor * ui->imageLabel->pixmap()->size());

    adjustScrollBar(ui->scrollArea->horizontalScrollBar(), factor);
    adjustScrollBar(ui->scrollArea->verticalScrollBar(), factor);

    ui->actionZoomer_25->setEnabled(scaleFactor < 3.0);
    ui->actionD_zoomer_25->setEnabled(scaleFactor > 0.333);
}
void principale::adjustScrollBar(QScrollBar *scrollBar, double factor)
//! [25] //! [26]
{
    scrollBar->setValue(int(factor * scrollBar->value()
                            + ((factor - 1) * scrollBar->pageStep()/2)));
}

void principale::on_actionD_zoomer_25_triggered()
{

    scaleImage(0.8);
}
void principale::on_actionTaille_normale_triggered()
{
    ui->imageLabel->adjustSize();
    scaleFactor = 1.0;
}
void principale::on_actionAjuster_la_fen_tre_triggered()
{
    bool fitToWindow = ui->actionAjuster_la_fen_tre->isChecked();
    ui->scrollArea->setWidgetResizable(fitToWindow);
    if (!fitToWindow) {
       on_actionTaille_normale_triggered();
    }
    updateActions();
}
void principale::on_actionGris_triggered()
{
    image_gris=TadRGB2GRIS(image_source);
    versLabel(image_gris);ui->actionHistogramme->setEnabled(true);
    ui->actionCanny->setEnabled(true);
    ui->actionD_bruitage_Filtre_Gaussien_3x3->setEnabled(true);
    ui->actionD_bruitage_Filtre_Moyenneur_3x3->setEnabled(true);
    ui->actionD_bruitage_Filtre_M_dian_3x3->setEnabled(true);
    ui->actionContours_Actifs->setEnabled(true);
    ui->actionCroissance_de_r_gion->setEnabled(true);
    ui->actionBruitage_Bruit_Gaussien_ajout->setEnabled(true);
    ui->actionBruitage_Bruit_de_type_poivre_et_sel_5->setEnabled(true);
    ui->actionS_gmentation_coop_rative_TG->setEnabled(true);
}
void principale::versLabel(IplImage* src){
QImage image=TadIplImage2QImage(src);
 ui->imageLabel->setPixmap(QPixmap::fromImage(image));}
void principale::on_actionHistogramme_triggered()
{
    image_hist=TadEtireHist(image_source);
    cvShowImage("Histogramme",image_hist);
}

void principale::on_actionCanny_triggered()
{
   this->image_canny= Tadcanny(this->image_gris);
    this->smin=getsmin();
     this->smax=getsmax();
     this->aperature=getaperature();
    versLabel(image_canny);

}

void principale::on_actionD_bruitage_Filtre_Gaussien_3x3_triggered()
{
   // cvNamedWindow( "Image filtree par un filtre Gaussien 3x3" );

    /// Reduce noise with a kernel 3x3
    //blur( src_gray, detected_edges, cvSize(3,3) );
    cvSmooth(this->image_gris, this->image_gris, CV_GAUSSIAN, 3, 3);

   /* cvShowImage("Image filtree par un filtre Gaussien 3x3", image_gris);
     cvWaitKey(0);*/
versLabel(image_gris);
}

void principale::on_actionContours_Actifs_triggered()
{


    snakes *ss= new snakes(this);
    ss->exec();

if(ss->rej) return;
this->siteration=ss->iter;
//qDebug()<<this->siteration;

 pts_snakes=Tadsnake(this->image_gris,this->seed,this->siteration,this->image_canny);
image_canny=getimgcanny();
 image_snakes=getimgsnakes();
length=getlength();
listcanny=getlstcanny();

/*
    cvNamedWindow("Contour actif",0);

cvShowImage("Contour actif",image_snakes);

cvWaitKey(0);*/
versLabel(image_snakes);

}
void principale::on_actionCroissance_de_r_gion_triggered()
{

     /*   boite_dialogue *dialogue = new boite_dialogue(0, this);
        int resultat = dialogue->exec();

        if (resultat == QDialog::Accepted)
        {
            coefficient_diviseur = Dim_image / dialogue->valeur_taille_affichage();
            Dim_image = Dim_image / coefficient_diviseur;
        }

        scroll_bar = true;
        delete dialogue;*/
    croissanceregion *dd= new croissanceregion(this);
    dd->iter=this->image_source->height*image_source->width;
    dd->exec();

if(dd->rej) return;
this->lamda=dd->lamda;
this->riteration=dd->iter;

this->image_crreg=Tadregiongrowing(this->image_gris,this->seed,lamda,riteration);
image_rbin=getimgrbin();
this->listreg=getlistreg();
this->listbruit=getlistbruit();

   /* cvNamedWindow("Croissance de region",0);

cvShowImage("Croissance de region",image_crreg);*/
versLabel(image_crreg);

cvWaitKey(0);
ui->actionEvaluation_des_r_sultats_de_segmentaion->setEnabled(true);

}

void principale::on_actionBruitage_Bruit_Gaussien_ajout_triggered()
{
   // cvNamedWindow( "Image avec bruit gaussien ajoutee" );

    /// add noise with a kernel 3x3
    //blur( src_gray, detected_edges, cvSize(3,3) );
    //cv::Scalar::all(128.0)
    cv::Mat noise(cv::Size(image_gris->width,image_gris->height), CV_8U);
    noise={{0.0}};
    cv::randn(noise, 128.0, 5.0);
    // cv::cvarrToMat(image_gris,true)
   cv::Mat img=cvCloneImage(image_gris);
   cv::add(noise,img,img);
     int x,y;
     for( x=0; x < image_gris->width; x++ ){
       for( y=0; y < image_gris->height; y++) {
         // note: CvMat is indexed (row, column) but IplImage is indexed (x,y)
         // so the indexes must be interchanged!
         cvSetReal2D( image_gris, y, x,(double) img.ptr(y)[x]  );
         //min((double) img.ptr(y)[x],255.0)
       }
     }


  /*  cvShowImage("Image avec bruit gaussien ajoutee", image_gris);
     cvWaitKey(0);*/
versLabel(image_gris);
}

void principale::on_actionS_gmentation_coop_rative_TG_triggered()
{
     GUISCG *gt= new GUISCG(this);
     gt->riter=this->image_source->height*image_source->width;
     gt->exec();
      if(gt->rej) return;
      this->lamda=gt->lamda;
  this->a=gt->a;this->alpha=gt->alpha;this->beta=gt->beta;this->siteration=gt->siter;this->riteration=gt->riter;
 this->image_crreg=Tadregiongrowing(this->image_gris,this->seed,lamda,riteration);
 SIUGT(image_gris,seed,siteration,this->image_canny,riteration,lamda,alpha,beta,a);
 image_rbin=getimgrbin();
 this->listreg=getlistreg();
 this->listbruit=getlistbruit();
image_snakes=getimgsnakes();
length=getlength();
listcanny=getlstcanny();
 image_rmask=getrmask();image_smask=getsmask();
 image_scg=printresgt(image_gris,image_rmask);
/* cvShowImage("Resultat de segmentation coopérative en utilisant la thérie des jeux", image_scg);
  cvWaitKey(0);*/
  versLabel(image_scg);
ui->actionEvaluation_des_r_sultats_de_segmentaion->setEnabled(true);
}

void principale::on_actionEvaluation_des_r_sultats_de_segmentaion_triggered()
{
    double rborsotti=borsotti(image_gris,listreg);
    double rzeboudj=ZEBOUDJ(image_gris,listreg);
    double scborsotti=0.0;
    double sczeboudj=0.0;
    int nbpregionseulement=listreg->total;
    int nbpregionsc=0;
    int nblargeur=image_gris->width;
    int nbhauteur=image_gris->height;
    int nbtotalimage=nblargeur*nbhauteur;
   if(image_rmask!=0){
       CvSeq *listscg=IPLIMAGE2CVSEQ(image_rmask);
    scborsotti=borsotti(image_gris,listscg);
    sczeboudj=ZEBOUDJ(image_gris,listscg);
    nbpregionsc=listscg->total;
    cvReleaseMemStorage(&(listscg->storage));
   }
    guisegeval *gt= new guisegeval(this,QFileInfo(fileName).fileName(),rborsotti,rzeboudj,scborsotti,sczeboudj,nbpregionseulement,nbpregionsc,nblargeur,nbhauteur,nbtotalimage);

    gt->exec();
}

void principale::on_actionD_bruitage_Filtre_M_dian_3x3_triggered()
{
    //cvNamedWindow( "Image filtree par un filtre Median 3x3" );

    /// Reduce noise with a kernel 3x3
    //blur( src_gray, detected_edges, cvSize(3,3) );
    cvSmooth(this->image_gris, this->image_gris, CV_MEDIAN, 3, 3);

   /* cvShowImage("Image filtree par un filtre Median 3x3", image_gris);
     cvWaitKey(0);*/
versLabel(image_gris);
}
void principale::on_actionD_bruitage_Filtre_Moyenneur_3x3_triggered()
{
    //cvNamedWindow( "Image filtree par un filtre Moyenneur 3x3" );

    /// Reduce noise with a kernel 3x3
    //blur( src_gray, detected_edges, cvSize(3,3) );
    cvSmooth(this->image_gris, this->image_gris, CV_BLUR, 3, 3);

 /*   cvShowImage("Image filtree par un filtre Moyenneur 3x3", image_gris);
     cvWaitKey(0);*/
versLabel(image_gris);
}

void principale::on_actionBruitage_Bruit_de_type_poivre_et_sel_5_triggered()
{long n=(this->image_gris->width*this->image_gris->height*5)/100;
    int i,j;
    //Salt
    for(int k=0; k<n; k++)
        {

             i = rand()%image_gris->width;

             j = rand()%image_gris->height;

                cvSetReal2D(image_gris,j,i,255.0);}
    //Pepper
    for(int k=0; k<n; k++)
        {

             i = rand()%image_gris->width;

             j = rand()%image_gris->height;

                cvSetReal2D(image_gris,j,i,0.0);}
/*    cvShowImage("Image bruitee par le bruit impulsionnel de type poivre et sel (5%)", image_gris);
     cvWaitKey(0);*/
versLabel(image_gris);

}

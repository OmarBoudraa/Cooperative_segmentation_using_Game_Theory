#ifndef PRINCIPALE_H
#define PRINCIPALE_H

#include <QMessageBox>
#include <QMainWindow>
#ifndef QT_NO_PRINTER
#include <QPrinter>
#endif

#include <iostream>
//#include <winsock2.h>
//#include <windows.h>
//#include <sqlite3.h>
//#include <stdlib.h>
#include <QMainWindow>
//#include <QPrinter>
#include <QtGui>
#include <stdio.h>
#include <math.h>
#include <QPainterPath>
//#include <stdarg.h>
#include <QApplication>
#include <QPainterPath>
#include <QMainWindow>
//#include <cxtypes.h>
//#include <stdarg.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
//#include <opencv/cxeigen.hpp>
//#include <cvcompat.h>
//#include <cvinternal.h>
//#include <cvtypes.h>
//#include <cvver.h>
//#include <cvvidsurv.hpp>
#include <opencv/cvwimage.h>
//#include <cxerror.h>
//#include <cxmat.hpp>
#include <opencv/cxmisc.h>
//#include <cxoperations.hpp>
//#include <cxflann.h>
#include <opencv/ml.h>
#include <QDockWidget>
//#include <QSqlRecord>
//#include <QtSql>
//#include <QSqlDatabase>
//#include <QSqlError>
//#include <QSqlQuery>
//#include <QVariant>
//#include <QIOdevice>
#include <QFile>
#include <QFileDialog>
//#include <QMessageBox>
//#include <QUndoGroup>
// #include <QUndoStack>
// #include <QFileDialog>

#include "statistique.h"
#include "segmentation.h"
#include "croissanceregion.h"
#include "snakes.h"
#include "guiscg.h"
#include "evaluationseg.h"
#include "guisegeval.h"
/*#include <QThread>
#include <QString>
#include <qtconcurrentrun.h>
#include <qfuture.h>
using namespace QtConcurrent;*/

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
QT_BEGIN_NAMESPACE
class QAction;
class QLabel;
class QMenu;
class QScrollArea;
class QString;
class QPrinter;
class QImage;
class QScrollBar;
//class QPrintDialog;
//class QPainter;
class QPixmap;
QT_END_NAMESPACE
namespace Ui {
class principale;
}

class principale : public QMainWindow
{
    Q_OBJECT

public:
    explicit principale(QWidget *parent = 0);
    ~principale();

private slots:

    void on_actionOuvrir_triggered();

    void on_actionImprimer_triggered();


    void on_actionFermer_triggered();

    void on_actionSauvegarder_triggered();

    void on_actionA_propos_de_QT_triggered();

    void on_actionA_propos_de_SCG_triggered();

    void on_actionZoomer_25_triggered();

    void on_actionD_zoomer_25_triggered();

    void on_actionTaille_normale_triggered();

    void on_actionAjuster_la_fen_tre_triggered();

    void on_actionGris_triggered();

    void on_actionHistogramme_triggered();

    void on_actionCanny_triggered();

    void on_actionD_bruitage_Filtre_Gaussien_3x3_triggered();

    void on_actionContours_Actifs_triggered();
    void on_actionCroissance_de_r_gion_triggered();

    void on_actionBruitage_Bruit_Gaussien_ajout_triggered();

    void on_actionS_gmentation_coop_rative_TG_triggered();

    void on_actionEvaluation_des_r_sultats_de_segmentaion_triggered();

    void on_actionD_bruitage_Filtre_M_dian_3x3_triggered();

    void on_actionD_bruitage_Filtre_Moyenneur_3x3_triggered();

    void on_actionBruitage_Bruit_de_type_poivre_et_sel_5_triggered();

protected:
    void mousePressEvent(QMouseEvent *event);
public :
    Ui::principale *ui;
    void updateActions();
   void versLabel(IplImage* src);
    void scaleImage(double factor);
    void adjustScrollBar(QScrollBar *scrollBar, double factor);
#ifndef QT_NO_PRINTER
    QPrinter printer;
#endif

    double scaleFactor;
    QString fileName;
    IplImage *image_source=0;
    IplImage *image_gris=0;
    IplImage *image_hist=0;
 //Paramètres de la méthode canny
    IplImage *image_canny=0;
    int smin=50;
    int smax=150;
    int aperature=3;
    CvSeq* listcanny;
//Paramètres du snakes
    IplImage *image_snakes=0;
 CvPoint* pts_snakes;
 long  length=0;
 long siteration;
 //Paramètres de la méthode croissance de région
 IplImage *image_crreg=0;
 IplImage *image_rbin=0;
 CvPoint* seed;
 CvSeq* listreg=0;
 CvSeq* listbruit=0;
 double lamda;
long riteration;
//Paramètres de la segmentation coopérative en utilisant la théorie des jeux
IplImage *image_scg=0;
IplImage *image_rmask=0;
IplImage *image_smask=0;
double a=0.11;
double alpha=1.0;
double beta=1.0;
};
#endif // PRINCIPALE_H


















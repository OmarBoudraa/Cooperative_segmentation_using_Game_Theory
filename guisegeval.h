#ifndef GUISEGEVAL_H
#define GUISEGEVAL_H

#include <QDialog>

namespace Ui {
class guisegeval;
}

class guisegeval : public QDialog
{
    Q_OBJECT

public:
    explicit guisegeval(QWidget *parent = 0,QString fn="File Name",double rb=0,double rz=0,double scb=0,double scz=0, int nbprs=0, int nbpsc=0, int nbl=0, int nbh=0, int nbtot=0);
    ~guisegeval();
     double rborsotti=0.0;
     double rzeboudj=0.0;
     double scborsotti=0.0;
     double sczeboudj=0.0;
     int nbpregionseulement=0;
     int nbpregionsc=0;
     int nblargeur=0;
     int nbhauteur=0;
     int nbtotalimage=0;

private slots:
    void on_pushButton_clicked();


public:
    Ui::guisegeval *ui;
};

#endif // GUISEGEVAL_H

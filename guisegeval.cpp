#include "guisegeval.h"
#include "ui_guisegeval.h"

guisegeval::guisegeval(QWidget *parent,QString fn,double rb,double rz,double scb,double scz, int nbprs, int nbpsc, int nbl, int nbh, int nbtot):
    QDialog(parent),
    ui(new Ui::guisegeval)
{

    ui->setupUi(this);
    ui->lineEdit_1->setText(QVariant(rb).toString());
    ui->lineEdit_2->setText(QVariant(rz).toString());
    ui->lineEdit_3->setText(QVariant(scb).toString());
    ui->lineEdit_4->setText(QVariant(scz).toString());
    ui->lineEdit_5->setText(QVariant(nbprs).toString());
    ui->lineEdit_6->setText(QVariant(nbpsc).toString());
    ui->lineEdit_7->setText(QVariant(nbl).toString());
    ui->lineEdit_8->setText(QVariant(nbh).toString());
    ui->lineEdit_9->setText(QVariant(nbtot).toString());
    ui->label_10->setText(fn);
}

guisegeval::~guisegeval()
{
    delete ui;
}

void guisegeval::on_pushButton_clicked()
{
    this->hide();
}



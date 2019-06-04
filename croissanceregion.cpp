#include "croissanceregion.h"
#include "ui_croissanceregion.h"

croissanceregion::croissanceregion(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::croissanceregion)
{
    ui->setupUi(this);

}

croissanceregion::~croissanceregion()
{
    delete ui;
}

void croissanceregion::on_buttonBox_accepted()
{this->lamda=ui->doubleSpinBox->value();
    this->iter=ui->spinBox->value();
    this->hide();
}

void croissanceregion::on_buttonBox_rejected()
{rej=true;
    this->hide();

}

void croissanceregion::on_pushButton_clicked()
{ui->spinBox->setMaximum(this->iter);
   ui->spinBox->setValue(this->iter);
}

void croissanceregion::on_croissanceregion_rejected()
{
    rej=true;
        this->hide();
}

#include "guiscg.h"
#include "ui_guiscg.h"

GUISCG::GUISCG(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GUISCG)
{
    ui->setupUi(this);
}

GUISCG::~GUISCG()
{
    delete ui;
}

void GUISCG::on_pushButton_clicked()
{
    ui->spinBox_2->setMaximum(this->riter);
       ui->spinBox_2->setValue(this->riter);
}

void GUISCG::on_buttonBox_rejected()
{rej=true;
    this->hide();

}

void GUISCG::on_buttonBox_accepted()
{
    this->lamda=ui->doubleSpinBox_1->value();
        this->siter=ui->spinBox_1->value();
    this->riter=ui->spinBox_2->value();
    this->a=ui->doubleSpinBox_2->value();
    this->alpha=ui->doubleSpinBox_3->value();
    this->beta=ui->doubleSpinBox_4->value();
        this->hide();
}

void GUISCG::on_GUISCG_rejected()
{
    rej=true;
        this->hide();
}

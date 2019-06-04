#include "snakes.h"
#include "ui_snakes.h"

snakes::snakes(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::snakes)
{
    ui->setupUi(this);
}

snakes::~snakes()
{
    delete ui;
}

void snakes::on_buttonBox_accepted()
{
    this->iter=ui->spinBox->value();
    this->hide();
}

void snakes::on_buttonBox_rejected()
{
    this->rej=true;
    this->hide();
}

void snakes::on_snakes_rejected()
{
    this->rej=true;
    this->hide();
}

#ifndef GUISCG_H
#define GUISCG_H

#include <QDialog>

namespace Ui {
class GUISCG;
}

class GUISCG : public QDialog
{
    Q_OBJECT

public:
    explicit GUISCG(QWidget *parent = 0);
    ~GUISCG();
public:
    Ui::GUISCG *ui;
     double lamda=2.0;
     long riter=1000;
     bool rej=false;
     long siter=1000;
     double a=0.11;
     double alpha=1.0;
     double beta=1.0;

private slots:
    void on_pushButton_clicked();

    void on_buttonBox_rejected();

    void on_buttonBox_accepted();
/*
private:
    Ui::GUISCG *ui;*/
    void on_GUISCG_rejected();
};

#endif // GUISCG_H

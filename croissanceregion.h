#ifndef CROISSANCEREGION_H
#define CROISSANCEREGION_H

#include <QDialog>

namespace Ui {
class croissanceregion;
}

class croissanceregion : public QDialog
{
    Q_OBJECT

public:
    explicit croissanceregion(QWidget *parent = 0);
    ~croissanceregion();

public:
    Ui::croissanceregion *ui;
    double lamda;
     long iter=1000;
     bool rej=false;
public slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();
private slots:
    void on_pushButton_clicked();
    void on_croissanceregion_rejected();
};

#endif // CROISSANCEREGION_H

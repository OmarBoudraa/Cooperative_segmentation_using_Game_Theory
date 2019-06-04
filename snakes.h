#ifndef SNAKES_H
#define SNAKES_H

#include <QDialog>

namespace Ui {
class snakes;
}

class snakes : public QDialog
{
    Q_OBJECT

public:
    explicit snakes(QWidget *parent = 0);
    ~snakes();

public slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

public:
    Ui::snakes *ui;
     long iter=1000;
     bool rej=false;
private slots:
    void on_snakes_rejected();
};

#endif // SNAKES_H

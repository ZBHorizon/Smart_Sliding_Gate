#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStandardItemModel>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void updateTable(int pin);
    static MainWindow* s_instance;
    
  public slots:
    void updateGateProgress(float position); // Now declared as a slot

  private:
    Ui::MainWindow *ui;
    QStandardItemModel* model;
    
    static QString digitalValToString(float value);
    static QString pwmValToString(float value);
    static QString updateMotorSpeed(float pwm, float direction);
};

#endif // MAINWINDOW_H

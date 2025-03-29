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
    void updateTable();
    
  private:
    Ui::MainWindow *ui;
    QStandardItemModel* model;
    
    static QString digitalValToString(float value);
    static QString pwmValToString(float value);
    static MainWindow* s_instance;
};

#endif // MAINWINDOW_H

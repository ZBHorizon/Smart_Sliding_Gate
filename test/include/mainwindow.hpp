#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStandardItemModel>
#include <atomic> 
#include <chrono>

using namespace std::chrono;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow: public QMainWindow {
  Q_OBJECT

public:

  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  void               updateTable(int pin);
  static MainWindow* s_instance;

public slots:
  void updateGateProgress(float position); // Now declared as a slot

private:

  Ui::MainWindow*                        ui; // changed from QPointer to raw pointer
  QStandardItemModel*                    model;
  inline static steady_clock::time_point _last_call = steady_clock::now();
  std::atomic<bool>                      _updating { false }; // Flag to prevent re-entrance
  static QString                         digitalValToString(float value);
  static QString                         pwmValToString(float value);
  static QString                         updateMotorSpeed(float pwm, float direction);
  void                                   check_for_update();
};

#endif // MAINWINDOW_H

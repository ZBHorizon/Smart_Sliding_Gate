#include <SlidingGate/Log.hpp>

#include <mainwindow.hpp>
#include <simulator.hpp>
#include <test.hpp>

#include <QStandardItemModel>
#include <QThread>
#include <QTimer>
#include <chrono>

#include "ui_mainwindow.h"
using namespace SlidingGate;
using namespace std::chrono;

MainWindow* MainWindow::s_instance = nullptr; // Definition des statischen Members

MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent)
  , ui(new Ui::MainWindow) {
  ui->setupUi(this);
  ui->light_barrier_interrupted->hide(); // initially hide the label
  s_instance = this;                     // Aktuelle Instanz speichern

  connect(ui->remote_a, &QPushButton::pressed, []() { GateSimulator::REMOTE_A_pressed(true); });
  connect(ui->remote_a, &QPushButton::released, []() { GateSimulator::REMOTE_A_pressed(false); });
  connect(ui->remote_b, &QPushButton::pressed, []() { GateSimulator::REMOTE_B_pressed(true); });
  connect(ui->remote_b, &QPushButton::released, []() { GateSimulator::REMOTE_B_pressed(false); });
  connect(ui->remote_c, &QPushButton::pressed, []() { GateSimulator::REMOTE_C_pressed(true); });
  connect(ui->remote_c, &QPushButton::released, []() { GateSimulator::REMOTE_C_pressed(false); });
  connect(ui->remote_d, &QPushButton::pressed, []() { GateSimulator::REMOTE_D_pressed(true); });
  connect(ui->remote_d, &QPushButton::released, []() { GateSimulator::REMOTE_D_pressed(false); });


  connect(ui->light_barrier, &QToolButton::toggled, [this](bool checked) {
    GateSimulator::LIGHT_BARRIER_interrupted(checked);
    ui->light_barrier_interrupted->setVisible(checked);
  });

  float pwm       = Test_IO::read_pin(Pin::PWM);
  float direction = Test_IO::read_pin(Pin::DIRECTION);


  model = new QStandardItemModel(11, 2, this);
  model->setItem(0, 0, new QStandardItem("PWM"));
  model->setItem(0, 1, new QStandardItem(pwmValToString(pwm)));
  model->setItem(1, 0, new QStandardItem("DIRECTION"));
  model->setItem(1, 1, new QStandardItem(digitalValToString(direction)));
  model->setItem(2, 0, new QStandardItem("OPEN_SWITCH"));
  model->setItem(2, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::OPEN_SWITCH))));
  model->setItem(3, 0, new QStandardItem("CLOSE_SWITCH"));
  model->setItem(3, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::CLOSE_SWITCH))));
  model->setItem(4, 0, new QStandardItem("LIGHT_BARRIER"));
  model->setItem(4, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::LIGHT_BARRIER))));
  model->setItem(5, 0, new QStandardItem("REMOTE_A"));
  model->setItem(5, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::REMOTE_A))));
  model->setItem(6, 0, new QStandardItem("REMOTE_B"));
  model->setItem(6, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::REMOTE_B))));
  model->setItem(7, 0, new QStandardItem("REMOTE_C"));
  model->setItem(7, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::REMOTE_C))));
  model->setItem(8, 0, new QStandardItem("REMOTE_D"));
  model->setItem(8, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::REMOTE_D))));
  model->setItem(9, 0, new QStandardItem("LAMP"));
  model->setItem(9, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::LAMP))));
  model->setItem(10, 0, new QStandardItem("GARDEN_DOOR"));
  model->setItem(10, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::GARDEN_DOOR))));
  ui->states->setModel(model);

  ui->states->horizontalHeader()->setVisible(false);
  ui->states->verticalHeader()->setVisible(false);

  ui->motorspeed->setText(MainWindow::updateMotorSpeed(pwm, direction));


  QTimer* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, [this]() { check_for_update(); });
  timer->start(200);
}

MainWindow::~MainWindow() {
  s_instance = nullptr;
  delete ui;
}

void MainWindow::check_for_update() {
  if (!ui || !ui->gatePosition || !ui->Gate) { return; }

  if (_last_call + milliseconds(200) < steady_clock::now()) { return; }
  updateTable(Pin::PWM);
  updateTable(Pin::DIRECTION);
  updateTable(Pin::OPEN_SWITCH);
  updateTable(Pin::CLOSE_SWITCH);
  updateTable(Pin::LIGHT_BARRIER);
  updateTable(Pin::REMOTE_A);
  updateTable(Pin::REMOTE_B);
  updateTable(Pin::REMOTE_C);
  updateTable(Pin::REMOTE_D);
  updateTable(Pin::LAMP);
  updateTable(Pin::GARDEN_DOOR);
}

void MainWindow::updateTable(int pin) {
  if (_updating.load()) { return; }
  _updating.store(true, std::memory_order_release);
  if (!ui) return;
  _last_call = steady_clock::now();
  switch (pin) {
    case Pin::PWM: { // update motorspeed when PWM changes
      float pwm       = Test_IO::read_pin(Pin::PWM);
      float direction = Test_IO::read_pin(Pin::DIRECTION);
      model->setItem(0, 1, new QStandardItem(pwmValToString(pwm)));
      ui->motorspeed->setText(MainWindow::updateMotorSpeed(pwm, direction));
    } break;
    case Pin::DIRECTION: { // update motorspeed when DIRECTION changes
      float pwm       = Test_IO::read_pin(Pin::PWM);
      float direction = Test_IO::read_pin(Pin::DIRECTION);
      model->setItem(1, 1, new QStandardItem(digitalValToString(direction)));
      ui->motorspeed->setText(MainWindow::updateMotorSpeed(pwm, direction));
    } break;
    case Pin::OPEN_SWITCH: model->setItem(2, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::OPEN_SWITCH)))); break;
    case Pin::CLOSE_SWITCH: model->setItem(3, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::CLOSE_SWITCH)))); break;
    case Pin::LIGHT_BARRIER: model->setItem(4, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::LIGHT_BARRIER)))); break;
    case Pin::REMOTE_A: model->setItem(5, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::REMOTE_A)))); break;
    case Pin::REMOTE_B: model->setItem(6, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::REMOTE_B)))); break;
    case Pin::REMOTE_C: model->setItem(7, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::REMOTE_C)))); break;
    case Pin::REMOTE_D: model->setItem(8, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::REMOTE_D)))); break;
    case Pin::LAMP: model->setItem(9, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::LAMP)))); break;
    case Pin::GARDEN_DOOR: model->setItem(10, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::GARDEN_DOOR)))); break;
    default: break;
  }
  _updating.store(false, std::memory_order_release);
}

QString MainWindow::digitalValToString(float value) {
  if (value == 0.0f) return "LOW";
  if (value == 1.0f) return "HIGH";
  return QString::number(value);
}

QString MainWindow::pwmValToString(float value) { return QString::number(value * 100.0f, 'f', 1) + "%"; }

QString MainWindow::updateMotorSpeed(float pwm, float direction) {
  float motorSpeed = pwm * 100.0f;
  if (direction == 1.0f) motorSpeed = -motorSpeed;
  return QString("Motorspeed: %1%").arg(motorSpeed, 0, 'f', 2);
}

void MainWindow::updateGateProgress(float position) {
  if (!ui) {
    qWarning("updateGateProgress: ui is null");
    return;
  }
  if (!ui->gatePosition || !ui->Gate) {
    qWarning("updateGateProgress: gatePosition or Gate is null");
    return;
  }
  if (QThread::currentThread() != this->thread()) {
    QMetaObject::invokeMethod(this, "updateGateProgress", Qt::QueuedConnection, Q_ARG(float, position));
    return;
  }

  ui->gatePosition->setText(QString("Gate Position: %1%").arg(position * 100.0f, 0, 'f', 2));
  int value = static_cast<int>(position * 100.0f);
  ui->Gate->setValue(value);
}
#include <moc_mainwindow.cpp>
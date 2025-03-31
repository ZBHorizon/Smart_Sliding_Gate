#include <simulator.hpp>
#include <test.hpp>
#include <QStandardItemModel>
#include <mainwindow.hpp>
#include "ui_mainwindow.h"

// Add the following to link the meta-object code
#include <moc_mainwindow.cpp>
#include <QTimer>
#include <QThread>
using namespace SlidingGate;

MainWindow* MainWindow::s_instance = nullptr; // Definition des statischen Members

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
      , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->light_barrier_interrupted->hide(); // initially hide the label
    s_instance = this; // Aktuelle Instanz speichern

    connect(ui->remote_a, &QPushButton::pressed, [](){ GateSimulator::REMOTE_A_pressed(true); });
    connect(ui->remote_a, &QPushButton::released, [](){ GateSimulator::REMOTE_A_pressed(false); });
    connect(ui->remote_b, &QPushButton::pressed, [](){ GateSimulator::REMOTE_B_pressed(true); });
    connect(ui->remote_b, &QPushButton::released, [](){ GateSimulator::REMOTE_B_pressed(false); });
    connect(ui->remote_c, &QPushButton::pressed, [](){ GateSimulator::REMOTE_C_pressed(true); });
    connect(ui->remote_c, &QPushButton::released, [](){ GateSimulator::REMOTE_C_pressed(false); });
    connect(ui->remote_d, &QPushButton::pressed, [](){ GateSimulator::REMOTE_D_pressed(true); });
    connect(ui->remote_d, &QPushButton::released, [](){ GateSimulator::REMOTE_D_pressed(false); });

    // Für den QToolButton light_barrier nützt sich das toggled(bool)-Signal:
    connect(ui->light_barrier, &QToolButton::toggled, [this] (bool checked) {
        GateSimulator::LIGHT_BARRIER_interrupted(checked);
        ui->light_barrier_interrupted->setVisible(checked);
    });

    float pwm = Test_IO::read_pin(Pin::PWM);
    float direction = Test_IO::read_pin(Pin::DIRECTION);

    // Neues Modell für die Tabelle "states" erstellen (11 Zeilen, 2 Spalten)
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
    // Header (Zeilen- und Spaltenüberschriften) ausblenden
    ui->states->horizontalHeader()->setVisible(false);
    ui->states->verticalHeader()->setVisible(false);

    ui->motorspeed->setText(MainWindow::updateMotorSpeed(pwm, direction));

    // // Timer einrichten, der alle 50ms updateTable() aufruft:
    // QTimer* timer = new QTimer(this);
    // connect(timer, &QTimer::timeout, this, &MainWindow::updateTable);
    // timer->start(100);
}

MainWindow::~MainWindow()
{
    s_instance = nullptr; // Instanz zurücksetzen
    delete ui;
}

void MainWindow::updateTable(int pin) {
    switch(pin) {
        case Pin::PWM:
        { // update motorspeed when PWM changes
            float pwm = Test_IO::read_pin(Pin::PWM);
            float direction = Test_IO::read_pin(Pin::DIRECTION);
            model->setItem(0, 1, new QStandardItem(pwmValToString(pwm)));
            ui->motorspeed->setText(MainWindow::updateMotorSpeed(pwm, direction));
            }
            break;
        case Pin::DIRECTION:
            { // update motorspeed when DIRECTION changes
            float pwm = Test_IO::read_pin(Pin::PWM);
            float direction = Test_IO::read_pin(Pin::DIRECTION);
            model->setItem(1, 1, new QStandardItem(digitalValToString(direction)));
            ui->motorspeed->setText(MainWindow::updateMotorSpeed(pwm, direction));
            }
            break;
        case Pin::OPEN_SWITCH:
            model->setItem(2, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::OPEN_SWITCH))));
            break;
        case Pin::CLOSE_SWITCH:
            model->setItem(3, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::CLOSE_SWITCH))));
            break;
        case Pin::LIGHT_BARRIER:
            model->setItem(4, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::LIGHT_BARRIER))));
            break;
        case Pin::REMOTE_A:
            model->setItem(5, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::REMOTE_A))));
            break;
        case Pin::REMOTE_B:
            model->setItem(6, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::REMOTE_B))));
            break;
        case Pin::REMOTE_C:
            model->setItem(7, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::REMOTE_C))));
            break;
        case Pin::REMOTE_D:
            model->setItem(8, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::REMOTE_D))));
            break;
        case Pin::LAMP:
            model->setItem(9, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::LAMP))));
            break;
        case Pin::GARDEN_DOOR:
            model->setItem(10, 1, new QStandardItem(digitalValToString(Test_IO::read_pin(Pin::GARDEN_DOOR))));
            break;
        default:
            break;
    }
}

QString MainWindow::digitalValToString(float value) {
    if (value == 0.0f) return "LOW";
    if (value == 1.0f) return "HIGH";
    return QString::number(value);
}

QString MainWindow::pwmValToString(float value) {
    return QString::number(value * 100, 'f', 1) + "%";
}

QString MainWindow::updateMotorSpeed(float pwm, float direction) {
    float motorSpeed = pwm * 100.0f;
    if (direction == 0.0f)
        motorSpeed = -motorSpeed;
    return QString("Motorspeed: %1%").arg(motorSpeed, 0, 'f', 1);
}

void MainWindow::updateGateProgress(float position) {
    // If we're not in the same thread as the UI, schedule ourselves to run on the UI thread
    if(QThread::currentThread() != this->thread()){
        QMetaObject::invokeMethod(this, "updateGateProgress", Qt::QueuedConnection, Q_ARG(float, position));
        return;
    }
    int value = static_cast<int>(position * 100);
    ui->Gate->setValue(value);
}
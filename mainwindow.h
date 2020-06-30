/*
    mainwindow.h (part of SimFerryController)
    Copyright (C) 2020 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QUdpSocket>
#include <QTimer>
#include "Eigen/Geometry"
#include "losolver.h"
#include "autopilot.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_Bind_clicked();
    void readyRead();

    void on_pushButton_Close_clicked();
    void on_cyclicTimer_timeout();

    void on_pushButton_Destination_Set_clicked();

    void on_pushButton_Destination_Randomize_clicked();

    void on_pushButton_UseReferenceCoordinates_clicked();

private:
    void addLogLine(const QString& line);

    LOSolver loSolver;
    Eigen::Vector3d oldRefPoints[3];

    Autopilot::Settings autopilotSettings;
    Autopilot autopilot;
    Autopilot::Destination autopilotDestination;

    Ui::MainWindow *ui;
    QUdpSocket* udpClientSocket = nullptr;
    QUdpSocket* udpServerSocket = nullptr;
    void printMatrix3d(Eigen::Matrix3d& matrix);
    void printTransform(Eigen::Transform<double, 3, Eigen::Affine>& matrix);

    void sendAutopilotOutputs(const Autopilot::Outputs& autopilotOutputs);

    QTimer* cyclicTimer = nullptr;
    unsigned int timeAfterSendingAutopilotCommand = 10000;

    int nearCounter;
};
#endif // MAINWINDOW_H

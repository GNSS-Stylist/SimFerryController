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

#include <QTime>
#include <QNetworkDatagram>
#include <QRandomGenerator>
#include <QMessageBox>
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    udpClientSocket = new QUdpSocket(this);
    udpServerSocket = new QUdpSocket(this);

    cyclicTimer = new QTimer(this);
    connect(cyclicTimer, SIGNAL(timeout()), this, SLOT(on_cyclicTimer_timeout()));
    cyclicTimer->start(125);

#if 0
    autopilotSettings.maxSpeed = 10;                    // m / s
    autopilotSettings.maxAngularSpeed = M_PI/5;             // Radians / s
    autopilotSettings.maxPropulsion = 50000;               // arbitrary units
    autopilotSettings.distanceFromCenter_Front = 9;    // m, typically positive value
    autopilotSettings.distanceFromCenter_Back = -9;     // m, typically negative value
    autopilotSettings.estimatedAcceleration = 1 / 10000;       // m / (s * s) / unit of propulsion
    autopilotSettings.estimatedAngularAcceleration = 1 / 10000;// Radians / (s * s) / unit of propulsion
#endif
    autopilotSettings.nearLimit = 20;                   // Distance from target where to change from "travel" to "orientation"-mode
    autopilotSettings.cruisePropulsion = 50000;
    autopilotSettings.cruiseDirectionProp = 0.2;

    autopilotSettings.pidSettings_Position.p = 20000;
    autopilotSettings.pidSettings_Position.i = 200;
    autopilotSettings.pidSettings_Position.d = 200000;
    autopilotSettings.pidSettings_Position.f = 0;
    autopilotSettings.pidSettings_Position.maxI = 50000;
    autopilotSettings.pidSettings_Position.maxOut = 20000;
    autopilotSettings.pidSettings_Position.rememberI = true;

    autopilotSettings.pidSettings_Heading.p = 80000;
    autopilotSettings.pidSettings_Heading.i = 1000;
    autopilotSettings.pidSettings_Heading.d = 500000;
    autopilotSettings.pidSettings_Heading.f = 0;
    autopilotSettings.pidSettings_Heading.maxI = 50000;
    autopilotSettings.pidSettings_Heading.maxOut = 20000;
    autopilotSettings.pidSettings_Heading.rememberI = false;

    autopilot.init(autopilotSettings);

    autopilotDestination.coord_N = 0;
    autopilotDestination.coord_E = 0;
    autopilotDestination.heading = 0;

    autopilot.setDestination(autopilotDestination);

    nearCounter = 0xFFFFF;

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_Bind_clicked()
{
    addLogLine("Binding...");

    if (!udpClientSocket->bind(QHostAddress(ui->lineEdit_Host->text()), ui->spinBox_Port_Bind->value()))
    {
        addLogLine("Binding failed.");
    }
    else
    {
        QObject::connect(udpClientSocket, SIGNAL(readyRead()),
                     this, SLOT(readyRead()));

        addLogLine("Ok.");
    }

    ui->pushButton_Bind->setEnabled(false);
    ui->pushButton_Close->setEnabled(true);
}

void MainWindow::addLogLine(const QString& line)
{
    //ui->listWidget->selectionBehavior()

    QTime currentTime = QTime::currentTime();

    QString timeString = currentTime.toString("hh:mm:ss:zzz");

    ui->plainTextEdit->setMaximumBlockCount(1000);
//    ui->plainTextEdit->setCenterOnScroll(ui->checkBox_PagedScroll->isChecked());
//    ui->plainTextEdit->setWordWrapMode(QTextOption::NoWrap);
    ui->plainTextEdit->appendPlainText(timeString + ": " + line);
}

void MainWindow::printMatrix3d(Eigen::Matrix3d& matrix)
{
    for (int row = 0; row < 3; row++)
    {
        QString rowPrint;
        for (int column = 0; column < 3; column++)
        {
            if (column != 0)
            {
                rowPrint += "\t";
            }
            rowPrint += QString::number(matrix(row, column), 'f', 3);
        }
        ui->plainTextEdit->appendPlainText(rowPrint);
    }
}

void MainWindow::printTransform(Eigen::Transform<double, 3, Eigen::Affine>& matrix)
{
    for (int row = 0; row < 4; row++)
    {
        QString rowPrint;
        for (int column = 0; column < 4; column++)
        {
            if (column != 0)
            {
                rowPrint += "\t";
            }
            rowPrint += QString::number(matrix(row, column), 'f', 3);
        }
        ui->plainTextEdit->appendPlainText(rowPrint);
    }
}

/*
static double refPointACoords[3] = { 1, 2, 3 };
static double refPointBCoords[3] = { 4, 5, 6 };
static double refPointCCoords[3] = { 9, 8, 7 };


static double pointACoords[3] = { 0, 0, 0 };
static double pointBCoords[3] = { 1, 0, 0 };
static double pointCCoords[3] = { 0, 1, 0 };
*/

void MainWindow::readyRead()
{
    // Add "separator line"
    ui->plainTextEdit->appendPlainText("");

    addLogLine("readyRead signal received.");

    unsigned short sendPort = ui->spinBox_Port_Send->value();

    while (udpClientSocket->hasPendingDatagrams())
    {
        QNetworkDatagram datagram = udpClientSocket->receiveDatagram();

        addLogLine("New datagram, Size: " + QString::number(datagram.data().size()) +
                   ", senderAddress: " + datagram.senderAddress().toString() +
                   ", senderPort: " + QString::number(datagram.senderPort()) + ": "+
                   datagram.data());

        QString dataString = datagram.data();

        QStringList subStrings = dataString.split(';');

        if (subStrings.size() < (2 * 3 * 3))
        {
            addLogLine("Not enough items!");
            continue;
        }

        double subValues[2 * 3 * 3];

        for (int i = 0; i < (2 * 3 * 3); i++)
        {
            subValues[i] = subStrings.at(i).toDouble();
        }

        Eigen::Vector3d refPointA(&subValues[3 * 3]);
        Eigen::Vector3d refPointB(&subValues[4 * 3]);
        Eigen::Vector3d refPointC(&subValues[5 * 3]);

        if (((refPointA != oldRefPoints[0]) ||
                (refPointB != oldRefPoints[1]) ||
                (refPointC != oldRefPoints[2])) &&
                (ui->checkBox_AutoUpdateReferenceCoordinates->isChecked()))
        {
            addLogLine("Got new reference points.");

            addLogLine("refPointA:\t" + QString::number(refPointA(0), 'f', 3) + ",\t" + QString::number(refPointA(1), 'f', 3)+ ",\t" + QString::number(refPointA(2), 'f', 3));
            addLogLine("refPointB:\t" + QString::number(refPointB(0), 'f', 3) + ",\t" + QString::number(refPointB(1), 'f', 3)+ ",\t" + QString::number(refPointB(2), 'f', 3));
            addLogLine("refPointC:\t" + QString::number(refPointC(0), 'f', 3) + ",\t" + QString::number(refPointC(1), 'f', 3)+ ",\t" + QString::number(refPointC(2), 'f', 3));

            oldRefPoints[0] = refPointA;
            oldRefPoints[1] = refPointB;
            oldRefPoints[2] = refPointC;

            ui->tableWidget_ReferenceCoordinates->item(0, 0)->setText(QString::number(refPointA(0), 'f', 3));
            ui->tableWidget_ReferenceCoordinates->item(0, 1)->setText(QString::number(refPointA(1), 'f', 3));
            ui->tableWidget_ReferenceCoordinates->item(0, 2)->setText(QString::number(refPointA(2), 'f', 3));

            ui->tableWidget_ReferenceCoordinates->item(1, 0)->setText(QString::number(refPointB(0), 'f', 3));
            ui->tableWidget_ReferenceCoordinates->item(1, 1)->setText(QString::number(refPointB(1), 'f', 3));
            ui->tableWidget_ReferenceCoordinates->item(1, 2)->setText(QString::number(refPointB(2), 'f', 3));

            ui->tableWidget_ReferenceCoordinates->item(2, 0)->setText(QString::number(refPointC(0), 'f', 3));
            ui->tableWidget_ReferenceCoordinates->item(2, 1)->setText(QString::number(refPointC(1), 'f', 3));
            ui->tableWidget_ReferenceCoordinates->item(2, 2)->setText(QString::number(refPointC(2), 'f', 3));

            if (!loSolver.setReferencePoints(refPointA, refPointB, refPointC))
            {
                addLogLine("Setting reference points failed, error code: " + QString::number(loSolver.getLastError()));
            }
        }

        Eigen::Vector3d pointA(&subValues[0 * 3]);
        Eigen::Vector3d pointB(&subValues[1 * 3]);
        Eigen::Vector3d pointC(&subValues[2 * 3]);

        Eigen::Transform<double, 3, Eigen::Affine> transform_EUS;

        loSolver.setPoints(pointA, pointB, pointC);

        Eigen::Transform<double, 3, Eigen::Affine> debugTransform;

        if (!loSolver.getTransformMatrix(transform_EUS, &debugTransform))
        {
            addLogLine("Getting transform matrix failed, error code: " + QString::number(loSolver.getLastError()));
        }
        else
        {
            double heading, pitch, roll;

            Eigen::Transform<double, 3, Eigen::Affine> transform_NED = LOSolver::changeAxesConvention(transform_EUS, LOSolver::AC_EUS, LOSolver::AC_NED);

            loSolver.getYawPitchRollAngles(transform_EUS, heading, pitch, roll, LOSolver::AC_EUS);

            heading *= 360. / (M_PI * 2);
            pitch *= 360. / (M_PI * 2);
            roll *= 360. / (M_PI * 2);

            heading = fmod((heading + 360), 360);

            addLogLine("Destination\tN: " + QString::number(autopilotDestination.coord_N,'f',3) +
                       "\tE: " + QString::number(autopilotDestination.coord_E,'f',3) +
                       "\tHeading: " + QString::number(fmod(autopilotDestination.heading * 360. / (M_PI * 2) + 360, 360),'f',2));

            addLogLine("Location\tN: " + QString::number(transform_NED(0,3),'f',3) +
                       "\tE: " + QString::number(transform_NED(1,3),'f',3) +
                       "\tD: " + QString::number(transform_NED(2,3),'f',2));

            addLogLine("Heading: " + QString::number(heading,'f',2) +
                       "\tPitch: " + QString::number(pitch,'f',2) +
                       "\tRoll: " + QString::number(roll,'f',2));

            QString outString =
                    // Note: Linear (basis) part of the transformation matrix here is transposed.
                    // Not fixing this now as it would break the compatibility with the simulator.
                    "1;" +   // "Command id": transform
                    QString::number(transform_EUS(0,0),'f',3) + ";" + QString::number(transform_EUS(1,0),'f',3) + ";" + QString::number(transform_EUS(2,0),'f',3) + ";" + QString::number(transform_EUS(0,3),'f',3) + ";" +
                    QString::number(transform_EUS(0,1),'f',3) + ";" + QString::number(transform_EUS(1,1),'f',3) + ";" + QString::number(transform_EUS(2,1),'f',3) + ";" + QString::number(transform_EUS(1,3),'f',3) + ";" +
                    QString::number(transform_EUS(0,2),'f',3) + ";" + QString::number(transform_EUS(1,2),'f',3) + ";" + QString::number(transform_EUS(2,2),'f',3) + ";" + QString::number(transform_EUS(2,3),'f',3) + ";" +
                    QString::number(transform_EUS(0,3),'f',3) + ";" + QString::number(transform_EUS(1,3),'f',3) + ";" + QString::number(transform_EUS(2,3),'f',3) + ";" + QString::number(transform_EUS(3,3),'f',3);

            QByteArray dataToSend = outString.toLatin1();
            udpServerSocket->writeDatagram(dataToSend, QHostAddress(ui->lineEdit_Host->text()), sendPort);

            outString =
                    // Note: Linear (basis) part of the transformation matrix here is transposed.
                    // Not fixing this now as it would break the compatibility with the simulator.
                    "10;" +   // "Command id": debugdata
                    QString::number(debugTransform(0,0),'f',3) + ";" + QString::number(debugTransform(1,0),'f',3) + ";" + QString::number(debugTransform(2,0),'f',3) + ";" + QString::number(debugTransform(0,3),'f',3) + ";" +
                    QString::number(debugTransform(0,1),'f',3) + ";" + QString::number(debugTransform(1,1),'f',3) + ";" + QString::number(debugTransform(2,1),'f',3) + ";" + QString::number(debugTransform(1,3),'f',3) + ";" +
                    QString::number(debugTransform(0,2),'f',3) + ";" + QString::number(debugTransform(1,2),'f',3) + ";" + QString::number(debugTransform(2,2),'f',3) + ";" + QString::number(debugTransform(2,3),'f',3) + ";" +
                    QString::number(debugTransform(0,3),'f',3) + ";" + QString::number(debugTransform(1,3),'f',3) + ";" + QString::number(debugTransform(2,3),'f',3) + ";" + QString::number(debugTransform(3,3),'f',3);

            dataToSend = outString.toLatin1();
            udpServerSocket->writeDatagram(dataToSend, QHostAddress(ui->lineEdit_Host->text()), sendPort);

            if (ui->checkBox_AutopilotActive->checkState())
            {
                Autopilot::Outputs autopilotOutputs;
                Autopilot::DebugOutputs autopilotDebugOutputs;

                autopilot.update(transform_NED, autopilotOutputs, 0.125, &autopilotDebugOutputs);

                addLogLine("AP: direction_Front: " + QString::number(autopilotOutputs.direction_Front * 360. / (M_PI * 2),'f', 2) +
                           "\tpower_Front: " + QString::number(autopilotOutputs.propulsion_Front,'f', 1) +
                           "\tdirection_Back: " + QString::number(autopilotOutputs.direction_Back * 360. / (M_PI * 2),'f', 2) +
                           "\tpower_Back: " + QString::number(autopilotOutputs.propulsion_Back,'f', 1));

                addLogLine("AP debug:\tabsBearing: " + QString::number(fmod(autopilotDebugOutputs.absBearing * 360. / (M_PI * 2) + 360, 360),'f', 2) +
                           "\trelativeBearing: " + QString::number(autopilotDebugOutputs.relativeBearing * 360. / (M_PI * 2),'f', 2) +
                           "\tdistanceToTarget: " + QString::number(autopilotDebugOutputs.distanceToTarget,'f', 3) +
                           "\tspeed: " + QString::number(autopilotDebugOutputs.speed,'f', 2));

                addLogLine("AP debug2:\tdirectionOfTravel: " + QString::number(fmod(autopilotDebugOutputs.directionOfTravel * 360. / (M_PI * 2) + 360, 360),'f', 2) +
                           "\theadingError: " + QString::number(autopilotDebugOutputs.headingError * 360. / (M_PI * 2),'f', 2) +
                           "\tstate: " + QString::number((int)autopilotDebugOutputs.state));

                sendAutopilotOutputs(autopilotOutputs);

                if (ui->checkBox_DestinationRandomizer_Auto_Active->checkState() &&
                        (((autopilotDebugOutputs.distanceToTarget <= ui->doubleSpinBox_DestinationRandomizer_Auto_DistanceLimit->value()) &&
                        ((fabs(autopilotDebugOutputs.headingError * 360 / (2* M_PI)) <= ui->doubleSpinBox_DestinationRandomizer_Auto_HeadingLimit->value())))  ||
                         (nearCounter > 0xFFFF /* First round */)))
                {
                    nearCounter++;

                    addLogLine("Waiting new waypoint randomizing, " + QString::number(nearCounter) + "/" + QString::number(ui->spinBox_DestinationRandomizer_Auto_TimeLimit->value()));

                    if (nearCounter >= ui->spinBox_DestinationRandomizer_Auto_TimeLimit->value())
                    {
                        on_pushButton_Destination_Randomize_clicked();
                        nearCounter = 0;
                    }
                }
                else
                {
                    if (ui->checkBox_DestinationRandomizer_Auto_Active->checkState())
                    {
                        // This is just to prevent lines from hopping up and down according to proximity
                        addLogLine("Destination not near enough for new waypoint randomizing");
                    }
                    nearCounter = 0;
                }
            }

            ui->progressBar_Heading->setValue(heading * 100);
            ui->label_Heading_Value->setText(QString::number(heading,'f', 2));
            ui->progressBar_Pitch->setValue(pitch * 100);
            ui->label_Pitch_Value->setText(QString::number(pitch,'f', 2));
            ui->progressBar_Roll->setValue(roll * 100);
            ui->label_Roll_Value->setText(QString::number(roll,'f', 2));
        }
    }
}

void MainWindow::on_pushButton_Close_clicked()
{
    udpClientSocket->close();

    QObject::disconnect(udpClientSocket, SIGNAL(readyRead()),
                 this, SLOT(readyRead()));

    ui->pushButton_Bind->setEnabled(true);
    ui->pushButton_Close->setEnabled(false);

    addLogLine("UDP client: socket closed.");
}

void MainWindow::sendAutopilotOutputs(const Autopilot::Outputs& autopilotOutputs)
{
    QString outString =
            "2;" +   // "Command id": Propulsion
            QString::number(autopilotOutputs.direction_Front,'f',3) + ";" + QString::number(autopilotOutputs.propulsion_Front,'f',3) + ";" +
            QString::number(autopilotOutputs.direction_Back,'f',3) + ";" + QString::number(autopilotOutputs.propulsion_Back,'f',3);

    QByteArray dataToSend = outString.toLatin1();

//    dataToSend.append(outString);
    udpServerSocket->writeDatagram(dataToSend, QHostAddress(ui->lineEdit_Host->text()), ui->spinBox_Port_Send->value());

    timeAfterSendingAutopilotCommand = 0;
}


void MainWindow::on_cyclicTimer_timeout()
{
    timeAfterSendingAutopilotCommand += cyclicTimer->interval();

    if ((timeAfterSendingAutopilotCommand > (125 * 2.5)) &&
            (ui->checkBox_AutopilotActive->checkState()) &&
            udpClientSocket->isOpen())
    {
        Autopilot::Outputs autopilotOutputs;

        autopilotOutputs.propulsion_Front = 0;
        autopilotOutputs.direction_Front = 0;
        autopilotOutputs.propulsion_Back = 0;
        autopilotOutputs.direction_Back = 0;

        sendAutopilotOutputs(autopilotOutputs);
    }
}


void MainWindow::on_pushButton_Destination_Set_clicked()
{
    autopilotDestination.coord_N = ui->doubleSpinBox_Destination_N->value();
    autopilotDestination.coord_E = ui->doubleSpinBox_Destination_E->value();
    autopilotDestination.heading = ui->doubleSpinBox_Destination_Heading->value() * 2. * M_PI / 360.;

    autopilot.setDestination(autopilotDestination);

    QString outString =
            "3;" +   // "Command id": Set destination
            QString::number(autopilotDestination.coord_E,'f',3) + ";" + QString::number(-autopilotDestination.coord_N,'f',3) + ";" +
            QString::number(autopilotDestination.heading,'f',3);

    QByteArray dataToSend = outString.toLatin1();
    udpServerSocket->writeDatagram(dataToSend, QHostAddress(ui->lineEdit_Host->text()), ui->spinBox_Port_Send->value());

    nearCounter = 0;
}

void MainWindow::on_pushButton_Destination_Randomize_clicked()
{
    ui->doubleSpinBox_Destination_N->setValue(ui->doubleSpinBox_DestinationRandomizer_N_Min->value() + QRandomGenerator::global()->generateDouble() * (ui->doubleSpinBox_DestinationRandomizer_N_Max->value() - ui->doubleSpinBox_DestinationRandomizer_N_Min->value()));
    ui->doubleSpinBox_Destination_E->setValue(ui->doubleSpinBox_DestinationRandomizer_E_Min->value() + QRandomGenerator::global()->generateDouble() * (ui->doubleSpinBox_DestinationRandomizer_E_Max->value() - ui->doubleSpinBox_DestinationRandomizer_E_Min->value()));
    ui->doubleSpinBox_Destination_Heading->setValue(ui->doubleSpinBox_DestinationRandomizer_Heading_Min->value() + QRandomGenerator::global()->generateDouble() * (ui->doubleSpinBox_DestinationRandomizer_Heading_Max->value() - ui->doubleSpinBox_DestinationRandomizer_Heading_Min->value()));

    on_pushButton_Destination_Set_clicked();
}

void MainWindow::on_pushButton_UseReferenceCoordinates_clicked()
{
    double refPointValues[3][3];

    for (int antennaIndex = 0; antennaIndex < 3; antennaIndex++)
    {
        for (int coordIndex= 0; coordIndex < 3; coordIndex++)
        {
            bool ok;

            refPointValues[antennaIndex][coordIndex] = ui->tableWidget_ReferenceCoordinates->item(antennaIndex, coordIndex)->text().toDouble(&ok);

            if (!ok)
            {
                QString detailedText = "Row " + QString::number(antennaIndex + 1) +
                        ", column " + QString::number(coordIndex + 1) +
                        " not convertible to a (double precision) floating point value. ";

                addLogLine("Error: " + detailedText);

                QMessageBox msgBox;
                msgBox.setText("Error.");
                msgBox.setInformativeText(detailedText);

                QPushButton *okButton = msgBox.addButton(QMessageBox::Ok);

                msgBox.setDefaultButton(okButton);

                msgBox.exec();
                return;
            }
        }
    }

    Eigen::Vector3d refPointA(&refPointValues[0][0]);
    Eigen::Vector3d refPointB(&refPointValues[1][0]);
    Eigen::Vector3d refPointC(&refPointValues[2][0]);

    addLogLine("New reference points.");

    addLogLine("refPointA:\t" + QString::number(refPointA(0), 'f', 3) + ",\t" + QString::number(refPointA(1), 'f', 3)+ ",\t" + QString::number(refPointA(2), 'f', 3));
    addLogLine("refPointB:\t" + QString::number(refPointB(0), 'f', 3) + ",\t" + QString::number(refPointB(1), 'f', 3)+ ",\t" + QString::number(refPointB(2), 'f', 3));
    addLogLine("refPointC:\t" + QString::number(refPointC(0), 'f', 3) + ",\t" + QString::number(refPointC(1), 'f', 3)+ ",\t" + QString::number(refPointC(2), 'f', 3));

    oldRefPoints[0] = refPointA;
    oldRefPoints[1] = refPointB;
    oldRefPoints[2] = refPointC;

    if (!loSolver.setReferencePoints(refPointA, refPointB, refPointC))
    {
        QString errorText = "Setting reference points failed, error code: " + QString::number(loSolver.getLastError());

        addLogLine(errorText);

        QMessageBox msgBox;
        msgBox.setText("Error.");
        msgBox.setInformativeText(errorText);

        QPushButton *okButton = msgBox.addButton(QMessageBox::Ok);

        msgBox.setDefaultButton(okButton);

        msgBox.exec();
    }
}

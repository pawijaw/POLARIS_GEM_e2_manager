#include "UserInterface.h"
#include "./ui_UserInterface.h"
#include "RobotManagerDefs.h"

#include <QFileDialog>


namespace RobotManager {

UserInterface::UserInterface(QWidget *parent) :
    QMainWindow(parent),

    ui(new Ui::UserInterface),
    m_timer(this),
    m_nodeHandle(),
    m_robotManager(5)
{
    ui->setupUi(this);

    connect(&m_timer, &QTimer::timeout, this, &UserInterface::onTimer);

    m_timer.setInterval(10);
    m_timer.start();

    m_seriesGpsAccuracy.setName("GPS Accuracy[cm]");
    m_seriesBattery.setName("Battery [%]");
    m_seriesTemperature.setName("Temperature [C]");
    m_seriesV2xSignal.setName("V2x Signal");
    m_seriesEstop.setName("E-Stop");

    QChart *chart = new QChart(); // ownership later passed to QChartView - no need to deallocate
    chart->addSeries(&m_seriesGpsAccuracy);
    chart->addSeries(&m_seriesBattery);
    chart->addSeries(&m_seriesTemperature);
    chart->addSeries(&m_seriesV2xSignal);
    chart->addSeries(&m_seriesEstop);

    chart->createDefaultAxes();
    chart->axes(Qt::Vertical).back()->setRange(0.0, 100.0);
    chart->axes(Qt::Horizontal).back()->setRange(0.0, 25.0);
  //  chart->axes(Qt::Horizontal).back()->setTitleText("ROS Time");

    chart->layout()->setContentsMargins(0, 0, 0, 0);
    chart->setBackgroundRoundness(0);

    ui->plotChart->setChart(chart);
    ui->plotChart->setRenderHint(QPainter::Antialiasing);
}

void UserInterface::onTimer()
{
    const auto vehData = m_robotManager.getVehicleData();

//    if(vehData.state != RobotState::RUNNING && ui->pushButtonLoadWps->isChecked())
//    {
//        ui->pushButtonLoadWps->setText("Load Waypoints");
//        ui->pushButtonLoadWps->setChecked(false);
//        ui->radioButtonCtrlPure->setEnabled(true);
//        ui->radioButtonCtrlStanley->setEnabled(true);
//    }

    //update vehicle data window
    std::stringstream dataStr;
    dataStr << "x= " << vehData.gpsData.gpsMsg.x << std::endl;
    dataStr << "y= " << vehData.gpsData.gpsMsg.y << std::endl;

    ui->labelVehData->setText(QString::fromStdString(dataStr.str()));

    //update vehicle state window
    QColor stateColour(Qt::red);
    QString robotState("ERROR");
    switch(vehData.state)
    {
    case RobotState::ERROR:
        robotState = "ERROR";
        stateColour = Qt::red;
        break;
    case RobotState::IDLE:
        robotState = "IDLE";
        stateColour = Qt::gray;
        break;
    case RobotState::RUNNING:
        robotState = "RUNNING";
        stateColour = Qt::green;
        break;
    }
    QPalette palette = ui->lineEditState->palette();
    palette.setColor(ui->lineEditState->backgroundRole(), stateColour);
    ui->lineEditState->setText(robotState);
    ui->lineEditState->setPalette(palette);

    //update sensor readings
    ui->labelBatteryValue->setText(QString("Battery level: ") + QString::number(vehData.batteryData.batteryMsg.stateOfCharge)+ QString(" %"));
    ui->labelTemperatureValue->setText(QString("Temperature: ") + QString::number(vehData.temperatureData.temperatureMsg.temperature) + QString(" C"));
    ui->labelGnssValue->setText(QString("Accuracy: ") + QString::number(vehData.gpsData.gpsMsg.horizontalAccuracy) + QString(" mm"));
    ui->labelEstopValue->setText(QString("E-Stop: ") + (vehData.estopData.estopMsg.estopTriggered ? "ACTIVE" : "OFF"));

    QString signalString;
    switch(vehData.v2xData.v2xMsg.signalStrength)
    {
    case 2:
        signalString = "Good";
        break;
    case 1:
        signalString = "Weak";
        break;
    case 0:
    default:
        signalString = "Loss";
        break;
    }

    ui->labelSignalValue->setText(QString("Signal: ") + signalString);

    //update Log
    for(const auto& logMsg : vehData.logs)
    {
        appendToLog(QString::fromStdString(logMsg.second), logMsg.first);
    }

    addSampleToChart(vehData);

}

UserInterface::~UserInterface()
{
    delete ui;
}


void UserInterface::on_pushButtonGpsLoadTest_clicked(bool checked)
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Data File"), tr("GPS Data"));

    if(fileName.size() > 0)
    {
        auto result = m_mockControl.sendGpsDemand(fileName.toStdString());
        appendToLog(QString::fromStdString(result));
    }
}

void UserInterface::on_pushButtonBatteryTest_clicked(bool checked)
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Data File"), tr("Battery Data"));

    if(fileName.size() > 0)
    {
        auto result = m_mockControl.sendBatteryDemand(fileName.toStdString());
        appendToLog(QString::fromStdString(result));
    }
}

void UserInterface::appendToLog(const QString& str, double time)
{
    static const auto startTime = ros::Time::now().toSec();
    const auto currentTime = time >= 0.0 ? (time) : (ros::Time::now().toSec());

    auto text = "t=" + QString::number(currentTime, 'f', 5) + ": " + str + "\n" + ui->textEditLog->toPlainText();
    //ROS_INFO("%s", text.toStdString().c_str());
    ui->textEditLog->setText(text);
}

void UserInterface::on_pushButtonLoadWps_clicked(bool checked)
{
    if(checked)
    {
        //attempt to load waypoints and activate navigation

        const auto fileName = QFileDialog::getOpenFileName(this, tr("Open Data File"), tr("Waypoint Data"));

        if(fileName.size() == 0)
        {
            return;
        }

        auto navStatus = m_robotManager.setNavigation(getSelectedNavService(), fileName.toStdString());
        if(navStatus.first)
        {
            //navigation activated sucessfuly
            ui->pushButtonLoadWps->setText("Stop Navigation");
            ui->radioButtonCtrlPure->setEnabled(false);
            ui->radioButtonCtrlStanley->setEnabled(false);
            appendToLog(QString("NAV COMMAND OK: ") + QString::fromStdString(navStatus.second));
        }
        else
        {
            //failed to activate navigation
            ui->pushButtonLoadWps->setChecked(false);
            appendToLog(QString("NAV COMMAND FAILED: ") + QString::fromStdString(navStatus.second));
        }
    }
    else
    {
        auto navStatus = m_robotManager.setNavigation(getSelectedNavService(), "");
        if(navStatus.first)
        {
            //navigation deactivated sucessfuly
            ui->pushButtonLoadWps->setText("Load Waypoints");
            ui->radioButtonCtrlPure->setEnabled(true);
            ui->radioButtonCtrlStanley->setEnabled(true);
            appendToLog(QString("NAV COMMAND OK: ") + QString::fromStdString(navStatus.second));
        }
        else
        {
            //failed to deactivate navigation
            ui->pushButtonLoadWps->setChecked(true);
            appendToLog(QString("NAV COMMAND FAILED: ") + QString::fromStdString(navStatus.second));
        }
    }
}

std::string UserInterface::getSelectedNavService() const
{
    //mutual exclusivity enforced by click handlers

    if(ui->radioButtonCtrlPure->isChecked())
    {
        return SERVICE_NAV_PURE_PURSUIT;
    }
    else if(ui->radioButtonCtrlStanley->isChecked())
    {
        return SERVICE_NAV_STANLEY;
    }
    return "";
}


void UserInterface::on_radioButtonCtrlPure_clicked(bool checked)
{
    ui->radioButtonCtrlStanley->setChecked(false);
}


void UserInterface::on_radioButtonCtrlStanley_clicked(bool checked)
{
    ui->radioButtonCtrlPure->setChecked(false);
}

void UserInterface::on_pushButtonTempTest_clicked(bool checked)
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Data File"), tr("Temperature Data"));

    if(fileName.size() > 0)
    {
        auto result = m_mockControl.sendTemperatureDemand(fileName.toStdString());
        appendToLog(QString::fromStdString(result));
    }
}

void UserInterface::on_pushButtonV2xTest_clicked(bool checked)
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Data File"), tr("V2x Data"));

    if(fileName.size() > 0)
    {
        auto result = m_mockControl.sendV2xDemand(fileName.toStdString());
        appendToLog(QString::fromStdString(result));
    }
}

void UserInterface::on_pushButtonEstopTest_clicked(bool checked)
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Data File"), tr("E-Stop Data"));

    if(fileName.size() > 0)
    {
        auto result = m_mockControl.sendEstopDemand(fileName.toStdString());
        appendToLog(QString::fromStdString(result));
    }
}

void UserInterface::setGuiNavActive(bool navActive)
{
    ui->pushButtonLoadWps->setText(navActive ? "Stop Navigation" : "Load Waypoints");
    ui->pushButtonLoadWps->setChecked(navActive);
    ui->radioButtonCtrlPure->setEnabled(!navActive);
    ui->radioButtonCtrlStanley->setEnabled(!navActive);
}

void UserInterface::addSampleToChart(const VehicleData& vehData)
{
    const auto currentTime = ros::Time::now().toSec();

    ui->plotChart->chart()->axes(Qt::Horizontal).back()->setRange(currentTime-30.0, currentTime);

    constexpr auto maxSamples = 10000;

    if(m_seriesGpsAccuracy.count() > maxSamples)
    {
        m_seriesGpsAccuracy.remove(0);
    }
    m_seriesGpsAccuracy.append(vehData.gpsData.receiveTimestamp, vehData.gpsData.gpsMsg.horizontalAccuracy/10.0);

    if(m_seriesBattery.count() > maxSamples)
    {
        m_seriesBattery.remove(0);
    }
    m_seriesBattery.append(vehData.batteryData.receiveTimestamp, vehData.batteryData.batteryMsg.stateOfCharge);

    if(m_seriesTemperature.count() > maxSamples)
    {
        m_seriesTemperature.remove(0);
    }
    m_seriesTemperature.append(vehData.temperatureData.receiveTimestamp, vehData.temperatureData.temperatureMsg.temperature);

    if(m_seriesV2xSignal.count() > maxSamples)
    {
        m_seriesV2xSignal.remove(0);
    }
    m_seriesV2xSignal.append(vehData.v2xData.receiveTimestamp, vehData.v2xData.v2xMsg.signalStrength*50.0);

    if(m_seriesEstop.count() > maxSamples)
    {
        m_seriesEstop.remove(0);
    }
    m_seriesEstop.append(vehData.estopData.receiveTimestamp, vehData.estopData.estopMsg.estopTriggered*99.0);

}






} //END namespace



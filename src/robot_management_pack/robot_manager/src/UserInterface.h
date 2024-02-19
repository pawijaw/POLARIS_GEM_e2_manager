#ifndef UserInterface_H
#define UserInterface_H


#include <QMainWindow>
#include <QTimer>
#include <QtCharts/QLineSeries>

#include "RobotManager.h"
#include "MockControl.h"


QT_BEGIN_NAMESPACE
namespace Ui { class UserInterface; }
QT_END_NAMESPACE

namespace RobotManager {


class UserInterface : public QMainWindow
{
    Q_OBJECT

public:
    UserInterface(QWidget *parent = nullptr);
    ~UserInterface();

private slots:
    void on_pushButtonGpsLoadTest_clicked(bool checked);
    void on_pushButtonBatteryTest_clicked(bool checked);
    void on_pushButtonLoadWps_clicked(bool checked);
    void on_radioButtonCtrlPure_clicked(bool checked);
    void on_radioButtonCtrlStanley_clicked(bool checked);
    void on_pushButtonTempTest_clicked(bool checked);
    void on_pushButtonV2xTest_clicked(bool checked);
    void on_pushButtonEstopTest_clicked(bool checked);

private:

    void onTimer();

    void appendToLog(const QString& str, double time = -1.0);

  //  bool navigationRequest(const std::string& navService);

  //  void appendToLog(const std::string& str);

    std::string getSelectedNavService() const;

    void setGuiNavActive(bool navActive);

    void setupChart();

    void addSampleToChart(const VehicleData& vehData);

private:
    Ui::UserInterface *ui;
    QTimer m_timer;

    ros::NodeHandle m_nodeHandle;

    RobotManager m_robotManager;
    MockControl m_mockControl;

    QtCharts::QLineSeries m_seriesGpsAccuracy;
    QtCharts::QLineSeries m_seriesBattery;
    QtCharts::QLineSeries m_seriesTemperature;
    QtCharts::QLineSeries m_seriesV2xSignal;
    QtCharts::QLineSeries m_seriesEstop;
};

} //END namespace
#endif // UserInterface_H

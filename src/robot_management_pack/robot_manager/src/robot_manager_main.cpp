

#include "UserInterface.h"

#include <QApplication>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_manager");

    QApplication application(argc, argv);
    RobotManager::UserInterface mgrWindow;
    mgrWindow.show();
    return application.exec();
}

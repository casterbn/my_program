/********************************************************************************
** Form generated from reading UI file 'openrtk_qt.ui'
**
** Created by: Qt User Interface Compiler version 5.12.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OPENRTK_QT_H
#define UI_OPENRTK_QT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_openrtk_qtClass
{
public:
    QAction *actionTest;
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menu;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QDockWidget *dockWidget_satellite;
    QWidget *dockContents_satellite;
    QVBoxLayout *vertical_satellite;
    QDockWidget *dockWidget_log;
    QWidget *dockContents_log;
    QVBoxLayout *vertical_log;
    QDockWidget *dockWidget_client;
    QWidget *dockContents_client;
    QVBoxLayout *vertical_client;

    void setupUi(QMainWindow *openrtk_qtClass)
    {
        if (openrtk_qtClass->objectName().isEmpty())
            openrtk_qtClass->setObjectName(QString::fromUtf8("openrtk_qtClass"));
        openrtk_qtClass->resize(1107, 871);
        actionTest = new QAction(openrtk_qtClass);
        actionTest->setObjectName(QString::fromUtf8("actionTest"));
        centralWidget = new QWidget(openrtk_qtClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        openrtk_qtClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(openrtk_qtClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1107, 22));
        menu = new QMenu(menuBar);
        menu->setObjectName(QString::fromUtf8("menu"));
        openrtk_qtClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(openrtk_qtClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        openrtk_qtClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(openrtk_qtClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        openrtk_qtClass->setStatusBar(statusBar);
        dockWidget_satellite = new QDockWidget(openrtk_qtClass);
        dockWidget_satellite->setObjectName(QString::fromUtf8("dockWidget_satellite"));
        dockWidget_satellite->setStyleSheet(QString::fromUtf8("QDockWidget > QWidget { \n"
"border: 1px solid gray; \n"
"}"));
        dockContents_satellite = new QWidget();
        dockContents_satellite->setObjectName(QString::fromUtf8("dockContents_satellite"));
        vertical_satellite = new QVBoxLayout(dockContents_satellite);
        vertical_satellite->setSpacing(6);
        vertical_satellite->setContentsMargins(11, 11, 11, 11);
        vertical_satellite->setObjectName(QString::fromUtf8("vertical_satellite"));
        vertical_satellite->setContentsMargins(0, 0, 0, 0);
        dockWidget_satellite->setWidget(dockContents_satellite);
        openrtk_qtClass->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dockWidget_satellite);
        dockWidget_log = new QDockWidget(openrtk_qtClass);
        dockWidget_log->setObjectName(QString::fromUtf8("dockWidget_log"));
        dockWidget_log->setStyleSheet(QString::fromUtf8("border: 1px solid gray; "));
        dockContents_log = new QWidget();
        dockContents_log->setObjectName(QString::fromUtf8("dockContents_log"));
        vertical_log = new QVBoxLayout(dockContents_log);
        vertical_log->setSpacing(6);
        vertical_log->setContentsMargins(11, 11, 11, 11);
        vertical_log->setObjectName(QString::fromUtf8("vertical_log"));
        vertical_log->setContentsMargins(0, 0, 0, 0);
        dockWidget_log->setWidget(dockContents_log);
        openrtk_qtClass->addDockWidget(static_cast<Qt::DockWidgetArea>(8), dockWidget_log);
        dockWidget_client = new QDockWidget(openrtk_qtClass);
        dockWidget_client->setObjectName(QString::fromUtf8("dockWidget_client"));
        dockWidget_client->setStyleSheet(QString::fromUtf8("QDockWidget > QWidget { \n"
"border: 1px solid gray; \n"
"}"));
        dockContents_client = new QWidget();
        dockContents_client->setObjectName(QString::fromUtf8("dockContents_client"));
        vertical_client = new QVBoxLayout(dockContents_client);
        vertical_client->setSpacing(6);
        vertical_client->setContentsMargins(11, 11, 11, 11);
        vertical_client->setObjectName(QString::fromUtf8("vertical_client"));
        vertical_client->setContentsMargins(0, 0, 0, 0);
        dockWidget_client->setWidget(dockContents_client);
        openrtk_qtClass->addDockWidget(static_cast<Qt::DockWidgetArea>(1), dockWidget_client);

        menuBar->addAction(menu->menuAction());
        menu->addAction(actionTest);

        retranslateUi(openrtk_qtClass);

        QMetaObject::connectSlotsByName(openrtk_qtClass);
    } // setupUi

    void retranslateUi(QMainWindow *openrtk_qtClass)
    {
        openrtk_qtClass->setWindowTitle(QApplication::translate("openrtk_qtClass", "openrtk_qt", nullptr));
        actionTest->setText(QApplication::translate("openrtk_qtClass", "Test", nullptr));
#ifndef QT_NO_SHORTCUT
        actionTest->setShortcut(QApplication::translate("openrtk_qtClass", "Alt+T", nullptr));
#endif // QT_NO_SHORTCUT
        menu->setTitle(QApplication::translate("openrtk_qtClass", "Windows(&W)", nullptr));
        dockWidget_satellite->setWindowTitle(QApplication::translate("openrtk_qtClass", "satellite", nullptr));
        dockWidget_log->setWindowTitle(QApplication::translate("openrtk_qtClass", "log", nullptr));
        dockWidget_client->setWindowTitle(QApplication::translate("openrtk_qtClass", "client", nullptr));
    } // retranslateUi

};

namespace Ui {
    class openrtk_qtClass: public Ui_openrtk_qtClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OPENRTK_QT_H

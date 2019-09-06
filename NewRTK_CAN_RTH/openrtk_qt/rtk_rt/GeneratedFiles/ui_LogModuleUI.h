/********************************************************************************
** Form generated from reading UI file 'LogModuleUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOGMODULEUI_H
#define UI_LOGMODULEUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LogModuleUI
{
public:
    QVBoxLayout *verticalLayout;
    QPlainTextEdit *m_plainTextEditResult;

    void setupUi(QWidget *LogModuleUI)
    {
        if (LogModuleUI->objectName().isEmpty())
            LogModuleUI->setObjectName(QString::fromUtf8("LogModuleUI"));
        LogModuleUI->resize(400, 300);
        verticalLayout = new QVBoxLayout(LogModuleUI);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(9, 9, 9, 9);
        m_plainTextEditResult = new QPlainTextEdit(LogModuleUI);
        m_plainTextEditResult->setObjectName(QString::fromUtf8("m_plainTextEditResult"));
        m_plainTextEditResult->setStyleSheet(QString::fromUtf8("QDockWidget > QWidget { \n"
"border: 1px solid gray; \n"
"}"));

        verticalLayout->addWidget(m_plainTextEditResult);


        retranslateUi(LogModuleUI);

        QMetaObject::connectSlotsByName(LogModuleUI);
    } // setupUi

    void retranslateUi(QWidget *LogModuleUI)
    {
        LogModuleUI->setWindowTitle(QApplication::translate("LogModuleUI", "LogModuleUI", nullptr));
    } // retranslateUi

};

namespace Ui {
    class LogModuleUI: public Ui_LogModuleUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOGMODULEUI_H

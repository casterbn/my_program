/********************************************************************************
** Form generated from reading UI file 'ClientModuleUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CLIENTMODULEUI_H
#define UI_CLIENTMODULEUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ClientModuleUI
{
public:
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_2;
    QRadioButton *m_radioButton_RTK;
    QRadioButton *m_radioButton_PPP;
    QSpacerItem *horizontalSpacer;
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_8;
    QLabel *label_7;
    QPlainTextEdit *m_plainTextEditData_1;
    QPlainTextEdit *m_plainTextEditData_2;
    QPlainTextEdit *m_plainTextEditData_3;
    QHBoxLayout *horizontalLayout;
    QPushButton *m_pushButtonStart;
    QPushButton *m_pushButtonClearData;

    void setupUi(QWidget *ClientModuleUI)
    {
        if (ClientModuleUI->objectName().isEmpty())
            ClientModuleUI->setObjectName(QString::fromUtf8("ClientModuleUI"));
        ClientModuleUI->resize(616, 419);
        verticalLayout_2 = new QVBoxLayout(ClientModuleUI);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        m_radioButton_RTK = new QRadioButton(ClientModuleUI);
        m_radioButton_RTK->setObjectName(QString::fromUtf8("m_radioButton_RTK"));
        m_radioButton_RTK->setChecked(true);

        horizontalLayout_2->addWidget(m_radioButton_RTK);

        m_radioButton_PPP = new QRadioButton(ClientModuleUI);
        m_radioButton_PPP->setObjectName(QString::fromUtf8("m_radioButton_PPP"));

        horizontalLayout_2->addWidget(m_radioButton_PPP);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);


        verticalLayout_2->addLayout(horizontalLayout_2);

        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(ClientModuleUI);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        label_2 = new QLabel(ClientModuleUI);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout->addWidget(label_2);

        label_3 = new QLabel(ClientModuleUI);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout->addWidget(label_3);

        label_4 = new QLabel(ClientModuleUI);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout->addWidget(label_4);

        label_5 = new QLabel(ClientModuleUI);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        verticalLayout->addWidget(label_5);

        label_6 = new QLabel(ClientModuleUI);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        verticalLayout->addWidget(label_6);

        label_8 = new QLabel(ClientModuleUI);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        verticalLayout->addWidget(label_8);


        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);

        label_7 = new QLabel(ClientModuleUI);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout->addWidget(label_7, 1, 0, 1, 1);

        m_plainTextEditData_1 = new QPlainTextEdit(ClientModuleUI);
        m_plainTextEditData_1->setObjectName(QString::fromUtf8("m_plainTextEditData_1"));

        gridLayout->addWidget(m_plainTextEditData_1, 1, 1, 1, 1);

        m_plainTextEditData_2 = new QPlainTextEdit(ClientModuleUI);
        m_plainTextEditData_2->setObjectName(QString::fromUtf8("m_plainTextEditData_2"));

        gridLayout->addWidget(m_plainTextEditData_2, 1, 2, 1, 1);

        m_plainTextEditData_3 = new QPlainTextEdit(ClientModuleUI);
        m_plainTextEditData_3->setObjectName(QString::fromUtf8("m_plainTextEditData_3"));
        m_plainTextEditData_3->setMaximumSize(QSize(16777215, 16777215));

        gridLayout->addWidget(m_plainTextEditData_3, 1, 3, 1, 1);


        verticalLayout_2->addLayout(gridLayout);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        m_pushButtonStart = new QPushButton(ClientModuleUI);
        m_pushButtonStart->setObjectName(QString::fromUtf8("m_pushButtonStart"));

        horizontalLayout->addWidget(m_pushButtonStart);

        m_pushButtonClearData = new QPushButton(ClientModuleUI);
        m_pushButtonClearData->setObjectName(QString::fromUtf8("m_pushButtonClearData"));

        horizontalLayout->addWidget(m_pushButtonClearData);


        verticalLayout_2->addLayout(horizontalLayout);


        retranslateUi(ClientModuleUI);

        QMetaObject::connectSlotsByName(ClientModuleUI);
    } // setupUi

    void retranslateUi(QWidget *ClientModuleUI)
    {
        ClientModuleUI->setWindowTitle(QApplication::translate("ClientModuleUI", "ClientModuleUI", nullptr));
        m_radioButton_RTK->setText(QApplication::translate("ClientModuleUI", "RTK", nullptr));
        m_radioButton_PPP->setText(QApplication::translate("ClientModuleUI", "PPP", nullptr));
        label->setText(QApplication::translate("ClientModuleUI", "Input Stream", nullptr));
        label_2->setText(QApplication::translate("ClientModuleUI", "Type", nullptr));
        label_3->setText(QApplication::translate("ClientModuleUI", "NTRIP Caster Host", nullptr));
        label_4->setText(QApplication::translate("ClientModuleUI", "Port", nullptr));
        label_5->setText(QApplication::translate("ClientModuleUI", "Mountpoint", nullptr));
        label_6->setText(QApplication::translate("ClientModuleUI", "User name", nullptr));
        label_8->setText(QApplication::translate("ClientModuleUI", "Password", nullptr));
        label_7->setText(QApplication::translate("ClientModuleUI", "Data log", nullptr));
        m_pushButtonStart->setText(QApplication::translate("ClientModuleUI", "Start", nullptr));
        m_pushButtonClearData->setText(QApplication::translate("ClientModuleUI", "clear", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ClientModuleUI: public Ui_ClientModuleUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CLIENTMODULEUI_H

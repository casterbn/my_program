/********************************************************************************
** Form generated from reading UI file 'SingleClientConfigUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SINGLECLIENTCONFIGUI_H
#define UI_SINGLECLIENTCONFIGUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SingleClientConfigUI
{
public:
    QVBoxLayout *verticalLayout;
    QCheckBox *m_checkBoxStream;
    QComboBox *m_comboBoxType;
    QComboBox *m_comboBoxHost;
    QLineEdit *m_lineEditPort;
    QComboBox *m_comboBoxMountpoint;
    QLineEdit *m_lineEditUser;
    QLineEdit *m_lineEditPassword;

    void setupUi(QWidget *SingleClientConfigUI)
    {
        if (SingleClientConfigUI->objectName().isEmpty())
            SingleClientConfigUI->setObjectName(QString::fromUtf8("SingleClientConfigUI"));
        SingleClientConfigUI->resize(218, 172);
        verticalLayout = new QVBoxLayout(SingleClientConfigUI);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        m_checkBoxStream = new QCheckBox(SingleClientConfigUI);
        m_checkBoxStream->setObjectName(QString::fromUtf8("m_checkBoxStream"));
        m_checkBoxStream->setChecked(true);

        verticalLayout->addWidget(m_checkBoxStream);

        m_comboBoxType = new QComboBox(SingleClientConfigUI);
        m_comboBoxType->addItem(QString());
        m_comboBoxType->setObjectName(QString::fromUtf8("m_comboBoxType"));
        m_comboBoxType->setEditable(false);

        verticalLayout->addWidget(m_comboBoxType);

        m_comboBoxHost = new QComboBox(SingleClientConfigUI);
        m_comboBoxHost->addItem(QString());
        m_comboBoxHost->setObjectName(QString::fromUtf8("m_comboBoxHost"));
        m_comboBoxHost->setEditable(true);

        verticalLayout->addWidget(m_comboBoxHost);

        m_lineEditPort = new QLineEdit(SingleClientConfigUI);
        m_lineEditPort->setObjectName(QString::fromUtf8("m_lineEditPort"));

        verticalLayout->addWidget(m_lineEditPort);

        m_comboBoxMountpoint = new QComboBox(SingleClientConfigUI);
        m_comboBoxMountpoint->addItem(QString());
        m_comboBoxMountpoint->addItem(QString());
        m_comboBoxMountpoint->addItem(QString());
        m_comboBoxMountpoint->addItem(QString());
        m_comboBoxMountpoint->addItem(QString());
        m_comboBoxMountpoint->addItem(QString());
        m_comboBoxMountpoint->addItem(QString());
        m_comboBoxMountpoint->addItem(QString());
        m_comboBoxMountpoint->setObjectName(QString::fromUtf8("m_comboBoxMountpoint"));
        m_comboBoxMountpoint->setEditable(true);

        verticalLayout->addWidget(m_comboBoxMountpoint);

        m_lineEditUser = new QLineEdit(SingleClientConfigUI);
        m_lineEditUser->setObjectName(QString::fromUtf8("m_lineEditUser"));

        verticalLayout->addWidget(m_lineEditUser);

        m_lineEditPassword = new QLineEdit(SingleClientConfigUI);
        m_lineEditPassword->setObjectName(QString::fromUtf8("m_lineEditPassword"));

        verticalLayout->addWidget(m_lineEditPassword);


        retranslateUi(SingleClientConfigUI);

        QMetaObject::connectSlotsByName(SingleClientConfigUI);
    } // setupUi

    void retranslateUi(QWidget *SingleClientConfigUI)
    {
        SingleClientConfigUI->setWindowTitle(QApplication::translate("SingleClientConfigUI", "SingleClientConfigUI", nullptr));
        m_checkBoxStream->setText(QApplication::translate("SingleClientConfigUI", "(1)Rover", nullptr));
        m_comboBoxType->setItemText(0, QApplication::translate("SingleClientConfigUI", "NTRIP Client", nullptr));

        m_comboBoxHost->setItemText(0, QApplication::translate("SingleClientConfigUI", "104.42.214.164", nullptr));

        m_lineEditPort->setText(QApplication::translate("SingleClientConfigUI", "2101", nullptr));
        m_comboBoxMountpoint->setItemText(0, QApplication::translate("SingleClientConfigUI", "SF01", nullptr));
        m_comboBoxMountpoint->setItemText(1, QApplication::translate("SingleClientConfigUI", "SF02", nullptr));
        m_comboBoxMountpoint->setItemText(2, QApplication::translate("SingleClientConfigUI", "SF03", nullptr));
        m_comboBoxMountpoint->setItemText(3, QApplication::translate("SingleClientConfigUI", "WX03", nullptr));
        m_comboBoxMountpoint->setItemText(4, QApplication::translate("SingleClientConfigUI", "WX04", nullptr));
        m_comboBoxMountpoint->setItemText(5, QApplication::translate("SingleClientConfigUI", "CLK93", nullptr));
        m_comboBoxMountpoint->setItemText(6, QApplication::translate("SingleClientConfigUI", "JPL_SSR", nullptr));
        m_comboBoxMountpoint->setItemText(7, QApplication::translate("SingleClientConfigUI", "BRDC", nullptr));

    } // retranslateUi

};

namespace Ui {
    class SingleClientConfigUI: public Ui_SingleClientConfigUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SINGLECLIENTCONFIGUI_H

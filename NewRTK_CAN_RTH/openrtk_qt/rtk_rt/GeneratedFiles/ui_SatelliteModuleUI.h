/********************************************************************************
** Form generated from reading UI file 'SatelliteModuleUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SATELLITEMODULEUI_H
#define UI_SATELLITEMODULEUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SatelliteModuleUI
{
public:
    QVBoxLayout *verticalLayout;
    QTableWidget *m_satelliteTable;

    void setupUi(QWidget *SatelliteModuleUI)
    {
        if (SatelliteModuleUI->objectName().isEmpty())
            SatelliteModuleUI->setObjectName(QString::fromUtf8("SatelliteModuleUI"));
        SatelliteModuleUI->resize(245, 550);
        verticalLayout = new QVBoxLayout(SatelliteModuleUI);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        m_satelliteTable = new QTableWidget(SatelliteModuleUI);
        if (m_satelliteTable->columnCount() < 2)
            m_satelliteTable->setColumnCount(2);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        m_satelliteTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        m_satelliteTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        m_satelliteTable->setObjectName(QString::fromUtf8("m_satelliteTable"));
        m_satelliteTable->setMaximumSize(QSize(16777215, 16777215));
        m_satelliteTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        m_satelliteTable->setSelectionMode(QAbstractItemView::SingleSelection);
        m_satelliteTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_satelliteTable->setHorizontalScrollMode(QAbstractItemView::ScrollPerItem);
        m_satelliteTable->verticalHeader()->setVisible(false);

        verticalLayout->addWidget(m_satelliteTable);


        retranslateUi(SatelliteModuleUI);

        QMetaObject::connectSlotsByName(SatelliteModuleUI);
    } // setupUi

    void retranslateUi(QWidget *SatelliteModuleUI)
    {
        SatelliteModuleUI->setWindowTitle(QApplication::translate("SatelliteModuleUI", "SatelliteModuleUI", nullptr));
        QTableWidgetItem *___qtablewidgetitem = m_satelliteTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("SatelliteModuleUI", "sat", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = m_satelliteTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("SatelliteModuleUI", "time", nullptr));
    } // retranslateUi

};

namespace Ui {
    class SatelliteModuleUI: public Ui_SatelliteModuleUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SATELLITEMODULEUI_H

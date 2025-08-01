/********************************************************************************
** Form generated from reading UI file 'widrocker.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WIDROCKER_H
#define UI_WIDROCKER_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_widrocker
{
public:
    QPushButton *BtnBack;

    void setupUi(QWidget *widrocker)
    {
        if (widrocker->objectName().isEmpty())
            widrocker->setObjectName(QString::fromUtf8("widrocker"));
        widrocker->resize(418, 548);
        BtnBack = new QPushButton(widrocker);
        BtnBack->setObjectName(QString::fromUtf8("BtnBack"));
        BtnBack->setGeometry(QRect(10, 0, 51, 51));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/back.png"), QSize(), QIcon::Normal, QIcon::Off);
        BtnBack->setIcon(icon);
        BtnBack->setIconSize(QSize(32, 32));

        retranslateUi(widrocker);

        QMetaObject::connectSlotsByName(widrocker);
    } // setupUi

    void retranslateUi(QWidget *widrocker)
    {
        widrocker->setWindowTitle(QApplication::translate("widrocker", "Form", nullptr));
        BtnBack->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class widrocker: public Ui_widrocker {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WIDROCKER_H

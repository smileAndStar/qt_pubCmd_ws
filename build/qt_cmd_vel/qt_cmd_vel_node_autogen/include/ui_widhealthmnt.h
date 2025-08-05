/********************************************************************************
** Form generated from reading UI file 'widhealthmnt.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WIDHEALTHMNT_H
#define UI_WIDHEALTHMNT_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_widhealthMnt
{
public:
    QPlainTextEdit *plainTextEdit;
    QPushButton *BtnBack2;

    void setupUi(QDialog *widhealthMnt)
    {
        if (widhealthMnt->objectName().isEmpty())
            widhealthMnt->setObjectName(QString::fromUtf8("widhealthMnt"));
        widhealthMnt->resize(966, 566);
        plainTextEdit = new QPlainTextEdit(widhealthMnt);
        plainTextEdit->setObjectName(QString::fromUtf8("plainTextEdit"));
        plainTextEdit->setGeometry(QRect(10, 10, 801, 541));
        BtnBack2 = new QPushButton(widhealthMnt);
        BtnBack2->setObjectName(QString::fromUtf8("BtnBack2"));
        BtnBack2->setGeometry(QRect(850, 500, 71, 51));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/back.png"), QSize(), QIcon::Normal, QIcon::Off);
        BtnBack2->setIcon(icon);

        retranslateUi(widhealthMnt);

        QMetaObject::connectSlotsByName(widhealthMnt);
    } // setupUi

    void retranslateUi(QDialog *widhealthMnt)
    {
        widhealthMnt->setWindowTitle(QApplication::translate("widhealthMnt", "Dialog", nullptr));
        BtnBack2->setText(QApplication::translate("widhealthMnt", "back", nullptr));
    } // retranslateUi

};

namespace Ui {
    class widhealthMnt: public Ui_widhealthMnt {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WIDHEALTHMNT_H

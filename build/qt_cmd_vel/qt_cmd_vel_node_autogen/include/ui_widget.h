/********************************************************************************
** Form generated from reading UI file 'widget.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WIDGET_H
#define UI_WIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Widget
{
public:
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *BtnLeft;
    QSpacerItem *horizontalSpacer;
    QVBoxLayout *verticalLayout_2;
    QPushButton *BtnForward;
    QSpacerItem *verticalSpacer;
    QPushButton *BtnStop;
    QSpacerItem *verticalSpacer_2;
    QPushButton *BtnBackward;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *BtnRight;
    QSpacerItem *horizontalSpacer_4;
    QSpacerItem *verticalSpacer_3;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QSpacerItem *horizontalSpacer_5;
    QSlider *LineSpeed;
    QSpacerItem *verticalSpacer_4;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_2;
    QSpacerItem *horizontalSpacer_6;
    QSlider *AngulSpeed;
    QSpacerItem *verticalSpacer_5;
    QPushButton *BtnRocker;
    QPushButton *BtnHealth;

    void setupUi(QWidget *Widget)
    {
        if (Widget->objectName().isEmpty())
            Widget->setObjectName(QString::fromUtf8("Widget"));
        Widget->resize(1415, 703);
        verticalLayoutWidget_2 = new QWidget(Widget);
        verticalLayoutWidget_2->setObjectName(QString::fromUtf8("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(210, 30, 936, 641));
        verticalLayout_3 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);

        BtnLeft = new QPushButton(verticalLayoutWidget_2);
        BtnLeft->setObjectName(QString::fromUtf8("BtnLeft"));
        BtnLeft->setMinimumSize(QSize(160, 100));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/right.png"), QSize(), QIcon::Normal, QIcon::Off);
        BtnLeft->setIcon(icon);
        BtnLeft->setIconSize(QSize(65, 65));

        horizontalLayout->addWidget(BtnLeft);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        BtnForward = new QPushButton(verticalLayoutWidget_2);
        BtnForward->setObjectName(QString::fromUtf8("BtnForward"));
        BtnForward->setMinimumSize(QSize(160, 100));
        BtnForward->setMaximumSize(QSize(160, 100));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/images/forward.png"), QSize(), QIcon::Normal, QIcon::Off);
        BtnForward->setIcon(icon1);
        BtnForward->setIconSize(QSize(64, 64));

        verticalLayout_2->addWidget(BtnForward);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        BtnStop = new QPushButton(verticalLayoutWidget_2);
        BtnStop->setObjectName(QString::fromUtf8("BtnStop"));
        BtnStop->setMinimumSize(QSize(160, 100));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/images/stop.png"), QSize(), QIcon::Normal, QIcon::Off);
        BtnStop->setIcon(icon2);
        BtnStop->setIconSize(QSize(64, 64));

        verticalLayout_2->addWidget(BtnStop);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_2);

        BtnBackward = new QPushButton(verticalLayoutWidget_2);
        BtnBackward->setObjectName(QString::fromUtf8("BtnBackward"));
        BtnBackward->setMinimumSize(QSize(160, 100));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/images/backward.png"), QSize(), QIcon::Normal, QIcon::Off);
        BtnBackward->setIcon(icon3);
        BtnBackward->setIconSize(QSize(64, 64));

        verticalLayout_2->addWidget(BtnBackward);


        horizontalLayout->addLayout(verticalLayout_2);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);

        BtnRight = new QPushButton(verticalLayoutWidget_2);
        BtnRight->setObjectName(QString::fromUtf8("BtnRight"));
        BtnRight->setMinimumSize(QSize(160, 100));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/images/left.png"), QSize(), QIcon::Normal, QIcon::Off);
        BtnRight->setIcon(icon4);
        BtnRight->setIconSize(QSize(65, 65));

        horizontalLayout->addWidget(BtnRight);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_4);


        verticalLayout_3->addLayout(horizontalLayout);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label = new QLabel(verticalLayoutWidget_2);
        label->setObjectName(QString::fromUtf8("label"));
        label->setMinimumSize(QSize(120, 80));
        label->setAlignment(Qt::AlignCenter);

        horizontalLayout_2->addWidget(label);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_5);

        LineSpeed = new QSlider(verticalLayoutWidget_2);
        LineSpeed->setObjectName(QString::fromUtf8("LineSpeed"));
        LineSpeed->setMinimumSize(QSize(780, 80));
        LineSpeed->setSizeIncrement(QSize(0, 0));
        LineSpeed->setMaximum(20);
        LineSpeed->setValue(2);
        LineSpeed->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(LineSpeed);


        verticalLayout_3->addLayout(horizontalLayout_2);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_4);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_2 = new QLabel(verticalLayoutWidget_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setMinimumSize(QSize(120, 80));
        label_2->setAlignment(Qt::AlignCenter);

        horizontalLayout_3->addWidget(label_2);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_6);

        AngulSpeed = new QSlider(verticalLayoutWidget_2);
        AngulSpeed->setObjectName(QString::fromUtf8("AngulSpeed"));
        AngulSpeed->setMinimumSize(QSize(780, 80));
        AngulSpeed->setMaximum(100);
        AngulSpeed->setValue(50);
        AngulSpeed->setOrientation(Qt::Horizontal);

        horizontalLayout_3->addWidget(AngulSpeed);


        verticalLayout_3->addLayout(horizontalLayout_3);

        verticalSpacer_5 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_5);

        BtnRocker = new QPushButton(Widget);
        BtnRocker->setObjectName(QString::fromUtf8("BtnRocker"));
        BtnRocker->setGeometry(QRect(1180, 30, 81, 71));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/images/Joystick.png"), QSize(), QIcon::Normal, QIcon::Off);
        BtnRocker->setIcon(icon5);
        BtnRocker->setIconSize(QSize(51, 51));
        BtnHealth = new QPushButton(Widget);
        BtnHealth->setObjectName(QString::fromUtf8("BtnHealth"));
        BtnHealth->setGeometry(QRect(1180, 120, 81, 61));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/images/medicine.png"), QSize(), QIcon::Normal, QIcon::Off);
        BtnHealth->setIcon(icon6);
        BtnHealth->setIconSize(QSize(60, 60));

        retranslateUi(Widget);

        QMetaObject::connectSlotsByName(Widget);
    } // setupUi

    void retranslateUi(QWidget *Widget)
    {
        Widget->setWindowTitle(QApplication::translate("Widget", "Widget", nullptr));
        BtnLeft->setText(QApplication::translate("Widget", "\345\267\246\350\275\254", nullptr));
        BtnForward->setText(QApplication::translate("Widget", "\345\211\215\350\277\233", nullptr));
        BtnStop->setText(QApplication::translate("Widget", "\345\201\234\346\255\242", nullptr));
        BtnBackward->setText(QApplication::translate("Widget", "\345\220\216\351\200\200", nullptr));
        BtnRight->setText(QApplication::translate("Widget", "\345\217\263\350\275\254", nullptr));
        label->setText(QApplication::translate("Widget", "\347\272\277\351\200\237\345\272\246", nullptr));
        label_2->setText(QApplication::translate("Widget", "\350\247\222\351\200\237\345\272\246", nullptr));
        BtnRocker->setText(QString());
        BtnHealth->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class Widget: public Ui_Widget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WIDGET_H

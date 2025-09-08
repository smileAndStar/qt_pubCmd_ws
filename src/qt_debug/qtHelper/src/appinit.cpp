﻿// #include "head.h"
#include "appinit.h"
#include "qmutex.h"
#include "qapplication.h"
#include "qevent.h"
#include "qwidget.h"
#include "qdebug.h"

SINGLETON_IMPL(AppInit)
AppInit::AppInit(QObject *parent) : QObject(parent)
{
}

bool AppInit::eventFilter(QObject *watched, QEvent *event)
{ 
    QWidget *w = (QWidget *)watched;
    if (!w->property("canMove").toBool()) {
        return QObject::eventFilter(watched, event);
    }

    static QPoint mousePoint;
    static bool mousePressed = false;

    int type = event->type();
    QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
    if (type == QEvent::MouseButtonPress) {
        if (mouseEvent->button() == Qt::LeftButton) {
            mousePressed = true;
            mousePoint = mouseEvent->globalPos() - w->pos();
        }
    } else if (type == QEvent::MouseButtonRelease) {
        mousePressed = false;
    } else if (type == QEvent::MouseMove) {
        if (mousePressed) {
            w->move(mouseEvent->globalPos() - mousePoint);
            return true;
        }
    }

    return QObject::eventFilter(watched, event);
}

void AppInit::start()
{
    qApp->installEventFilter(this);
}

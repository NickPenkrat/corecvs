#include <QtCore/QtCore>
/**
 * \file qtHelper.cpp
 *
 * \date Apr 27, 2013
 **/

#include "qtHelper.h"

using corecvs::Vector2dd;
using corecvs::Vector3dd;

QDebug & operator<< (QDebug & stream, const Vector2dd & vector)
{
    stream << "[" << vector.x() << "," << vector.y() << "]";
    return stream;
}

QDebug & operator<< (QDebug & stream, const Vector3dd & vector)
{
    stream << "[" << vector.x() << "," << vector.y() << "," << vector.z() << "]";

    return stream;
}

void setValueBlocking(QDoubleSpinBox *box, double value)
{
    bool wasBlocked = box->blockSignals(true);
    box->setValue(value);
    box->blockSignals(wasBlocked);
}


QDebug &operator<<(QDebug &stream, const corecvs::Vector2d<int> &vector)
{
    stream << "[" << vector.x() << "," << vector.y() << "]";
    return stream;
}

/* This class uses some of Qt Examples code*/
QString printWindowFlags(const Qt::WindowFlags &flags)
{
    QString text;

    Qt::WindowFlags type = (flags & Qt::WindowType_Mask);
    if (type == Qt::Window) {
        text = "Qt::Window";
    } else if (type == Qt::Dialog) {
        text = "Qt::Dialog";
    } else if (type == Qt::Sheet) {
        text = "Qt::Sheet";
    } else if (type == Qt::Drawer) {
        text = "Qt::Drawer";
    } else if (type == Qt::Popup) {
        text = "Qt::Popup";
    } else if (type == Qt::Tool) {
        text = "Qt::Tool";
    } else if (type == Qt::ToolTip) {
        text = "Qt::ToolTip";
    } else if (type == Qt::SplashScreen) {
        text = "Qt::SplashScreen";
    }

    if (flags & Qt::MSWindowsFixedSizeDialogHint)
        text += "\n| Qt::MSWindowsFixedSizeDialogHint";
    if (flags & Qt::X11BypassWindowManagerHint)
        text += "\n| Qt::X11BypassWindowManagerHint";
    if (flags & Qt::FramelessWindowHint)
        text += "\n| Qt::FramelessWindowHint";
    if (flags & Qt::NoDropShadowWindowHint)
        text += "\n| Qt::NoDropShadowWindowHint";
    if (flags & Qt::WindowTitleHint)
        text += "\n| Qt::WindowTitleHint";
    if (flags & Qt::WindowSystemMenuHint)
        text += "\n| Qt::WindowSystemMenuHint";
    if (flags & Qt::WindowMinimizeButtonHint)
        text += "\n| Qt::WindowMinimizeButtonHint";
    if (flags & Qt::WindowMaximizeButtonHint)
        text += "\n| Qt::WindowMaximizeButtonHint";
    if (flags & Qt::WindowCloseButtonHint)
        text += "\n| Qt::WindowCloseButtonHint";
    if (flags & Qt::WindowContextHelpButtonHint)
        text += "\n| Qt::WindowContextHelpButtonHint";
    if (flags & Qt::WindowShadeButtonHint)
        text += "\n| Qt::WindowShadeButtonHint";
    if (flags & Qt::WindowStaysOnTopHint)
        text += "\n| Qt::WindowStaysOnTopHint";
    if (flags & Qt::CustomizeWindowHint)
        text += "\n| Qt::CustomizeWindowHint";

    return text;
}

QString printWindowState(const Qt::WindowStates &state)
{
    QString text;
    if (state == Qt::WindowNoState)
        text +=" Qt::WindowNoState";	//The window has no state set (in normal state).
    if (state & Qt::WindowMinimized)
        text +=" Qt::WindowMinimized";	//The window is minimized (i.e. iconified).
    if (state & Qt::WindowMaximized)
        text +=" Qt::WindowMaximized";	//The window is maximized with a frame around it.
    if (state & Qt::WindowFullScreen)
        text +=" Qt::WindowFullScreen";	//The window fills the entire screen without any frame around it.
    if (state & Qt::WindowActive)
        text +=" Qt::WindowActive";	//The window is the active window, i.e. it has keyboard focus.

    return text;
}

QString printSelecionModel(const QItemSelectionModel::SelectionFlags &flag)
{
    QString text;

    if ( flag == QItemSelectionModel::NoUpdate)	text = "No selection";

    if (flag & QItemSelectionModel::Clear	) text += "Cleared.";
    if (flag & QItemSelectionModel::Select	) text += "Select.";
    if (flag & QItemSelectionModel::Deselect) text += "deselected.";
    if (flag & QItemSelectionModel::Toggle  ) text += "selected or deselected depending on their current state.";
    if (flag & QItemSelectionModel::Current ) text += "The current selection will be updated.";
    if (flag & QItemSelectionModel::Rows    ) text += "All indexes will be expanded to span rows.";
    if (flag & QItemSelectionModel::Columns ) text += "All indexes will be expanded to span columns.";

    return text;
}


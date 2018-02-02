#include "capSettingsDialog.h"
#include "parameterSelector.h"

#include <QMessageBox>
#include <QtCore/QSettings>
#include <QtCore/QDebug>

CapSettingsDialog::CapSettingsDialog(QWidget *parent, QString rootPath)
    :   QWidget(parent)
    ,   mRootPath(rootPath)
    ,   mUi(new Ui::CapSettingsDialog)
    ,   mCaptureInterface(NULL)
    ,   signalMapper(NULL)
    ,   resetSignalMapper(NULL)
{
    mUi->setupUi(this);

    setWindowTitle(rootPath);
    setWindowFlags(windowFlags() ^ Qt::WindowMinimizeButtonHint);

    refreshDialog();
}

void CapSettingsDialog::setCaptureInterface(ImageCaptureInterface *pInterface, bool updateOnFly)
{
    mCaptureInterface = pInterface;
    mUpdateOnFly = updateOnFly;
    refreshDialog();
}

void CapSettingsDialog::clearDialog()
{
    delete_safe(signalMapper);
    delete_safe(resetSignalMapper);

    QLayout *layout = mUi->scrollAreaWidgetContents->layout();
    QLayoutItem *item;
    while ((item = layout->takeAt(0)) != nullptr)
    {
        delete item->widget();
        delete_safe(item);
    }
}

void CapSettingsDialog::refreshDialog()
{
    clearDialog();
    if (mCaptureInterface == NULL)
        return;

    //ImageCaptureInterface::CapErrorCode res =
    mCaptureInterface->queryCameraParameters(mCameraParameters);

    mUi->updateOnFlyCheckBox->setChecked(mUpdateOnFly ? Qt::CheckState::Checked : Qt::CheckState::Unchecked);

    QLayout *layout = mUi->scrollAreaWidgetContents->layout();

    signalMapper      = new QSignalMapper(this);
    resetSignalMapper = new QSignalMapper(this);

    for (int id = CameraParameters::FIRST; id < CameraParameters::LAST; id++)
    {
        CaptureParameter &params = mCameraParameters.mCameraControls[id];
        if (!params.active())
            continue;

        ParameterEditorWidget *paramEditor = NULL;

        if (!params.isMenu() || !params.hasMenuItems())
        {
            ParameterSlider* paramSlider = new ParameterSlider(this);
            paramSlider->setMinimum(params.minimum());
            paramSlider->setMaximum(params.maximum());
            paramSlider->setStep   (params.step());
            paramEditor = paramSlider;
        }
        else
        {
            ParameterSelector* paramSelector = new ParameterSelector(this);
            for (unsigned index = 0; index < params.getMenuItemNumber(); index++)
            {
                paramSelector->pushOption(params.getMenuItem(index).c_str(),
                                          params.getMenuValue(index));
            }
            paramEditor = paramSelector;
        }

        paramEditor->setName(CameraParameters::names[id]);
        paramEditor->setAutoSupported(params.autoSupported());
        paramEditor->setEnabled(params.active());

        int value;
        mCaptureInterface->getCaptureProperty(id, &value);
        paramEditor->setValue(value);

        layout->addWidget(paramEditor);
        sliders.insert(id, paramEditor);

        connect(paramEditor, SIGNAL(valueChanged(int)), signalMapper, SLOT(map()));
        signalMapper->setMapping(paramEditor, id);

        connect(paramEditor, SIGNAL(resetPressed()), resetSignalMapper, SLOT(map()));
        resetSignalMapper->setMapping(paramEditor, id);
    }

    layout->addItem(new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Expanding));

    //layout->addChildLayout()(new QSpacerItem(0,0));
    connect(signalMapper     , SIGNAL(mapped(int)), this, SLOT(parameterChanged(int)));
    connect(resetSignalMapper, SIGNAL(mapped(int)), this, SLOT(resetPressed(int)));
}

void CapSettingsDialog::loadFromQSettings(const QString &fileName, const QString &_root, bool interfaceGroup)
{
    if (mCaptureInterface == NULL)
        return;

    QString interfaceName = QString::fromStdString(mCaptureInterface->getInterfaceName());
    if (interfaceName.isEmpty())
    {
        qDebug("CapSettingsDialog::loadFromQSettings(): Loading won't happen. InterfaceName is empty");
        return;
    }
    qDebug() << "CapSettingsDialog::loadFromQSettings(): Interface name: " << interfaceName;

    QSettings *settings = new QSettings(fileName, QSettings::IniFormat);
    settings->beginGroup(_root);
    settings->beginGroup(mRootPath);
    if (interfaceGroup)
        settings->beginGroup(interfaceName);

    QMapIterator<int, ParameterEditorWidget *> i(sliders);
    while (i.hasNext())
    {
        i.next();
        int id = i.key();
        const char *name = CameraParameters::names[id];
        ParameterEditorWidget *widget = i.value();
        double value = widget->value();
        double newValue = settings->value(name, value).toDouble();
        qDebug() << "   " << QString(name).leftJustified(16, ' ') << ": " << value << " -> " << newValue;
        widget->setValue(newValue);
    }

    if (interfaceGroup)
        settings->endGroup();
    settings->endGroup();
    settings->endGroup();

    delete_safe(settings);
}

void CapSettingsDialog::saveToQSettings(const QString &fileName, const QString &_root, bool interfaceGroup)
{
    qDebug() << QString("CapSettingsDialog::saveToQSettings(\"%1\", \"%2\"): called").arg(fileName, _root);
    if (mCaptureInterface == NULL)
    {
        qDebug() << "CapSettingsDialog::saveToQSettings(): mCaptureInterface is null";
        return;
    }

    QString interfaceName = QString::fromStdString(mCaptureInterface->getInterfaceName());
    if (interfaceName.isEmpty())
    {
        qDebug() << "CapSettingsDialog::saveToQSettings(): interface name is empty";
        return;
    }

    qDebug() << "CapSettingsDialog::saveToQSettings(): Interface name: " << interfaceName;

    std::unique_ptr<QSettings> settings(new QSettings(fileName, QSettings::IniFormat));
    settings->beginGroup(_root);
    settings->beginGroup(mRootPath);
    if (interfaceGroup)
        settings->beginGroup(interfaceName);

    QMapIterator<int, ParameterEditorWidget *> i(sliders);
    while (i.hasNext())
    {
        i.next();
        int id = i.key();
        const char *name = CameraParameters::names[id];
        ParameterEditorWidget *widget = i.value();
        double value = widget->value();
        settings->setValue(name, value);
        qDebug() << "   " << QString(name).leftJustified(16) << ": " << value;
    }

    if (interfaceGroup)
        settings->endGroup();
    settings->endGroup();
    settings->endGroup();
}

void CapSettingsDialog::refreshLimitsPressed()
{
    clearDialog();
    refreshDialog();
}

CapSettingsDialog::~CapSettingsDialog()
{
    clearDialog();
    mCaptureInterface = NULL;
    delete_safe(mUi);
}

void CapSettingsDialog::parameterChanged(int id)
{
    if (mCaptureInterface == NULL)
        return;

    bool updCam = mUi->updateOnFlyCheckBox->isChecked();
    int  v      = sliders[id]->value();

    qDebug() << "parameterChanged\t" << CameraParameters::names[id] << (updCam ? "cam" : "NO-cam") << "\tv:" << v;

    if (updCam)
        mCaptureInterface->setCaptureProperty(id, v);
}

void CapSettingsDialog::newCameraParamValue(int id)
{
    qDebug() << "CapSettingsDialog::newCameraParameterValue\t" << CameraParameters::names[id];

    if (mCaptureInterface == NULL)
        return;

    int v;
    mCaptureInterface->getCaptureProperty(id, &v);
    sliders[id]->setValue(v);

    qDebug() << "CapSettingsDialog::newCameraParameterValue\t" << CameraParameters::names[id] << "from cam" << "\tv:" << v;
}

void CapSettingsDialog::resetPressed(int id)
{
    if (mCaptureInterface == NULL)
        return;

    bool updCam = mUi->updateOnFlyCheckBox->isChecked();
    int  v      = mCameraParameters.mCameraControls[id].defaultValue();

    qDebug() << "resetPressed\t" << CameraParameters::names[id] << (updCam ? "cam" : "NO-cam") << "+dlg\tv:" << v;

    if (updCam)
        mCaptureInterface->setCaptureProperty(id, v);

    sliders[id]->setValue(v);
}

void CapSettingsDialog::resetAllPressed()
{
    for (int id = CameraParameters::FIRST; id < CameraParameters::LAST; id++)
    {
        CaptureParameter &param = mCameraParameters.mCameraControls[id];
        if (!param.active())
            continue;

        resetPressed(id);
    }
}

void CapSettingsDialog::updateAllPressed()
{
    for (int id = CameraParameters::FIRST; id < CameraParameters::LAST; id++)
    {
        CaptureParameter &param = mCameraParameters.mCameraControls[id];
        if (!param.active())
            continue;

        int prev;
        mCaptureInterface->getCaptureProperty(id, &prev);

        int v = sliders[id]->value();

        if (prev != v)
        {
            qDebug() << "updateAll\t" << CameraParameters::names[id] << "cam \tv:" << v;

            mCaptureInterface->setCaptureProperty(id, v);
        }
    }
}

void CapSettingsDialog::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        mUi->retranslateUi(this);
        break;
    default:
        break;
    }
}

void CapSettingsDialog::showEvent(QShowEvent *e)
{
    Q_UNUSED(e);
#if 0
    if (!disabledAlertShown  && !(mUi->aeEnabledCheckBox->isEnabled()))
    {
        disabledAlertShown = true;
        QMessageBox msgBox;
        msgBox.setText("Error querying camera exposure properties. You will not be able to modify exposure settings.\n"
                       "This may occur because you are using different camera devices or because your cameras do not support"
                       "exposure settings.");
        msgBox.exec();
    }
#endif
}

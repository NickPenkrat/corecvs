#include "aboutPropsTableWidget.h"
#include <QHeaderView>

#ifdef WITH_TBB
#include <tbb/tbb_stddef.h>
#endif

AboutPropsTableWidget::AboutPropsTableWidget(QWidget *parent) : QTableWidget(parent)
{   
    if (this->columnCount() < 2)
        this->setColumnCount(2);

    QTableWidgetItem *header1 = new QTableWidgetItem();
    QTableWidgetItem *header2 = new QTableWidgetItem();

    this->setHorizontalHeaderItem(0, header1);
    this->setHorizontalHeaderItem(1, header2);

    this->setObjectName(QStringLiteral("detailsTableWidget"));
    this->setFrameShadow(QFrame::Sunken);
    this->horizontalHeader()->setCascadingSectionResizes(false);
    this->horizontalHeader()->setDefaultSectionSize(300);


#define GCC_XSTR(value) GCC_STR(value)
#define GCC_STR(value) #value

#ifdef GIT_VERSION
    addParameter("Source Version", GCC_XSTR(GIT_VERSION));
#endif

#ifdef DEBUG
    addParameter("Build type", "debug");
#else
    addParameter("Build type", "release");
#endif

#ifdef WITH_SSE
    addParameter("SSE Support", "On");
#else
    addParameter("SSE Support", "Off");
#endif

#ifdef WITH_AVX
    addParameter("AVX Support", "On");
#else
    addParameter("AVX Support", "Off");
#endif

#ifdef WITH_AVX2
    addParameter("AVX2 Support", "On");
#else
    addParameter("AVX2 Support", "Off");
#endif

#ifdef WITH_FMA
    addParameter("FMA Support", "On");
#else
    addParameter("FMA Support", "Off");
#endif

#ifdef WITH_TBB
    addParameter("TBB Support", "On");
    addParameter("TBB Version", GCC_XSTR(TBB_INTERFACE_VERSION));
    addParameter("TBB Compat Version", GCC_XSTR(TBB_COMPATIBLE_INTERFACE_VERSION));

    int runtimeTBB = tbb::TBB_runtime_interface_version();
    addParameter("TBB Runtime Version", QString::number(runtimeTBB));
#else
    addParameter("TBB Support", "Off");
#endif

#ifdef WITH_BLAS
#ifdef WITH_MKL
    addParameter("BLAS Support", "On (Intel MKL)");
#endif
#ifdef WITH_OPENBLAS
    addParameter("BLAS Support", "On (OpenBLAS)");
#endif
#else
    addParameter("BLAS Support", "Off");
#endif

#ifdef WITH_OPENCV
    addParameter("OpenCV Support", "On");
#else
    addParameter("OpenCV Support", "Off");
#endif

#if    defined (Q_OS_WIN)
    addParameter("Target OS", "Windows");
#elif  defined (Q_OS_MAC)
    addParameter("Target OS", "Mac OS");
#elif  defined (Q_OS_LINUX)
    addParameter("Target OS", "Linux");
#endif

#ifdef __x86_64
     addParameter("Platform", "64bit");
#endif

#if defined (__GNUC__) && defined (__GNUC_MINOR__)
     addParameter("Compiler", "GCC");
     addParameter("Compiler Version", GCC_XSTR(__GNUC__) "." GCC_XSTR(__GNUC_MINOR__));
#elif defined (__clang__)
     addParameter("Compiler", "CLANG");
#elif defined (_MSC_VER)
     addParameter("Compiler", "MSVC");
#elif defined (__MINGW32__)
     addParameter("Compiler", "MinGW");
#elif defined (__INTEL_COMPILER)
     addParameter("Compiler", "ICC");
#endif

#undef GCC_STR
#undef GCC_XSTR

}

AboutPropsTableWidget::~AboutPropsTableWidget()
{
}

void AboutPropsTableWidget::addParameter(QString name, QString value)
{
    int pos = this->rowCount();
    this->insertRow(pos);
    QTableWidgetItem *item1 = new QTableWidgetItem(name);
    QTableWidgetItem *item2 = new QTableWidgetItem(value);

    this->setItem(pos, 0, item1);
    this->setItem(pos, 1, item2);
}

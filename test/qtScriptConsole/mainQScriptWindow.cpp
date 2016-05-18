#include <QScriptEngine>
#include <QDebug>
#include <QDateTime>

#include "mainQScriptWindow.h"
#include "ui_mainQScriptWindow.h"

MainQScriptWindow::MainQScriptWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainQScriptWindow)
{
    ui->setupUi(this);
    connect(ui->executeButton, SIGNAL(released()), this, SLOT(executeScript()));
}

void MainQScriptWindow::executeScript(void)
{
    QString scriptText = ui->textEdit->toPlainText();


    qDebug() << "Parsing:" << scriptText;

    QScriptSyntaxCheckResult parseResult = QScriptEngine::checkSyntax(scriptText);

    if (parseResult.state() == QScriptSyntaxCheckResult::Error)
    {
         qDebug() << "Error:" << parseResult.errorMessage();
         qDebug() << " At:" << parseResult.errorLineNumber() << ":" << parseResult.errorColumnNumber();

         QStringList lines = scriptText.split("\n");
         for (int n = 0; n < lines.size(); n++)
         {
             QString line = lines[n];
             qDebug() << n << " - " << line;
             lines[n] = lines[n].toHtmlEscaped();
         }
         int lnum = parseResult.errorLineNumber() - 1;
         int cnum = parseResult.errorColumnNumber() - 1;

         if (lines.size() <= lnum) {
             qDebug() << "No such line" << endl;
             return;
         }

         lines[lnum].insert(cnum + 1, "</font>");
         lines[lnum].insert(cnum, "<font color='red'>");

         lines[lnum].prepend("<font color='blue'>");
         lines[lnum].append("</font>");

         scriptText = lines.join("<br/>");
         ui->textEdit->setHtml(scriptText);
         qDebug() << "Script Text:" << scriptText;
         return;
    }

    if (parseResult.state() == QScriptSyntaxCheckResult::Intermediate)
    {
        qDebug() << "Error:" << parseResult.errorMessage();
        qDebug() << " At:" << parseResult.errorLineNumber() << ":" << parseResult.errorColumnNumber();
    }

    if (parseResult.state() == QScriptSyntaxCheckResult::Valid)
    {
        qDebug() << "Program is ok";
    }




   /*=============================*/

    QScriptEngine engine;
    qDebug() << "Executing:" << scriptText;


    QScriptValue value = engine.evaluate(scriptText);

    if (value.isBool()) {
        qDebug() << "Result is bool:" << value.toBool();
    }

    if (value.isNumber()) {
        qDebug() << "Result is int:" << value.toInteger();
    }

    if (value.isArray()) {
        qDebug() << "Result is array:";
    }

    if (value.isString()) {
        qDebug() << "Result is string: <" << value.toString() << ">";
    }

    if (value.isObject()) {
        qDebug() << "Result is Object";
        QObject *obj = value.toQObject();
        if (obj != NULL) {
            qDebug() << "  name:" << value.toQObject()->objectName();
        } else {
            qDebug() << "  (NULL)";
        }
    }


    if (value.isDate()) {
        qDebug() << "Result is date:" << value.toDateTime();
    }

    if (engine.hasUncaughtException()) {
        QScriptValue errors = engine.uncaughtException();
        if (errors.isError()) {
            qDebug() << "Error: " << errors.toString();
        }
        QStringList backtrace = engine.uncaughtExceptionBacktrace();
        {
            qDebug() << backtrace;
        }
    }


}

MainQScriptWindow::~MainQScriptWindow()
{
    delete ui;
}

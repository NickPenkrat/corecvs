#include <QApplication>
#include <QFileInfo>

#include <iostream>

#include "blurApplicator.h"
#include "resizeApplicator.h"
#include "jsonSetter.h"
#include "jsonGetter.h"

void helpMessage()
{
    std::cout << "Images augmentation utility.\n"
              << "Usage: imageAugment command <arguments>\n"
              << "Commands are:\n"
              << "blur - apply blur to images selected by template\n\t" << BlurApplicator().helpMessage() << "\n"
              << "blurtask - apply blur to images specified in task file\n\t"
              << "Usage : blurtask [taskfile=task.json]\n"
              << "resize - apply resize transformation to images selected by template\n\t" << ResizeApplicator().helpMessage() << "\n"
              << "resizetask - apply resize transformation to images specified in task file\n\t"
              << "Usage : resizetask [taskfile=task.json]\n"
              << "help - this message\n";
}

template<typename T>
void applyNewTask(Applicator<T>* applicator, const QStringList &params)
{
    applicator->applyParameters(params);
    applicator->allApplyAugment();
    QFileInfo task("task.json");
    JSONSetter setter(task.absoluteFilePath());
    setter.visit(*applicator, "augmentation");
    std::cout << "Configuration stored into " << task.absoluteFilePath().toStdString() << std::endl;
}

template<typename T>
void applyStoredTask(Applicator<T>* applicator, const QStringList &params)
{
    QString taskfile = "task.json";
    if(params.size() > 2)
        taskfile = params.at(2);
    QFileInfo task(taskfile);
    if(!task.exists())
        std::cerr << "No taskfile " << task.absoluteFilePath().toStdString() << " found\n";
    std::cout << "Configuration restored from " << task.absoluteFilePath().toStdString() << std::endl;
    JSONGetter getter(task.absoluteFilePath());
    getter.visit(*applicator, "augmentation");
    applicator->allApplyAugment();
}

template<typename T>
void applyTask(bool newTask, const QStringList &params)
{
    T applicator;
    if(newTask)
        applyNewTask(&applicator, params);
    else
        applyStoredTask(&applicator, params);
}

int main(int argc, char *argv[])
{
    Q_INIT_RESOURCE(main);
//    Q_INIT_RESOURCE(resresource);

    QApplication a(argc, argv);
    QStringList params = a.arguments();
    QStringList commands;
    commands << "help"        //0
             << "blur"        //1
             << "blurtask"    //2
             << "resize"      //3
             << "resizetask"  //4

                ;

    if(params.size() < 2 )
    {
        std::cerr << "Too few arguments\n\n";
        helpMessage();
        return 1;
    }

    int command = commands.indexOf(params.at(1));
    switch (command)
    {
    case 0:
    {
        helpMessage();
        return 0;
        break;
    }
    case 1:
    case 2:
    {
        applyTask<BlurApplicator>(command % 2, params);
        break;
    }
    case 3:
    case 4:
    {
        applyTask<ResizeApplicator>(command % 2, params);
        break;
    }
    default:
    {
        std::cerr << "Unknown command\n\n";
        helpMessage();
        return 1;
        break;
    }
    }
}

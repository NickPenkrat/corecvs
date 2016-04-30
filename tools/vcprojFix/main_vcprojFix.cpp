#include <string>
#include <iostream>

#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <QtCore/QDirIterator>

using namespace std;

int main(int argc, char **argv)
{
    int result = -1;
    if (argc < 4) {
        cout << "Please use with params: <path_to_projects> <mask> <support_x64_mode>" << endl;
        return result;
    }

    QString path(argv[1]);
    QString mask(argv[2]);
    QString md64(argv[3]);

    QString vcprojExt(mask);  vcprojExt.replace("*", "");

    bool bAddPlatformWinSDK = md64.contains("winsdk", Qt::CaseInsensitive);
    if (bAddPlatformWinSDK) {
        cout << "The mode to adapt projects for Window SDK usage is ON" << endl;
    }
    cout << "Scanning '" << path.toStdString().c_str()
         << "' for MSVC project files with mask '"
         << mask.toStdString().c_str() << "'..." << endl;

    QStringList filters; filters.append(mask);
    QDirIterator it(path, filters, QDir::Files|QDir::NoSymLinks, QDirIterator::Subdirectories);

    while (it.hasNext())
    {
        QString path = it.next();

        string spaces(max(1, 50 - path.length()), ' ');
        cout << "Process '" << path.toStdString().c_str() << "':" << spaces.c_str() << "\t";

        if (path.contains("_bkp" + vcprojExt)) {
            cout << "skip" << endl;
            continue;
        }
        cout << "backup";

        QString pathNew(path);
        pathNew.replace(vcprojExt, "_bkp" + vcprojExt);

        QFile::remove(pathNew);
        if (!QFile::copy(path, pathNew)) {
            cout << endl << "Couldn't create backup file '" << pathNew.toStdString().c_str() << "' Stop!" << endl;
            return result;
        }
        cout << "ed";

        QFile fileI(pathNew);
        if (!fileI.open(QIODevice::ReadOnly))
        {
            cout << endl << "Couldn't open file '" << pathNew.toStdString().c_str() << "' Stop!" << endl;
            return result;
        }

        QFile fileO(path);
        if (!fileO.open(QIODevice::WriteOnly|QIODevice::Truncate))
        {
            cout << endl << "Couldn't open file '" << path.toStdString().c_str() << "' Stop!" << endl;
            return result;
        }
        cout << ", convert";

        QTextStream is(&fileI);
        QTextStream os(&fileO);
        while (!is.atEnd())
        {
            QString line = is.readLine();
            line.replace("@echo;", "");
            line.replace("&apos;", "'");
            line.replace("&quot;", "\"");

            os << line << endl;

            if (bAddPlatformWinSDK && line.contains("<PrimaryOutput>"))
            {
                os << "    <PlatformToolset>Windows7.1SDK</PlatformToolset>" << endl;
            }
        }

        cout << "ed" << endl;

        fileI.close();
        fileO.close();

        // kill empty debug&release folders, which qmake creates because of bug
        path = QFileInfo(path).path();
        QDir dir;
        dir.rmdir(path + "/debug");
        dir.rmdir(path + "/release");
    }

    return 0;
}

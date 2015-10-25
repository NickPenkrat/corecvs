#include <iostream>

#include "log.h"
#include "commandLineSetter.h"

const char *Log::level_names[] =
{
     "D.Debug"     // LEVEL_DETAILED_DEBUG, /**< Detailed log */
   , "Debug"       // LEVEL_DEBUG,          /**< Debugging information is outputed */
   , "Info"        // LEVEL_INFO,           /**< Normal information messages are outputed */
   , "Warning"     // LEVEL_WARNING,        /**< Only warnings are reported */
   , "Error"       // LEVEL_ERROR,          /**< Only errors are reported */
                   // LEVEL_LAST            /**< Last enum value for iterating*/
};

STATIC_ASSERT(CORE_COUNT_OF(Log::level_names) == Log::LEVEL_LAST, wrong_number_of_log_levels);

/**
 * It is impossible to tell when this function will be executed, so you should not log from
 * static initializers
 **/
Log::LogLevel           Log::mMinLogLevel = LEVEL_FIRST;
std::vector<LogDrain *> Log::mLogDrains;

int Log::mDummy = Log::staticInit();

int Log::staticInit()
{
    LogDrain *defaultDrain = new StdStreamLogDrain(std::cout);
    mLogDrains.push_back(defaultDrain);
//	logStream.reset(new std::ofstream(fileName.c_str(), std::ios::out | std::ios::app));
    return 0;
}

void Log::message(Message &message)
{
    for (unsigned int i = 0; i < mLogDrains.size(); i++)
    {
        mLogDrains[i]->drain(message);
    }
}

Log::Log(const LogLevel /*maxLocalLevel*/)
{}

Log::~Log()
{}

//std::string Log::msgBufToString(const char* msg)
//{
//    std::string message(msg);
//
//    if (message.size() != 0 && message[message.size() - 1] == '\n') {
//        message.resize(message.size() - 1);
//    }
//
//    return message;
//}

//static
const char* Log::levelName(LogLevel logLevel)
{
    return (unsigned)logLevel < LEVEL_LAST ? level_names[logLevel] : "unknown";
}

//static
std::string Log::formatted(const char *format, ...)
{
    std::string result;

    va_list marker;
    va_start(marker, format);
        size_t len = vsnprintf(NULL, 0, format, marker) + 1;    // to add a nul symbol
    va_end(marker);

    va_list marker2;
    va_start(marker2, format);
#if 1
        char* buf = new char[len];
        vsnprintf(buf, len, format, marker2);
        result.assign(buf);
        delete[] buf;
#else
        // TODO: this code inserts one dummy symbol at begin of the result?!!
        result.resize(len);
      //vsnprintf(result._Myptr(), len, format, marker2);
        vsnprintf(&result[0], len, format, marker2);
#endif
    va_end(marker2);

    return result;
}

cchar* LogDrain::time2str(time_t &time)
{
    struct tm *timeinfo = localtime(&time);

    snprintf2buf(timeBuffer, "%d-%d-%d %02d:%02d:%02d",
        timeinfo->tm_year + 1900,
        timeinfo->tm_mon  + 1,
        timeinfo->tm_mday,
        timeinfo->tm_hour,
        timeinfo->tm_min,
        timeinfo->tm_sec
        );
    return timeBuffer;
}

void StdStreamLogDrain::drain(Log::Message &message)
{
    std::ostringstream prefix;

    if (message.get()->mThreadId != 0)
    {
        prefix << message.get()->mThreadId << ";";
    }
    prefix << time2str(message.get()->mTime)        << ";"
           << Log::levelName(message.get()->mLevel) << ";"
           << message.get()->mOriginFileName        << ";"
           << message.get()->mOriginLineNumber      << ";"
           << message.get()->mOriginFunctionName    << "() ";

    mMutex.lock();
        size_t len = prefix.str().size();
        mOutputStream << prefix.str() << '\t';

        const std::string &messageString = message.get()->s.str();
        size_t pos = 0;
        do {
            if (pos != 0) {
                mOutputStream << std::string(len, ' ') << '\t';
            }
            size_t posBr = messageString.find('\n', pos);

            mOutputStream << messageString.substr(pos, posBr - pos) << std::endl;

            if (posBr == std::string::npos)
                break;
            pos = posBr + 1;

        } while (true);
        mOutputStream.flush();
    mMutex.unlock();
}


FileLogDrain::FileLogDrain(const std::string &path, bool bAppend)
    : mFile(path.c_str(), bAppend ? std::ios_base::app : std::ios_base::trunc)
{}

FileLogDrain::~FileLogDrain()
{
    mFile.flush();
    mFile.close();
}

void FileLogDrain::drain(Log::Message &message)
{
    if (mFile.is_open())
    {
        mMutex.lock();
            mFile << message.get()->s.str() << std::endl;
            mFile.flush();
        mMutex.unlock();
    }
}

void LiteStdStreamLogDrain::drain(Log::Message &message)
{
    mMutex.lock();
        mOutputStream << message.get()->s.str() << std::endl;
        mOutputStream.flush();
    mMutex.unlock();
}

void Log::addAppLog(int argc, char* argv[], cchar* logFileName)
{
    /** detect min LogLevel and log filename from params
     */
    corecvs::CommandLineSetter setter(argc, (const char **)argv);
    Log::LogLevel minLogLevel = (Log::LogLevel)setter.getInt("logLevel", Log::LEVEL_INFO);
    std::string   logFile     = setter.getString("logFile", logFileName ? logFileName : "");

    /** add needed log drains
     */
    cchar* progPath = argv[0];
    std::string pathApp(progPath);
    size_t pos = pathApp.rfind(PATH_SEPARATOR);
    pathApp.resize(pos + 1);                            // extract path to the program

    if (!logFile.empty())
    {
        //Log::mLogDrains.clear();
        //Log::mLogDrains.push_back(new LiteStdStreamLogDrain(std::cout));
        Log::mLogDrains.push_back(new FileLogDrain(pathApp + logFile));
    }

#ifdef GIT_VERSION
# define GCC_XSTR(value) GCC_STR(value)
# define GCC_STR(value)  #value
    cchar* git_version = GCC_XSTR(GIT_VERSION);
#else
    cchar* git_version = "unknown";
#endif

    L_INFO_P("%s app built %s %s", &progPath[pos + 1], __DATE__, __TIME__);
    L_INFO_P("%s app version %s" , &progPath[pos + 1], git_version);
    Log::mMinLogLevel = LEVEL_DETAILED_DEBUG;
    L_INFO_P("App Log Level: %s", Log::levelName(minLogLevel));
    Log::mMinLogLevel = minLogLevel;
}

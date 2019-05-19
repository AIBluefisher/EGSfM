#include <string>
#include <iostream>

#include "rang.hpp"

enum LogType { ERROR, WARNING, TIP, INFO, HEAD, TEXT };

namespace GraphSfM {
namespace log {

class Logger 
{
public:
    static void LogToScreen(std::string, LogType log_type);
    static void LogToFile(std::ofstream& os);
};

} // log
} // GraphSfM
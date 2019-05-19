#include "log.h"

namespace GraphSfM {
namespace log {
void Logger::LogToScreen(std::string str, LogType log_type)
{
    switch (log_type) {
        case ERROR: 
            std::cout << rang::fg::red << str << rang::style::reset;
            break;
        case WARNING:
            std::cout << rang::fg::yellow << str << rang::style::reset;
            break;
        case TIP:
            std::cout << rang::bg::magenta << str << rang::style::reset;
            break;
        case INFO:
            std::cout << rang::fg::green << str << rang::fg::reset << rang::style::reset;
            break;
        case HEAD:
            std::cout << rang::style::bold << str << rang::style::reset;
            break;
        case TEXT:
            std::cout << rang::fg::black << str << rang::style::reset;
            break;
        default: 
            break;
    }
    std::cout << rang::fg::reset;
}

void Logger::LogToFile(std::ofstream& os)
{
    // TODO:
}

} // namespace log
} // namespace GraphSfM
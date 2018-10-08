#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <atomic>
#include <ctime>
#include <chrono>

#include "NaoPoseInfo.hpp"
#include "utils.hpp"

namespace utils
{

std::vector<std::string> split(const std::string &s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

std::string getISOTimeString()
{
    time_t now;
    time(&now);
    char buf[sizeof "2011-10-08T07:07:09Z"];
    strftime(buf, sizeof buf, "%FT%TZ", gmtime(&now));
    // this will work too, if your compiler doesn't support %F or %T:
    //strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
    return std::string(buf);
}

std::string getMilliSecondsString()
{
    using namespace std::chrono;

    // // get current time
    auto now = system_clock::now();

    // // get number of milliseconds for the current second
    // // (remainder after division into seconds)
    // auto ms = duration_cast<milliseconds>(now.time_since_epoch()).count();
    return std::string(std::to_string(duration_cast<milliseconds>(now.time_since_epoch()).count()));
    // std::ostringstream oss;
    // oss << ms;
    // return oss.str();
    // return "";
}

} // namespace utils

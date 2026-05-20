#include <ctime>
#include <string>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <iostream>

// use strftime to format time_t into a "date time"
std::string date_time(std::time_t posix)
{
    char buf[20]; // big enough for 2015-07-08 10:06:51\0
    std::tm tp = *std::localtime(&posix);
    return {buf, std::strftime(buf, sizeof(buf), "%F %T", &tp)};
}

std::string stamp()
{
    using namespace std;
    using namespace std::chrono;

    // get absolute wall time
    auto now = system_clock::now();

    // find the number of milliseconds
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    // build output string
    std::ostringstream oss;
    oss.fill('0');

    // convert absolute time to time_t seconds
    // and convert to "date time"
    oss << date_time(system_clock::to_time_t(now));
    oss << '.' << setw(3) << ms.count();

    return oss.str();
}

int main()
{
    std::cout << stamp() << '\n';
}
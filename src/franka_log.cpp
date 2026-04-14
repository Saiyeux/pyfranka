//
// Created by cyshi on 2021/2/26.
//
#include "franka_log.h"

void writeLogToFile(const std::vector<franka::Record>& log) {
    if (log.empty()) {
        return;
    }
    try {
        Poco::Path temp_dir_path(Poco::Path::current());
        temp_dir_path.pushDirectory("libfranka-logs");

        Poco::File temp_dir(temp_dir_path);
        temp_dir.createDirectories();

        std::string now_string =
                Poco::DateTimeFormatter::format(Poco::LocalDateTime{}, "%Y-%m-%d-%H-%M-%S-%i");
        std::string filename = std::string{"log-" + now_string + ".csv"};
        Poco::File log_file(Poco::Path(temp_dir_path, filename));
        if (!log_file.createFile()) {
            std::cout << "Failed to write log file." << std::endl;
            return;
        }
        std::ofstream log_stream(log_file.path().c_str());
        log_stream << franka::logToCSV(log);

        std::cout << "Log file written to: " << log_file.path() << std::endl;
    } catch (...) {
        std::cout << "Failed to write log file." << std::endl;
    }
}

void test_path()
{
    Poco::Path temp_dir_path(Poco::Path::current());
    temp_dir_path.pushDirectory("libfranka-logs");

    Poco::File temp_dir(temp_dir_path);
    temp_dir.createDirectories();

    std::string now_string = Poco::DateTimeFormatter::format(Poco::LocalDateTime{}, "%Y-%m-%d-%H-%M-%S-%i");
    std::string filename = std::string{"log-" + now_string + ".csv"};
    Poco::File log_file(Poco::Path(temp_dir_path, filename));

    if (!log_file.createFile()) {
        std::cout << "Failed to write log file." << std::endl;
        return;
    }
    std::ofstream log_stream(log_file.path().c_str());
    log_stream << "111";

    std::cout << "temp_dir_path: " << temp_dir.path() << std::endl;

}



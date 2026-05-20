#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include <unistd.h>

#include "xutils/XCfg/xini.h"
#include "xutils/XLog/xlog.h"
#include "xutils/xutils.h"

namespace {

struct LogConfig {
    bool logEnable = true;
    int logLevel = XLOG_LEVEL_INFO;
    bool intervalEnable = false;
    int intervalMinutes = 30;
    std::string configDir = "./config";
};

std::string configDirFromEnv()
{
    const char* env = std::getenv("FJ212_CONFIG_DIR");
    if (env != nullptr && env[0] != '\0') {
        return env;
    }
    return "./config";
}

template <typename T>
bool readIniValue(ini::IniFile& iniFile, const std::string& section,
                  const std::string& key, T& value)
{
    try {
        value = iniFile.GetProperty(section, key).as<T>();
        return true;
    } catch (const std::exception& ex) {
        std::cerr << "daemon: use default for [" << section << "] " << key
                  << ": " << ex.what() << std::endl;
        return false;
    }
}

LogConfig loadLogConfig()
{
    LogConfig config;
    config.configDir = configDirFromEnv();

    const auto configPath =
        std::filesystem::path(config.configDir) / "logInterval.cfg";
    if (!std::filesystem::exists(configPath)) {
        std::cerr << "daemon: config not found: " << configPath
                  << ", use defaults" << std::endl;
        return config;
    }

    try {
        ini::IniFile configIni;
        configIni.setMultiLineValues(true);
        configIni.load(configPath.string());

        readIniValue(configIni, "Interval", "enLog_b", config.logEnable);
        readIniValue(configIni, "Interval", "logLevel_i", config.logLevel);
        readIniValue(configIni, "Interval", "enInterval_b",
                     config.intervalEnable);
        readIniValue(configIni, "Interval", "interval",
                     config.intervalMinutes);
    } catch (const std::exception& ex) {
        std::cerr << "daemon: failed to load " << configPath
                  << ", use defaults: " << ex.what() << std::endl;
    }

    if (config.intervalMinutes <= 0) {
        std::cerr << "daemon: invalid interval, use 30 minutes" << std::endl;
        config.intervalMinutes = 30;
    }

    if (config.logLevel < XLOG_LEVEL_FATAL || config.logLevel > XLOG_LEVEL_DEBUG) {
        std::cerr << "daemon: invalid log level, use info" << std::endl;
        config.logLevel = XLOG_LEVEL_INFO;
    }

    return config;
}

std::string logrotateCommand(const std::string& configDir)
{
    const auto logConfig = std::filesystem::path(configDir) / "logconfig.cfg";
    return "logrotate " + logConfig.string();
}

}  // namespace

int main(int /*argc*/, char* /*argv*/[])
{
    const LogConfig config = loadLogConfig();

    std::unique_ptr<XLog> xlog = std::make_unique<XLog>(config.logEnable);
    xlog->Initialise("logDaemon.log");
    xlog->SetThreshold(static_cast<Type>(config.logLevel));

    xlog->Info(">>>>>>>>>>> daemon start!!! <<<<<<<<<<");

    const uint32_t startTime = getTimeStap_ms();
    uint32_t heartbeatLastTime = startTime;
    uint32_t logLastTime = startTime;
    const std::string rotateCommand = logrotateCommand(config.configDir);

    while (true) {
        const uint32_t curTime = getTimeStap_ms();

        if ((curTime - heartbeatLastTime) >= 60 * 1000) {
            heartbeatLastTime = curTime;
            xlog->Info(">>>>>>>>>>> daemon is working!!! <<<<<<<<<<");
        }

        if (config.intervalEnable &&
            (curTime - logLastTime) >=
                static_cast<uint32_t>(config.intervalMinutes) * 60 * 1000) {
            logLastTime = curTime;
            xlog->Debug(">>>>>>>>>>> rotate log once <<<<<<<<<<");
            const int ret = std::system(rotateCommand.c_str());
            if (ret != 0) {
                xlog->Warn("logrotate command failed: %s, ret=%d",
                           rotateCommand.c_str(), ret);
            }
        }

        usleep(5000000);
    }

    return 0;
}

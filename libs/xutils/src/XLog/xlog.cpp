/*
 * @Descripttion:
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2019-12-12 16:25:22
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2022-11-25 15:48:53
 */
#include "xlog.h"
#include "global.h"
#include <ctime>
#include <iostream>


/**
 * @brief   Gets the name of the log category from the enum value
 *
 * @param   The enum value of the category
 * @return  The name of the category; returns the word UNKNOWN if not valid.
 */
const char *XLog::TypeToString(const Type &type) {
  switch (type) {
  case XLOG_LEVEL_FATAL:
    return "FATAL ";
  case XLOG_LEVEL_ERROR:
    return "ERROR ";
  case XLOG_LEVEL_WARN:
    return "WARN ";
  case XLOG_LEVEL_INFO:
    return "INFO ";
  case XLOG_LEVEL_DEBUG:
    return "DEBUG ";
  default:
    break;
  }
  return "UNKWN";
}

void XLog::Enable(bool _en, bool relativePath) {
#ifndef MR133_BUILD
  m_en = _en;
#else
  m_en = _en; // for mr133
#endif
  m_relativePath = relativePath;
}

/**
 * @brief   Initialises the file stream
 *
 * @param   fileName The location of the file to create/append to
 * @return  True if the file was successfully initialised; false if already
 * initialised
 */
bool XLog::Initialise(const std::string &fileName) {

  std::string logdir = std::string(LOG_PREFIX);
  m_initialised = false;
  if (!m_en) {
    // std::cout<<"m_en is false"<<std::endl;
    return false;
  }

  if (!m_initialised) {
    m_fileName = logdir + fileName;
    this->m_stream.open(m_fileName.c_str(),
                        std::ios_base::app | std::ios_base::out);
    m_initialised = true;
    m_stream << "\n" << std::endl;
    Info("*******************  %s LOG INITIALISED  *********************",
         fileName.c_str());
    return true;
  } else {
    std::cout << "m_initialised is true" << std::endl;
    return false;
  }

  return false;
}

/**
 * @brief   Finalises the file stream
 *
 * @return  True if the file was successfully finalised; false if not
 * initialised
 */
bool XLog::Finalise() {

  if (m_initialised) {
    Info("*******************  %s LOG FINALISED  *********************",
         m_fileName.c_str());
    m_stream.close();
    m_stream << "\n" << std::endl;
    return true;
  }
  return false;
}

/**
 * @brief  Sets the debugging threshold
 * This is the debugging threshold to use when reporting bugs, useful for having
 * lots of debugging information that sometimes you just want to turn off.
 *
 * @param  type The given debugging threshold to use
 */
void XLog::SetThreshold(const Type &type) { m_threshold = type; }

/**
 * @brief   Writes a Fatal Error to the log
 *
 * @param   message The message to log
 * @return  True if the log was successful
 */
bool XLog::Fatal(const std::string &message) {
  return log(XLOG_LEVEL_FATAL, message);
}

/**
 * @brief   Writes a Fatal Error to the log
 *
 * @param   format The format of the message
 * @param   ... Variable arguments
 * @return  True if the log was successful
 */
bool XLog::Fatal(const char *format, ...) {
  va_list varArgs;
  va_start(varArgs, format);
  bool success = log(XLOG_LEVEL_FATAL, format, varArgs);
  va_end(varArgs);
  return success;
}

/**
 * @brief   Writes an Error to the log
 *
 * @param   message The message to log
 * @return  True if the log was successful
 */
bool XLog::Error(const std::string &message) {
  return log(XLOG_LEVEL_ERROR, message);
}

/**
 * @brief  Writes an Error to the log
 *
 * @param   format The format of the message
 * @param   ... Variable arguments
 * @return  True if the log was successful
 */
bool XLog::Error(const char *format, ...) {
  va_list varArgs;
  va_start(varArgs, format);
  bool success = log(XLOG_LEVEL_ERROR, format, varArgs);
  va_end(varArgs);
  return success;
}

/**
 * @brief   Writes a warning to the log
 *
 * @param   message The message to log
 * @return  True if the log was successful
 */
bool XLog::Warn(const std::string &message) {
  return log(XLOG_LEVEL_WARN, message);
}

/**
 * @brief   Writes a warning to the log
 *
 * @param   format The format of the message
 * @param   ... Variable arguments
 * @return  True if the log was successful
 */
bool XLog::Warn(const char *format, ...) {
  va_list varArgs;
  va_start(varArgs, format);
  bool success = log(XLOG_LEVEL_WARN, format, varArgs);
  va_end(varArgs);
  return success;
}

/**
 * @brief   Writes an information message to the log
 *
 * @param   message The message to log
 * @return  True if the log was successful
 */
bool XLog::Info(const std::string &message) {
  return log(XLOG_LEVEL_INFO, message);
}

/**
 * @brief   Writes an information message to the log
 *
 * @param   format The format of the message
 * @param   ... Variable arguments
 * @return  True if the log was successful
 */
bool XLog::Info(const char *format, ...) {
  va_list varArgs;
  va_start(varArgs, format);
  bool success = log(XLOG_LEVEL_INFO, format, varArgs);
  va_end(varArgs);
  return success;
}

/**
 * @brief   Writes a Debug message to the log
 *
 * @param   message The message to log
 * @return  True if the log was successful
 */
bool XLog::Debug(const std::string &message) {
  return log(XLOG_LEVEL_DEBUG, message);
}

/**
 * @brief   Writes an Debug message to the log
 *
 * @param   format The format of the message
 * @param   ... Variable arguments
 * @return  True if the log was successful
 */
bool XLog::Debug(const char *format, ...) {
  va_list varArgs;
  va_start(varArgs, format);
  bool success = log(XLOG_LEVEL_DEBUG, format, varArgs);
  va_end(varArgs);
  return success;
}

/**
 * @brief   Peeks at the top element of the function stack
 *
 * @return  The top element of the function stack
 */
std::string XLog::Peek() { return m_stack.back(); };

/**
 * @brief   Pushes the function stack with the given message
 *
 * @param   input The message to store in the stack (typically the name of the
 *          function)
 * @return  True if the stack was successfully pushed
 */
bool XLog::Push(const std::string &input) {
  if (!input.empty()) {
    Debug(input + " BEGIN");
    m_stack.push_back(input);
    return true;
  }
  return false;
}

/**
 * @brief   Pops the top element off the stack
 *
 * @return  The message just popped off the stack
 */
std::string XLog::Pop() {

  if (!m_stack.empty()) {
    std::string temp(Peek());
    m_stack.pop_back();
    Debug(temp + " END");
    return temp;
  }
  return std::string();
}

/**
 * @brief  Writes the stack to the log
 */
void XLog::PrintStackTrace() {
  std::string temp = "---Stack Trace---\n";

  for (std::vector<std::string>::reverse_iterator i = m_stack.rbegin();
       i != m_stack.rend(); ++i) {
    temp += "| " + *i + "\n";
  }

  temp += "-----------------";
  this->write(temp.c_str());
}
/**
 * @brief  Constructor
 */
XLog::XLog()
    : m_stream(), m_threshold(XLOG_LEVEL_INFO), m_initialised(false),
      m_fileName(), m_stack(), m_en(false), m_relativePath(), m_en_cout(true) {}

/**
 * @brief  Copy constructor
 * Kept private in order to preserve singleton

XLog::XLog(const XLog&) {
}

 */
/**
 * @brief  Destructor
 * Logs the shut down then closes the file stream
 */
XLog::~XLog() { Finalise(); }

void XLog::EnableCout(bool _en) { m_en_cout = _en; }

/**
 * @brief  Writes the specified message to the console and the log file
 *
 * @param  format The format of the message
 * @param  ... Variable arguments
 */
void XLog::write(const char *format, ...) {

  if (!m_en)
    return;

  char buffer[512];

  va_list varArgs;
  va_start(varArgs, format);
  vsnprintf(buffer, sizeof(buffer), format, varArgs);
  va_end(varArgs);
  if (m_initialised) {
    // std::cout << buffer << std::endl;
    m_stream << buffer << std::endl;
  }
  if (m_en_cout)
    std::cout << buffer << std::endl;
}
void XLog::writeV2(const std::string &ts, const std::string &type,
                   const std::string &msg) {
  if (m_initialised) {
    // std::cout << buffer << std::endl;
    m_stream << ts << type << msg << std::endl;
  }
}

void XLog::WriteRaw(const char *format, ...) {
  if (!m_en)
    return;
  char buffer[512];

  va_list varArgs;
  va_start(varArgs, format);
  vsnprintf(buffer, sizeof(buffer), format, varArgs);
  va_end(varArgs);
  if (m_initialised) {
    // std::cout << buffer;
    m_stream << buffer;
  }
  if (m_en_cout)
    std::cout << buffer;
}

void XLog::Flush() {
  if (m_initialised) {
    // std::cout << std::endl;
    m_stream << std::endl;
  }

  if (m_en_cout)
    std::cout << std::endl;
}

/**
 * @brief   Logs the specified message with a timestamp and category prefix
 * The constant TIMESTAMP_BUFFER_SIZE was calculated as the maximum number of
 * characters required for the timestamp "[HH:MM:SS MM/DD/YY] ".
 *
 * @param   type The category of message to write based on the enum Log::Type
 * @param   message The message to be sent
 * @return  True if the log was successful
 */
bool XLog::log(const Type &type, const std::string &message) {
  if (!m_en)
    return false;

  std::string ts = stamp() + " ";
  if (type <= m_threshold) {

    this->writeV2(ts, TypeToString(type), message);
    // this->write( "[%s] %s - %s", ts.c_str(), TypeToString( type ),
    // message.c_str() );
    return true;
  }
  return false;
}

/**
 * @brief   Logs the specified message with a timestamp and category prefix
 * The constant TIMESTAMP_BUFFER_SIZE was calculated as the maximum number of
 * characters required for the timestamp "[HH:MM:SS MM/DD/YY] ".
 *
 * @param   type The category of message to write based on the enum Log::Type
 * @param   format The format of the message
 * @param   ... Variable arguments
 * @return  True if the log was successful
 */
bool XLog::log(const Type &type, const char *format, va_list varArgs) {
  char buffer[512];
  vsnprintf(buffer, sizeof(buffer), format, varArgs);
  return log(type, buffer);
}

/**
 * @brief  Copy operator
 * Kept private in order to preserve singleton
 */
XLog &XLog::operator=(const XLog &) { return *this; }

/* Jephy for millisecond ts */
std::string XLog::date_time(std::time_t posix) {
  char buf[20]; // big enough for 2015-07-08 10:06:51\0
  std::tm tp = *std::localtime(&posix);
  return {buf, std::strftime(buf, sizeof(buf), "%F %T", &tp)};
}

std::string XLog::stamp() {
  using namespace std;
  using namespace std::chrono;
  using std::setw;
  // get absolute wall time
  auto now = system_clock::now();
  now += std::chrono::seconds(8 * 3600); // UTC+8h
  // find the number of milliseconds
  auto us = duration_cast<microseconds>(now.time_since_epoch()) % 1000000;

  // build output string
  std::ostringstream oss;
  oss.fill('0');

  // convert absolute time to time_t seconds
  // and convert to "date time"
  oss << date_time(system_clock::to_time_t(now));
  oss << '.' << setw(6) << us.count();

  return oss.str();
}

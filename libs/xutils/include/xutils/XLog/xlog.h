/*
 * @Descripttion:
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2019-12-12 16:25:25
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2022-11-25 16:51:34
 */
#ifndef __XLOG_H__
#define __XLOG_H__

#include <fstream>
#include <string>
#include <vector>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdarg.h>


typedef enum Type {
  XLOG_LEVEL_FATAL,
  XLOG_LEVEL_ERROR,
  XLOG_LEVEL_WARN,
  XLOG_LEVEL_INFO,
  XLOG_LEVEL_DEBUG,
} XLOG_LEVEL;

class XLog {
public:
  XLog();
  XLog(bool en) {
    m_en = en;
    m_relativePath = true;
    m_en_cout = true;
    m_fileName.clear();
    m_stack.clear();
    m_stream.clear();
    m_initialised = false;
  }
  // XLog(const XLog&);
  ~XLog();

  const char *TypeToString(const Type &type);

  void Enable(bool _en, bool relativePath = true);
  bool Initialise(const std::string &fileName);
  bool Finalise();
  void EnableCout(bool _en = false);

  void SetThreshold(const Type &type);

  bool Fatal(const std::string &message);
  bool Fatal(const char *format, ...);

  bool Error(const std::string &message);
  bool Error(const char *format, ...);

  bool Warn(const std::string &message);
  bool Warn(const char *format, ...);

  bool Info(const std::string &message);
  bool Info(const char *format, ...);

  bool Debug(const std::string &message);
  bool Debug(const char *format, ...);

  std::string Peek();
  bool Push(const std::string &input);
  std::string Pop();
  void PrintStackTrace();
  std::ofstream m_stream;

  void WriteRaw(const char *format, ...);
  void Flush();

private:
  Type m_threshold;
  bool m_initialised;
  std::string m_fileName;
  std::vector<std::string> m_stack;

  bool m_en;
  bool m_relativePath;
  bool m_en_cout;

  void write(const char *format, ...);
  void writeV2(const std::string &ts, const std::string &type,
               const std::string &msg);

  bool log(const Type &type, const std::string &message);
  bool log(const Type &type, const char *format, va_list varArgs);

  XLog &operator=(const XLog &);
  // use strftime to format time_t into a "date time"
  std::string date_time(std::time_t posix);
  std::string stamp();
};

#endif

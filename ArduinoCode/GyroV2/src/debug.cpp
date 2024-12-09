#include "debug.h"

#ifdef PIO_UNIT_TESTING
#include <cstring>
inline const char *trim_start(const char *str)
{
  while (memchr(" \t\n\r", *str, 4))
    ++str;
  return str;
}
inline const char *trim_end(const char *end)
{
  while (memchr(" \t\n\r", end[-1], 4))
    --end;
  return end;
}
inline std::string trim(const char *buffer, int len) // trim a buffer (input?)
{
  return std::string(trim_start(buffer), trim_end(buffer + len));
}
inline void trim_inplace(std::string &str)
{
  str.assign(trim_start(str.c_str()),
             trim_end(str.c_str() + str.length()));
}

String::String(const char *str)
{
  stdString = str;
}
String::String()
{
  stdString = "";
}

double String::toDouble()
{
  // printf("String::toDouble %s=>%f\n", stdString.c_str(), std::stod(stdString));
  return std::stod(stdString);
}
void String::trim()
{
  trim_inplace(stdString);
}
String String::substring(int start, int end)
{
  return String(stdString.substr(start, end - start).c_str());
}
int String::indexOf(char c)
{
  size_t pos = stdString.find(c);
  if (pos == std::string::npos)
    return -1;
  return (int)pos;
}

#else
#include <Arduino.h>
#endif

#define NANOPRINTF_IMPLEMENTATION
#include "nanoprintf.h"

void debug_printf(const char *fmt, ...)
{
  char buffer[80];
  va_list args;
  va_start(args, fmt);
  npf_vsnprintf(buffer, sizeof(buffer), fmt, args);
  buffer[sizeof(buffer)-1]  = 0;
  va_end(args);
#ifdef PIO_UNIT_TESTING
  printf("%s", buffer);
#else
  if (strlen(buffer) > sizeof(buffer)-5)
  {
    Serial.println("Warning: debug_printf buffer overflow");
  }
  Serial.print(buffer);
  delay(100); // otherwise serial breaks arduino...
#endif
}

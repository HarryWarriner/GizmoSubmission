#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef PIO_UNIT_TESTING
#include <string>
class String
{
  std::string stdString;

public:
  String(const char *str);
  String();

  double toDouble();
  void trim();
  String substring(int start, int end = -1);
  int indexOf(char c);
  const char *c_str() { return stdString.c_str(); }
  size_t length() { return stdString.length(); }
};

#endif

void debug_printf(const char *fmt, ...);

#endif
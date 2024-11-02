#include <Arduino.h>

#ifndef LOGGER_HPP
#define LOGGER_HPP

class Log{
public:
  static void print(char c);
  static void print(char* msg);
  static void print(String msg);
  static void print(float num);
  static void print(int num);
  
  static void println(void);
  static void println(const char* msg);
};

#endif

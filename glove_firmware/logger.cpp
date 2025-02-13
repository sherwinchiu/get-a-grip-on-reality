#include "logger.hpp"

#include <Arduino.h>
#include "bluetooth.hpp"

void Log::print(char c){
  // transmitMessage(&c, (size_t)1);
}

void Log::print(const char* msg){
  size_t len = 0;
  while (*(msg+len)) len++;
  transmitMessage(msg, len);
}

void Log::print(String msg){
  transmitMessage((char*) msg.c_str(), msg.length());
}

void Log::print(float num){
  String s = String(num);
  transmitMessage((char*)s.c_str(), s.length());  
}

void Log::print(int num){
  String s = String(num);
  transmitMessage((char*)s.c_str(), s.length());  
}


void Log::println(void){
  print('\r');
  print('\n');
}

void Log::println(const char* msg){
  print((char*) msg);
  print('\r');
  print('\n');
}
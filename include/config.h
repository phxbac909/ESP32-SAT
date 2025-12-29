// Trong config.h
// #define ENABLE_SERIAL_DEBUG  // Comment dòng này để tắt hoàn toàn

#ifdef ENABLE_SERIAL_DEBUG
  #define SERIAL_BEGIN(x) Serial.begin(x)
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define SERIAL_BEGIN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

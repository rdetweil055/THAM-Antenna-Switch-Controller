// Debug utilities

#if DEBUGLEVEL >= 0
#define DEBUGPRINT0(x) Serial.print(x)
#define DEBUGPRINTF0(...) Serial.printf(__VA_ARGS__)
#define DEBUGPRINTLN0(x) Serial.println(x)
#else
#define DEBUGPRINT0(x)
#define DEBUGPRINTF0(...)
#define DEBUGPRINTLN0(x)
#endif

#if DEBUGLEVEL >= 1
#define DEBUGPRINT1(x) Serial.print(x)
#define DEBUGPRINTF1(...) Serial.printf(__VA_ARGS__)
#define DEBUGPRINTLN1(x) Serial.println(x)
#else
#define DEBUGPRINT1(x)
#define DEBUGPRINTF1(...)
#define DEBUGPRINTLN1(x)
#endif

#if DEBUGLEVEL >= 2
#define DEBUGPRINT2(x) Serial.print(x)
#define DEBUGPRINTF2(...) Serial.printf(__VA_ARGS__)
#define DEBUGPRINTLN2(x) Serial.println(x)
#else
#define DEBUGPRINT2(x)
#define DEBUGPRINTF2(...)
#define DEBUGPRINTLN2(x)
#endif

#if DEBUGLEVEL >= 3
#define DEBUGPRINT3(x) Serial.print(x)
#define DEBUGPRINTF3(...) Serial.printf(__VA_ARGS__)
#define DEBUGPRINTLN3(x) Serial.println(x)
#else
#define DEBUGPRINT3(x)
#define DEBUGPRINTF3(...)
#define DEBUGPRINTLN3(x)
#endif

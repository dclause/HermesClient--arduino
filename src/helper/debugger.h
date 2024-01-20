#ifndef HERMESCLIENT_DEBUGGER_H
#define HERMESCLIENT_DEBUGGER_H

#include <Arduino.h>

// Active debug: use 1, otherwise 0.
#define ACTIVATE_DEBUG 1
#define SERIAL_DEBUGGER Serial1

#if ACTIVATE_DEBUG && defined(SERIAL_DEBUGGER)

#if !defined(USE_SERIAL_PROTOCOL)
#include <HardwareSerial.h>
#endif

#define TRACE(X) SERIAL_DEBUGGER.println("# " + String(X))
#endif

// Defines the TRACE command.
#if ACTIVATE_DEBUG && !defined(TRACE)
#define TRACE(X) IO::debug(X)
#elif !defined(TRACE)
#define TRACE(X)
#endif

#endif // HERMESCLIENT_DEBUGGER_H

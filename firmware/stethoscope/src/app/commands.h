// Text command interpreter shared by the USB serial link and BLE.
// Type "help" in the Serial Monitor (115200 baud, newline line ending).
#pragma once

#include <Print.h>
#include <stddef.h>

namespace commands {

// Executes one command line (the buffer is modified). Replies go to out.
void execute(char* line, Print& out);

// Writes the status JSON object (no trailing newline). Returns its length.
size_t statusJson(char* buf, size_t cap);

// One-line human-readable status (used by "stream log").
size_t statusLine(char* buf, size_t cap);

void printHelp(Print& out);

// Set by the "reboot" command; the main loop performs the restart.
bool rebootRequested();

}  // namespace commands

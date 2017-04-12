#pragma once

struct _LogEntry; typedef _LogEntry LogEntry;
struct _Controller; typedef _Controller Controller;

float policyTurntable(const LogEntry& state);
float policyWheel(const LogEntry& state);
void setPolicy(const Controller& new_controller);

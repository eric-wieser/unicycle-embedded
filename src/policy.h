#pragma once

struct _LogEntry; typedef _LogEntry LogEntry;
struct _SetController; typedef _SetController SetController;

float policyTurntable(const LogEntry& state);
float policyWheel(const LogEntry& state);
void setPolicy(const SetController& new_controller);

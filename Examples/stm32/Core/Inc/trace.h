#pragma once

#include "main.h"

enum task_tag_e { TASK_TAG_LED = 10, TASK_TAG_TOF };

void TaskSwitched(uint8_t direction, int taskTag);
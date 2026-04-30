#include "trace.h"

#define DIRECTION_IN  1U
#define DIRECTION_OUT 0U

void TaskSwitched(uint8_t direction, int taskTag)
{
    switch (direction) {
    case DIRECTION_IN:
        switch (taskTag) {
        case TASK_TAG_TOF:
            HAL_GPIO_WritePin(HOOK_TASK1_GPIO_Port, HOOK_TASK1_Pin, GPIO_PIN_SET);
            break;

        default:
            break;
        }
        break;

    case DIRECTION_OUT:
        switch (taskTag) {
        case TASK_TAG_TOF:
            HAL_GPIO_WritePin(HOOK_TASK1_GPIO_Port, HOOK_TASK1_Pin, GPIO_PIN_RESET);
            break;

        default:
            break;
        }
        break;
    }
}
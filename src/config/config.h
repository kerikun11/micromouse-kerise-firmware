/**
 * @file config.h
 * @brief Configuration for MicroMouse KERISE
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2022-03-24
 * @copyright Copyright 2022 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <soc/soc.h>

/* KERISE Select */
#define KERISE_SELECT 6

/* Drive Mode */
#define MACHINE_DRIVE_AUTO_ENABLED 0

/* Log Target */
#define APP_LOG_MEM_MODE_ENABLED 0

/* Log Level */
#define APP_LOG_LEVEL 3
#define MR_LOG_LEVEL 3
#define MA_LOG_LEVEL 3

/* Task Priority */
#define TASK_PRIORITY_REFLECTOR 20

#define TASK_PRIORITY_ENCODER 10
#define TASK_PRIORITY_IMU 9
#define TASK_PRIORITY_SPEED_CONTROLLER 8
#define TASK_PRIORITY_WALL_DETECTOR 7

#define TASK_PRIORITY_MOVE_ACTION 3
#define TASK_PRIORITY_LED 1
#define TASK_PRIORITY_TOF 1
#define TASK_PRIORITY_BUTTON 1
#define TASK_PRIORITY_BUZZER 1
#define TASK_PRIORITY_DRIVE 2
#define TASK_PRIORITY_PRINT 1

/* Core ID */
/* Application CPU */
#define TASK_CORE_ID_REFLECTOR APP_CPU_NUM
/* Processor CPU */
#define TASK_CORE_ID_ENCODER PRO_CPU_NUM
#define TASK_CORE_ID_IMU PRO_CPU_NUM
#define TASK_CORE_ID_SPEED_CONTROLLER PRO_CPU_NUM
#define TASK_CORE_ID_WALL_DETECTOR PRO_CPU_NUM
/* No Affinity */
#define TASK_CORE_ID_MOVE_ACTION tskNO_AFFINITY
#define TASK_CORE_ID_LED tskNO_AFFINITY
#define TASK_CORE_ID_TOF tskNO_AFFINITY
#define TASK_CORE_ID_BUTTON tskNO_AFFINITY
#define TASK_CORE_ID_BUZZER tskNO_AFFINITY
#define TASK_CORE_ID_DRIVE tskNO_AFFINITY
#define TASK_CORE_ID_PRINT tskNO_AFFINITY

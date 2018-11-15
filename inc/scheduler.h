#ifndef _SCHEDULER_
#define _SCHEDULER_


#define TASKS_NUM			2
#define TASK_TIMER_PERIOD	5

typedef struct _state {
   int state0;                 // Current state
   int state1;
} state_t;

typedef struct _task {
   unsigned long period;      // Rate at which the task should tick
   unsigned long elapsedTime; // Time since task's last tick in secs
   state_t (*TickFct)(state_t);       // Function to call for task's tick
   state_t state;
} task_t;

void initTask(int task, int period, state_t (*TickFct)(state_t), int state0, int state1);
int schedule();

#endif


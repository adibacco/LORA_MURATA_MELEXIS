#include <stdio.h>
#include "scheduler.h"


task_t tasks[TASKS_NUM] = { { 0, 0, NULL, { 0, 0 } }, {0, 0, NULL, { 0, 0 } } };


void initTask(int task, int period, state_t (*TickFct)(state_t), int state0, int state1) {
	   tasks[task].period      = period;
	   tasks[task].elapsedTime = tasks[task].period;
	   tasks[task].TickFct     = TickFct;
	   state_t initState = { state0, state1 };
	   tasks[task].state       = initState;
}

int schedule() 
{
	  // Heart of the scheduler code
	  for (int i = 0; i < TASKS_NUM; ++i) {
		 if (tasks[i].period > 0) {
			 if (tasks[i].elapsedTime >= tasks[i].period) {
				// Ready
				tasks[i].state = tasks[i].TickFct(tasks[i].state);
				tasks[i].elapsedTime = 0;
			 }
			 tasks[i].elapsedTime += TASK_TIMER_PERIOD;
		 }
	  }

	  return 0;
}



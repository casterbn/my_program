#ifndef __TIMER_H
#define __TIMER_H

int   dacqTimerInit(int period);
int   dacqTimerStart();
void  timerElapsedCallback();
void  ConfigureTimerFor1PPSCapture();
int   ConfigureLinkedTimers(int freq);
void  ConfigureReferenceTimer();
void  StartLinkedTimers();
int   GetSensToPpsDelay();
int   GetPpsToDrdyDelay();
int   WaitForNewTick();
int   TimeNow();
int   IsOverrun();

#endif

#ifndef _wiced_H_
#define _wiced_H_

#define MILLISECOND   1
#define SECOND        (1000 * MILLISECOND)
#define MINUTE        (60 * SECOND)
#define HOUR          (60 * MINUTE)

struct Task { 
  unsigned long   time;
  unsigned long   interval;
  void          (*execute) (Task* task);
  int             status;
} ;

/* tasks header functions. */
void updateNeo(Task* task);
void updateButtons(Task* task);
void updateOled(Task* task);
void updateLed(Task* task);
void updateMqtt(Task* task);
#endif


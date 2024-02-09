#ifndef DEFINE_H
#define DEFINE_H

#define MSBFIRST  0
#define LSBFIRST  1

#define HIGH      1
#define LOW       0

#define ENABLE    1
#define DISABLE   0

#define ON        1
#define OFF       0

#define delayNonBlocking(ms) vTaskDelay(pdMS_TO_TICKS(ms))


#endif // DEFINE_H
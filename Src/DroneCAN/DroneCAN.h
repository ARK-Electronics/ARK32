#pragma once

#include <stdbool.h>

#if DRONECAN_SUPPORT
void DroneCAN_Init(void);
void DroneCAN_update();
/* True while esc.RawCommand is live (~250 ms window). Wire inputs defer. */
bool DroneCAN_active(void);

#endif // DRONECAN_SUPPORT

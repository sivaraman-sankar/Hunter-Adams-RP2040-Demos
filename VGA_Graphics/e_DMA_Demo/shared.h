#ifndef SHARED_H
#define SHARED_H

#include "pt_cornell_rp2040_v1_3.h"

// Define message types
typedef enum
{
    MSG_NOTE_CHANGE = 1,
    MSG_BACKING_TRACK_CHANGE = 2,
    MSG_KEY_SIGNATURE_CHANGE = 3,
    MSG_PLAYBACK_START = 4,
    MSG_PLAYBACK_STOP = 5,
    MSG_PLAYBACK_PAUSE = 6,
    MSG_PLAYBACK_RESUME = 7,
    MSG_DRUMS_CHANGE = 8,
    MSG_INSTRUMENT_CHANGE = 9,
} MessageType;

// Define a message format
typedef struct
{
    MessageType type;
    char payload;
} StateMessage;

#endif
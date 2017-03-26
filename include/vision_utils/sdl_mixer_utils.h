#ifndef SDL_MIXER_UTILS_H
#define SDL_MIXER_UTILS_H
#include <SDL2/SDL_mixer.h>

inline void Mix_FreeChunk_safe(Mix_Chunk * & chunk) {
  if (chunk)
    Mix_FreeChunk(chunk);
  chunk = NULL;
}

#endif // SDL_MIXER_UTILS_H

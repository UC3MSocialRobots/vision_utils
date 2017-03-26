#ifndef SDL_TTF_UTILS_H
#define SDL_TTF_UTILS_H
#include <vision_utils/sdl_utils.h>
#include <SDL2/SDL_ttf.h>
#include <string>

bool text2texture(Texture & texture,
                  SDL_Renderer* renderer,
                  TTF_Font *font,
                  std::string textureText,
                  Uint8 r, Uint8 g, Uint8 b, Uint8 a = 255,
                  Uint8 bgr = 0, Uint8 bgg = 0, Uint8 bgb = 0, Uint8 bga = 0) {
  //Get rid of preexisting texture
  texture.free();
  //Render text surface
  SDL_Surface* surface;
  SDL_Color fg = {r, g, b, a};
  if (bga <= 0)
    surface = TTF_RenderText_Solid( font, textureText.c_str(), fg );
  else {
    SDL_Color bg = {bgr, bgg, bgb, bga};
    surface = TTF_RenderText_Shaded( font, textureText.c_str(), fg, bg );
  }
  if( surface == NULL ) {
    printf( "Unable to render text surface! SDL_ttf Error: %s\n", TTF_GetError() );
    return false;
  }
  //Create texture from surface pixels
  return texture.from_surface(surface, renderer);
}

#endif // SDL_TTF_UTILS_H

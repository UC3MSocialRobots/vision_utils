#ifndef SDL_GFX_UTILS_H
#define SDL_GFX_UTILS_H
#include <vision_utils/sdl_utils.h>
#include <SDL2/SDL2_gfxPrimitives.h>

void render_point(SDL_Renderer* renderer, const Point2d & p, int thickness,
                  Uint8 r, Uint8 g, Uint8 b, Uint8 a = 255) {
  Uint8 r0, g0, b0, a0; // get original colors of the renderer
  SDL_GetRenderDrawColor( renderer, &r0, &g0, &b0, &a0 );
  SDL_SetRenderDrawColor( renderer, r, g, b, a );
  SDL_Rect fillRect = { (int) (p.x-thickness/2),
                        (int) (p.y-thickness/2),
                        thickness, thickness };
  SDL_RenderFillRect( renderer, &fillRect );
  SDL_SetRenderDrawColor( renderer, r0, g0, b0, a0 );
}

////////////////////////////////////////////////////////////////////////////////

//! \caution CPU costs are much higher if \arg thickness > 1
bool render_line(SDL_Renderer* renderer, Point2d pt1, Point2d pt2,
                 Uint8 r, Uint8 g, Uint8 b, Uint8 a = 255, int thickness=1) {
  Uint8 r0, g0, b0, a0; // get original colors of the renderer
  SDL_GetRenderDrawColor( renderer, &r0, &g0, &b0, &a0 );
  // https://stackoverflow.com/questions/21560384/how-to-specify-width-or-point-size-in-sdl-2-0-draw-points-lines-or-rect
  if (thickness == 1) {
    SDL_SetRenderDrawColor( renderer, r, g, b, a );
    if (SDL_RenderDrawLine(renderer, pt1.x, pt1.y, pt2.x, pt2.y) != 0) {
      printf("SDL_RenderDrawLine() returned an error!\n");
      return false;
    }
  }
  // http://www.ferzkopp.net/Software/SDL2_gfx/Docs/html/_s_d_l2__gfx_primitives_8h.html#a247136a562abec2649718d38f5819b44
  else if (thickLineRGBA(renderer, pt1.x, pt1.y, pt2.x, pt2.y, thickness, r, g, b, a) != 0) {
    printf("thickLineRGBA() returned an error!\n");
    return false;
  }
  SDL_SetRenderDrawColor( renderer, r0, g0, b0, a0 );
  return true;
}

////////////////////////////////////////////////////////////////////////////////

//! \caution CPU costs are much higher if \arg thickness > 1
bool render_polygon(SDL_Renderer* renderer, const std::vector<Point2d> & poly,
                    Uint8 r, Uint8 g, Uint8 b, Uint8 a = 255, int thickness=1) {
  unsigned int npts = poly.size();
  if (npts == 0)
    return true;
  bool ok = true;
  for (unsigned int i = 0; i < npts-1; ++i)
    ok = ok && render_line(renderer, poly[i], poly[i+1], r, g, b, a, thickness);
  ok = ok && render_line(renderer, poly[0], poly[npts-1], r, g, b, a, thickness);
  return ok;
}

////////////////////////////////////////////////////////////////////////////////

//! \caution CPU costs are much higher if \arg thickness > 1
bool render_rect(SDL_Renderer* renderer, const SDL_Rect & rect,
                 Uint8 r, Uint8 g, Uint8 b, Uint8 a = 255, int thickness=1) {
  std::vector<Point2d> pts;
  pts.push_back(Point2d(rect.x, rect.y));
  pts.push_back(Point2d(rect.x, rect.y+rect.h));
  pts.push_back(Point2d(rect.x+rect.w, rect.y+rect.h));
  pts.push_back(Point2d(rect.x+rect.w, rect.y));
  return render_polygon(renderer, pts, r, g, b, a, thickness);
}

////////////////////////////////////////////////////////////////////////////////

//! \caution CPU costs are much higher if \arg thickness > 1
inline bool render_arrow
(SDL_Renderer* renderer, const Point2d & pt1, const Point2d & pt2,
 Uint8 r, Uint8 g, Uint8 b, Uint8 a = 255, int thickness=1) {
  // draw the body of the arrow
  if (!render_line(renderer, pt1, pt2, r, g, b, a, thickness))
    return false;
  // compute parameters of the arrow
  double side_strokes_length = hypot(pt2.y - pt1.y, pt2.x - pt1.x) / 3;
  double arrow_orien = atan2(pt2.y - pt1.y, pt2.x - pt1.x);
  Point2d pt_end;
  // draw first side stroke
  pt_end.x  = pt2.x + side_strokes_length * cos(arrow_orien + M_PI + M_PI / 6);
  pt_end.y  = pt2.y + side_strokes_length * sin(arrow_orien + M_PI + M_PI / 6);
  if (!render_line(renderer, pt2, pt_end, r, g, b, a, thickness))
    return false;
  // draw second side stroke
  pt_end.x  = pt2.x + side_strokes_length * cos(arrow_orien + M_PI - M_PI / 6);
  pt_end.y  = pt2.y + side_strokes_length * sin(arrow_orien + M_PI - M_PI / 6);
  if (!render_line(renderer, pt2, pt_end, r, g, b, a, thickness))
    return false;
  return true;
} // end draw_arrow();

#endif // SDL_GFX_UTILS_H

/*!
  \file        sdl_utis.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/4/9

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file
 */
#ifndef SDL_UTILS_H
#define SDL_UTILS_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_mixer.h>
#include "timer.h"
#include <sstream>
#include <vector>
#include <sensor_msgs/Image.h>

// uncomment to render bounding boxes and to display function calls in terminals
//#define DEBUG

#ifdef DEBUG
#  define DEBUG_PRINT(...)   printf( __FILE__ ":" __VA_ARGS__)
#else
#  define DEBUG_PRINT(...)   {}
#endif

#define RAD2DEG     57.2957795130823208768  //!< to convert radians to degrees
#define DEG2RAD     0.01745329251994329577  //!< to convert degrees to radians

#define ROTATE_COSSIN_X(x, y, cos_angle, sin_angle) \
  ((cos_angle) * (x) - (sin_angle) * (y))
#define ROTATE_COSSIN_Y(x, y, cos_angle, sin_angle) \
  ((sin_angle) * (x) + (cos_angle) * (y))
#define ROTATE_ANGLE_X(x, y, angle) \
  (ROTATE_COSSIN_X(x, y, cos(angle), sin(angle)) )
#define ROTATE_ANGLE_Y(x, y, angle) \
  (ROTATE_COSSIN_Y(x, y, cos(angle), sin(angle)) )

SDL_Color SDL_Color_ctor(Uint8 r = 0, Uint8 g = 0, Uint8 b = 0, Uint8 a = 255)  {
  SDL_Color ans;
  ans.r = r;
  ans.g = g;
  ans.b = b;
  ans.a = a;
  return ans;
}

SDL_Rect SDL_Rect_ctor(int x, int y, int w, int h)  {
  SDL_Rect ans;
  ans.x = x;
  ans.y = y;
  ans.w = w;
  ans.h = h;
  return ans;
}

bool operator ==(const SDL_Color &A, const SDL_Color &B) {
  return A.r == B.r && A.g == B.g && A.b == B.b && A.a == B.a;
}


/*! a generic templated class for 2D points.
  It contains a x and a y field so as to be compatible
  with OpenCV Point2d*
*/
template<class _Type>
class Point2 {
public:
  typedef _Type value_type; //!< an alias for the data type (compat with OpenCV)

  //! a constructor without arguments
  Point2() : x(0), y(0) {}

  //! a constructor
  Point2(const _Type & x_, const _Type & y_) :
    x(x_), y(y_) {}

  //! the == operator, for some std algorithms
  bool operator== (const Point2<_Type>& b) const {
    return (x == b.x) && (y == b.y);
  }

  double norm() const { return hypot(x, y); }
  void renorm(const double & newnorm) {
    double currn = norm();
    if (fabs(currn) < 1E-6)
      return;
    x *= newnorm / currn;
    y *= newnorm / currn;
  }

  //! the + operator, that adds field by field
  Point2<_Type> operator + (const Point2<_Type>& B) const {
    return Point2<_Type>(x + B.x, y + B.y);
  }

  //! the - operator, that substracts field by field
  Point2<_Type> operator - (const Point2<_Type>& B) const {
    return Point2<_Type>(x - B.x, y - B.y);
  }

  //! the * operator, that multiplies field by field
  void operator *= (const double & alpha) {
    x = alpha * x;
    y = alpha * y;
  }

  //! the * operator, that multiplies field by field
  void operator += (const Point2<_Type>& B) {
    x += B.x;
    y += B.y;
  }

  //! the dot operator
  inline _Type dot(const Point2<_Type>& B) const {
    return x * B.x + y * B.y;
  }

  //! define the output to a stream
  friend std::ostream & operator << (std::ostream & stream,
                                     const Point2<_Type> & P) {
    stream << P.to_string();
    return stream;
  }

  std::string to_string() const {
    std::ostringstream out;
    out << '[' << x << ", " << y << ']';
    return out.str();
  }

  SDL_Point to_sdl() const {
    SDL_Point ans;
    ans.x = x;
    ans.y = y;
    return ans;
  }
  //implicit conversion
  operator SDL_Point() const { return to_sdl(); }

  template<class _Type2>
  Point2<_Type2> recast() const {
    return Point2<_Type2>(x, y);
  }

  _Type x; //!< the first data field
  _Type y; //!< the second data field
}; // end Point2

//! the * operator, that multiplies field by field
template<class _Type>
static Point2<_Type> operator * (double alpha, const Point2<_Type> & p) {
  return Point2<_Type>(alpha * p.x, alpha * p.y);
}

typedef Point2<int> Point2i;
typedef Point2<double> Point2d;

////////////////////////////////////////////////////////////////////////////////

inline Point2d rotate(const Point2d &pt, double angle) {
  double cosa = cos(angle), sina = sin(angle);
  Point2d ans;
  ans.x = ROTATE_COSSIN_X(pt.x, pt.y, cosa, sina);
  ans.y = ROTATE_COSSIN_Y(pt.x, pt.y, cosa, sina);
  return ans;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief   detect if a point is inside a polygon - return true or false
 *  http://en.wikipedia.org/wiki/Point_in_polygon#Winding_number_algorithm
     *
 * \param   p the point
 * \param   poly the polygon
 * \return  true if the point is in the polygon
 */
static inline bool point_inside_polygon(const Point2d & p,
                                        const std::vector<Point2d> & poly) {
  /*
     * algo from http://www.visibone.com/inpoly/
     */
  Point2d p_old, p_new, p1, p2;
  bool inside = false;
  int npoints = poly.size();
  if (npoints < 3) {
    return false;
  }
  p_old = poly[npoints-1];
  for (int i=0 ; i < npoints ; i++) {
    p_new = poly[i];
    if (p_new.x > p_old.x) {
      p1 = p_old;
      p2 = p_new;
    }
    else {
      p1 = p_new;
      p2 = p_old;
    }
    if ((p_new.x < p.x) == (p.x <= p_old.x)          /* edge "open" at one end */
        && 1.f * (p.y-p1.y) * (p2.x-p1.x) < 1.f * (p2.y-p1.y) * (p.x-p1.x)) {
      inside = !inside;
    }
    p_old.x = p_new.x;
    p_old.y = p_new.y;
  } // end loop i
  return(inside);
}

////////////////////////////////////////////////////////////////////////////////

/*! https://stackoverflow.com/questions/10962379/how-to-check-intersection-between-2-rotated-rectangles
 * Checks if the two polygons are intersecting.
 * \param reverse_already_checked
 *  the function is internally made of 2 subcalls.
 *  DO NOT set this parameter to true.
 */
bool IsPolygonsIntersecting(const std::vector<Point2d> & A,
                            const std::vector<Point2d> & B,
                            bool reverse_already_checked = false) {
  unsigned int sA =A.size(), sB = B.size();
  for (unsigned int i1 = 0; i1 <sA; i1++) {
    unsigned int i2 = (i1 + 1) %sA;
    Point2d p1 = A[i1], p2 = A[i2];
    Point2d normal(p2.y - p1.y, p1.x - p2.x);

    double UNDEF = 1E9;
    double minA = UNDEF, maxA = UNDEF;
    for (unsigned int i = 0; i <sA; ++i) {
      Point2d p = A[i];
      double projected = normal.x * p.x + normal.y * p.y;
      if (minA == UNDEF || projected < minA)
        minA = projected;
      if (maxA == UNDEF || projected > maxA)
        maxA = projected;
    }

    double minB = UNDEF, maxB = UNDEF;
    for (unsigned int i = 0; i < sB; ++i) {
      Point2d p = B[i];
      double projected = normal.x * p.x + normal.y * p.y;
      if (minB == UNDEF || projected < minB)
        minB = projected;
      if (maxB == UNDEF || projected > maxB)
        maxB = projected;
    }

    if (maxA < minB || maxB < minA)
      return false;
  }
  if (!reverse_already_checked)
    return true;
  return IsPolygonsIntersecting(B, A, false);
}

////////////////////////////////////////////////////////////////////////////////

// https://www.libsdl.org/release/SDL-1.2.15/docs/html/guidevideo.html
Uint32 getpixel(SDL_Surface *surface, int x, int y) {
  int bpp = surface->format->BytesPerPixel;
  /* Here p is the address to the pixel we want to retrieve */
  Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;
  switch(bpp) {
    case 1:
      return *p;
      break;

    case 2:
      return *(Uint16 *)p;
      break;

    case 3:
      if(SDL_BYTEORDER == SDL_BIG_ENDIAN)
        return p[0] << 16 | p[1] << 8 | p[2];
      else
        return p[0] | p[1] << 8 | p[2] << 16;
      break;

    case 4:
      return *(Uint32 *)p;
      break;

    default:
      return 0;       /* shouldn't happen, but avoids warnings */
  }
}

// https://www.libsdl.org/release/SDL-1.2.15/docs/html/guidevideo.html
void putpixel(SDL_Surface *surface, int x, int y, Uint32 pixel) {
  int bpp = surface->format->BytesPerPixel;
  /* Here p is the address to the pixel we want to set */
  Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

  switch(bpp) {
    case 1:
      *p = pixel;
      break;

    case 2:
      *(Uint16 *)p = pixel;
      break;

    case 3:
      if(SDL_BYTEORDER == SDL_BIG_ENDIAN) {
        p[0] = (pixel >> 16) & 0xff;
        p[1] = (pixel >> 8) & 0xff;
        p[2] = pixel & 0xff;
      } else {
        p[0] = pixel & 0xff;
        p[1] = (pixel >> 8) & 0xff;
        p[2] = (pixel >> 16) & 0xff;
      }
      break;

    case 4:
      *(Uint32 *)p = pixel;
      break;
  }
}

// http://www.sdltutorials.com/sdl-scale-surface
SDL_Surface *ScaleSurface(SDL_Surface *Surface, Uint16 Width, Uint16 Height)
{
  if(!Surface || !Width || !Height)
    return 0;

  SDL_Surface *_ret = SDL_CreateRGBSurface(Surface->flags, Width, Height, Surface->format->BitsPerPixel,
                                           Surface->format->Rmask, Surface->format->Gmask,
                                           Surface->format->Bmask, Surface->format->Amask);

  double    _stretch_factor_x = (static_cast<double>(Width)  / static_cast<double>(Surface->w)),
      _stretch_factor_y = (static_cast<double>(Height) / static_cast<double>(Surface->h));

  for(Sint32 y = 0; y < Surface->h; y++)
    for(Sint32 x = 0; x < Surface->w; x++)
      for(Sint32 o_y = 0; o_y < _stretch_factor_y; ++o_y)
        for(Sint32 o_x = 0; o_x < _stretch_factor_x; ++o_x)
          putpixel(_ret, static_cast<Sint32>(_stretch_factor_x * x) + o_x,
                   static_cast<Sint32>(_stretch_factor_y * y) + o_y, getpixel(Surface, x, y));
  return _ret;
}

////////////////////////////////////////////////////////////////////////////////

void render_point(SDL_Renderer* renderer, Point2d p, int thickness,
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

////////////////////////////////////////////////////////////////////////////////

inline void Mix_FreeChunk_safe(Mix_Chunk * & chunk) {
  if (chunk)
    Mix_FreeChunk(chunk);
  chunk = NULL;
}

////////////////////////////////////////////////////////////////////////////////

class Texture {
public:
  Texture() {
    _sdltex = NULL;
    _sdlsurface_raw = _sdlsurface_rescaled = NULL;
    _width =  _height = 0;
    _resize_scale = 1;
  }
  ~Texture() { free(); }

  void free() {
    if(!_width)
      return;
    DEBUG_PRINT("Texture::free(%ix%i)\n", _width, _height);
    _width =  _height = 0;
    _resize_scale = 1;
    //Free texture if it exists
    if (_sdltex != NULL)
      SDL_DestroyTexture( _sdltex );
    if (_sdlsurface_raw != NULL)
      SDL_FreeSurface( _sdlsurface_raw );
    _sdltex = NULL;
    _sdlsurface_raw = NULL;
  } // end free()

  //////////////////////////////////////////////////////////////////////////////

  inline bool empty() const { return _width == 0 || _height == 0;}
  inline int get_width() const { return _width;}
  inline int get_height() const { return _height;}
  inline double get_resize_scale() const { return _resize_scale;}
  inline Point2d center() const  { return Point2d(get_width()/2, get_height()/2); }

  //////////////////////////////////////////////////////////////////////////////

  bool from_file(SDL_Renderer* renderer, const std::string &str,
                 int goalwidth = -1, int goalheight = -1, double goalscale = -1) {
    DEBUG_PRINT("Texture::from_file('%s'), goal:(%i, %i, %g)\n",
                str.c_str(), goalwidth, goalheight, goalscale);
    free();
    // Load image as SDL_Surface
    _sdlsurface_raw = IMG_Load( str.c_str() );
    if( _sdlsurface_raw == NULL ) {
      printf( "Unable to load image %s! SDL Error: %s\n", str.c_str(), SDL_GetError() );
      return false;
    }
    return from_surface(renderer, goalwidth, goalheight, goalscale);
  }// end from_file()

  //////////////////////////////////////////////////////////////////////////////

  bool from_ros_image(SDL_Renderer* renderer, const sensor_msgs::Image & img,
                      int goalwidth = -1, int goalheight = -1, double goalscale = -1) {
    DEBUG_PRINT("Texture::from_ros_image(), goal:(%i, %i, %g)\n",
                goalwidth, goalheight, goalscale);
    // Load image as SDL_Surface
    const unsigned char* data = img.data.data();
    int depth = 24; // the depth of the surface in bits
    int pitch = img.step; // the length of a row of pixels in bytes
    Uint32 Rmask = 0, Gmask = 0, Bmask = 0, Amask = 0;

    if (img.encoding == "bgr8") {
      depth = 24; // the depth of the surface in bits
      // TODO : Warning Rmask and Bmask has been inverted for the sumo ! encoding information is not coherent
      Rmask = 0xff0000;
      Gmask = 0x00ff00;
      Bmask = 0x0000ff;
      Amask = 0;
    } else if (img.encoding == "rgba8") {
      depth = 32; // the depth of the surface in bits
      // http://www.gamedev.net/topic/227811-sdl_creatergbsurfacefrom/
#if SDL_BYTEORDER == SDL_BIG_ENDIAN
      Rmask = 0xff000000;
      Gmask = 0x00ff0000;
      Bmask = 0x0000ff00;
      Amask = 0x000000ff;
#else
      Rmask = 0x000000ff;
      Gmask = 0x0000ff00;
      Bmask = 0x00ff0000;
      Amask = 0xff000000;
#endif
    } else {
      printf("from_ros_image(): encoding '%s' is not supported!\n",
             img.encoding.c_str());
      return false;
    }

    // http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
    // https://wiki.libsdl.org/SDL_CreateRGBSurfaceFrom
    // free if needed
    SDL_FreeSurface( _sdlsurface_raw );
    _sdlsurface_raw = SDL_CreateRGBSurfaceFrom((void*) data, img.width, img.height,
                                               depth, pitch,
                                               Rmask, Gmask, Bmask, Amask);
    if( _sdlsurface_raw == NULL ) {
      printf( "Unable to load texture from ROS image! SDL Error: %s\n",
              SDL_GetError() );
      return false;
    }
    return from_surface(renderer, goalwidth, goalheight, goalscale);
  } // end from_ros_image()

  //////////////////////////////////////////////////////////////////////////////

  bool from_surface(SDL_Renderer* renderer,
                    int goalwidth = -1, int goalheight = -1, double goalscale = -1) {
    SDL_Surface* src_surface = _sdlsurface_raw;
    if (goalwidth <= 0 && goalheight <= 0 && goalscale <= 0) {
      _resize_scale = 1;
    }
    else {
      double scalex =  (goalwidth > 0 ? 1. * goalwidth / _sdlsurface_raw->w : 1E6);
      double scaley =  (goalheight > 0 ? 1. * goalheight / _sdlsurface_raw->h : 1E6);
      double scalescale =  (goalscale > 0 ? goalscale : 1E6);
      _resize_scale = std::min(scalescale, std::min(scalex, scaley));
      int resized_w = _resize_scale * _sdlsurface_raw->w;
      int resized_h = _resize_scale * _sdlsurface_raw->h;
      SDL_FreeSurface( _sdlsurface_rescaled );
      _sdlsurface_rescaled = ScaleSurface(_sdlsurface_raw, resized_w, resized_h);
      src_surface = _sdlsurface_rescaled;
    }

    // SDL_Surface is just the raw pixels
    // Convert it to a hardware-optimzed texture so we can render it
    SDL_DestroyTexture( _sdltex );
    _sdltex = SDL_CreateTextureFromSurface( renderer, src_surface );
    if (_sdltex == NULL) {
      printf("Could not convert surface -> texture :'%s'\n", SDL_GetError());
      return false;
    }
    //Get image dimensions
    _width = src_surface->w;
    _height = src_surface->h;
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool from_rendered_text(SDL_Renderer* renderer,
                          TTF_Font *font,
                          std::string textureText,
                          Uint8 r, Uint8 g, Uint8 b, Uint8 a = 255,
                          Uint8 bgr = 0, Uint8 bgg = 0, Uint8 bgb = 0, Uint8 bga = 0) {
    //Get rid of preexisting texture
    free();
    //Render text surface
    SDL_Color fg = {r, g, b, a};
    if (bga <= 0)
      _sdlsurface_raw = TTF_RenderText_Solid( font, textureText.c_str(), fg );
    else {
      SDL_Color bg = {bgr, bgg, bgb, bga};
      _sdlsurface_raw = TTF_RenderText_Shaded( font, textureText.c_str(), fg, bg );
    }
    if( _sdlsurface_raw == NULL ) {
      printf( "Unable to render text surface! SDL_ttf Error: %s\n", TTF_GetError() );
      return false;
    }
    //Create texture from surface pixels
    _sdltex = SDL_CreateTextureFromSurface( renderer, _sdlsurface_raw );
    if( _sdltex == NULL ) {
      printf( "Unable to create texture from rendered text! SDL Error: %s\n", SDL_GetError() );
      return false;
    }
    //Get image dimensions
    _width = _sdlsurface_raw->w;
    _height = _sdlsurface_raw->h;
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool render( SDL_Renderer* renderer, const Point2i & p,
               double scale = 1, SDL_Rect* clip = NULL,
               double angle_rad = 0, Point2d center = Point2d(-1, -1),
               SDL_RendererFlip flip = SDL_FLIP_NONE) const {
    //Set rendering space and render to screen
    SDL_Rect renderQuad = { p.x, p.y,
                            (int) (scale * _width),
                            (int) (scale * _height) };

    //Set clip rendering dimensions
    if( clip != NULL ) {
      renderQuad.w = scale * clip->w;
      renderQuad.h = scale * clip->h;
    }
    //Render to screen
    if (flip == SDL_FLIP_NONE && fabs(angle_rad) < 1E-2) {
      bool ok = (SDL_RenderCopy( renderer, _sdltex, clip, &renderQuad ) == 0);
      if (!ok)
        printf("SDL_RenderCopy() returned an error '%s'!\n", SDL_GetError());
      return ok;
    }

    SDL_Point psdl = p.to_sdl();
    SDL_Point* psdl_ptr = (center.x < 0 && center.y < 0 ? NULL : &psdl);
    bool ok = (SDL_RenderCopyEx( renderer, _sdltex, clip, &renderQuad,
                                 angle_rad * RAD2DEG, psdl_ptr, flip ) == 0);
    if (!ok)
      printf("SDL_RenderCopyEx() returned an error '%s'!\n", SDL_GetError());
    return ok;
  } // end render()

  inline bool render_center( SDL_Renderer* renderer, const Point2i & p,
                             double scale = 1, SDL_Rect* clip = NULL,
                             double angle_rad = 0, Point2d center = Point2d(-1, -1),
                             SDL_RendererFlip flip = SDL_FLIP_NONE) const {
    Point2i p_c;
    p_c.x = p.x - scale*this->center().x;
    p_c.y = p.y - scale*this->center().y;
    return render( renderer, p_c, scale, clip, angle_rad, center, flip);
  } // end render_center()

  inline bool render_center( SDL_Renderer* renderer, const Point2d & p,
                             double scale = 1, SDL_Rect* clip = NULL,
                             double angle_rad = 0, Point2d center = Point2d(-1, -1),
                             SDL_RendererFlip flip = SDL_FLIP_NONE) const {
    return render_center(renderer, p.recast<int>(), scale, clip, angle_rad, center, flip);
  } // end render_center()


  //////////////////////////////////////////////////////////////////////////////

  // http://www.sdltutorials.com/sdl-per-pixel-collision
  //! \return alpha in [0, 255], or < 0 if out of bounds
  inline int get_alpha(const Point2d & p) const {
    if (p.x < 0 || p.x >= get_width()
        || p.y < 0 || p.y >= get_height())
      return -1;
    Uint8 red, green, blue, alpha;
    SDL_GetRGBA(getpixel(_sdlsurface_raw, p.x, p.y),
                _sdlsurface_raw->format, &red, &green, &blue, &alpha);
    return alpha;
  }

  //Image dimensions
  int _width, _height;

private:
  //The actual hardware texture
  SDL_Texture* _sdltex;
  SDL_Surface* _sdlsurface_raw, *_sdlsurface_rescaled;
  //Image dimensions
  // int _width, _height;
  double _resize_scale;
}; // end Texture

////////////////////////////////////////////////////////////////////////////////

class Entity {
public:
  Entity() {
    _tex_ptr = NULL;
    _bbox_offset.resize(4);
    _tex_radius = _angle = _angspeed = 0;
    _rendering_scale  = 1;
    _compute_tight_bbox_needed = true;
    _collision_pt = Point2d(-1, -1);
    set_position(Point2d(0, 0));
  }

  double get_update_timer() const               { return  _update_timer.getTimeSeconds(); }
  double get_life_timer()   const               { return  _life_timer.getTimeSeconds(); }
  void set_angle(const double & angle)          { _angle = angle; }
  double get_angle() const                      { return  _angle; }
  void set_angspeed(const double & angspeed)    { _angspeed = angspeed; }
  double get_angspeed() const                   { return  _angspeed; }
  void increase_angle(const double & dangle)    {
    _compute_tight_bbox_needed = true;
    _angle += dangle;
  }
  void set_accel(const Point2d & accel)         { _accel = accel; }
  void renorm_accel(const double & newnorm)     { _accel.renorm(newnorm); }
  Point2d get_accel() const                     { return  _accel; }
  void set_speed(const Point2d & speed)         { _speed = speed; }
  void renorm_speed(const double & newnorm)     { _speed.renorm(newnorm); }
  Point2d get_speed() const                     { return  _speed; }
  void set_tan_nor_speed(const Point2d & speed) { _speed = rotate(speed, _angle); }
  void set_position(const Point2d & position)   {
    _compute_tight_bbox_needed = true;
    _position = position;
    update_children_positions();
  }
  Point2d get_position() const                  { return  _position; }
  void advance(const double & dist) {
    set_position(_position + rotate(Point2d(dist, 0), _angle));
  }
  void rotate_towards_speed_direction() {
    if (fabs(_speed.y)>1E-2)
      _angle = atan2(_speed.y, _speed.x);
  }
  void update_pos_speed() {
    vision_utils::Timer::Time time = _update_timer.getTimeSeconds();
    _compute_tight_bbox_needed = true;
    _angle += time * _angspeed;
    _speed += time * _accel;
    _position += time * _speed;
    // update children
    for (unsigned int i = 0; i < _children.size(); ++i)
      _children[i].second.update_pos_speed();
    update_children_positions();
    _update_timer.reset();
  }

  bool set_texture(Texture* texture) {
    _tex_ptr = texture;
    _tex_radius = hypot(get_width(), get_height()) / 2;
    _entity_radius = _tex_radius * _rendering_scale;
    _bbox_offset[0] = Point2d(0, 0);
    _bbox_offset[1] = Point2d(0, get_height());
    _bbox_offset[2] = Point2d(get_width(), get_height());
    _bbox_offset[3] = Point2d(get_width(), 0);
    _compute_tight_bbox_needed = true;
    return true;
  } // end from_file()
  Texture* get_texture() const { return _tex_ptr; }

  void set_rendering_scale(const double & rendering_scale) {
    _rendering_scale = rendering_scale;
    _entity_radius = _tex_radius * _rendering_scale;
  }
  double get_rendering_scale() const                     { return  _rendering_scale; }
  double get_entity_radius()   const                     { return  _entity_radius; }
  inline int get_width() const {
    return ( _tex_ptr  ? _tex_ptr->get_width() : -1);
  }
  inline int get_height() const {
    return (_tex_ptr  ? _tex_ptr->get_height() : -1);
  }
  bool is_visible(int winw, int winh) {
    return (_position.x >= -_entity_radius
            && _position.x <= winw+_entity_radius
            && _position.y >= -_entity_radius
            && _position.y <= winh+_entity_radius);
  }
  void add_child(const Point2d & offset, const Entity & child) {
    _children.push_back(std::make_pair(offset, child));
  }

  bool render(SDL_Renderer* renderer) {
    if (!_tex_ptr) {
      printf("Entity::render() failed : no texture set\n");
      return false;
    }
    if (!_tex_ptr->render_center(renderer, _position, _rendering_scale, NULL, _angle)) {
      printf("Entity::render() failed : tex_ptr->render_center() failed.\n");
      return false;
    }
    bool ok = true;
    for (unsigned int i = 0; i < _children.size(); ++i)
      ok = ok && _children[i].second.render(renderer);
#ifdef DEBUG
    //render_point(renderer, _position, 3, 255, 0, 0, 255);
    render_arrow(renderer, _position, _position + _speed, 255, 0, 0, 255);
    render_arrow(renderer, _position, _position + _accel, 0, 255, 0, 255);
    SDL_Rect rb;
    rough_bbox(rb);
    render_rect(renderer, rb, 200, 0, 0, 255);
    render_polygon(renderer, get_tight_bbox(), 0, 255, 0, 255);
    if (_collision_pt.x > 0)
      render_point(renderer, _collision_pt, 5, 255, 255, 0);
#endif
    return ok;
  }

  inline Point2d offset2world_pos(const Point2d & p) const {
    return _position + rotate(_rendering_scale * (p - _tex_ptr->center()), _angle);
  }
  inline Point2d world_pos2offset(const Point2d & p) const {
    return _tex_ptr->center() + rotate(p - _position, -_angle);
  }

  inline void rough_bbox(SDL_Rect & bbox) const {
    bbox.x = _position.x - _entity_radius;
    bbox.y = _position.y - _entity_radius;
    bbox.w = 2 * _entity_radius;
    bbox.h = 2 * _entity_radius;
  }
  inline void compute_tight_bbox_if_needed() {
    if (!_compute_tight_bbox_needed)
      return;
    _compute_tight_bbox_needed = false;
    _tight_bbox.resize(4);
    for (unsigned int i = 0; i < _bbox_offset.size(); ++i)
      _tight_bbox[i] = offset2world_pos(_bbox_offset[i]);
  }

  inline bool collides_with(Entity & other,
                            int minalpha = 1) {
    // rough radius check
    if ((_position-other._position).norm() > _entity_radius + other._entity_radius)
      return false;
    // tight bbox check
    if (!IsPolygonsIntersecting(get_tight_bbox(), other.get_tight_bbox()))
      return false;
    // http://www.sdltutorials.com/sdl-per-pixel-collision
    // compute rectangle intersection between both rough bboxes
    SDL_Rect aB, bB, inter;
    rough_bbox(aB);
    other.rough_bbox(bB);
    SDL_IntersectRect(&aB, &bB, &inter);
    // for each point of the intersect, check if it can be a collision pt
    std::vector<Point2d> aT = get_tight_bbox(), bT = get_tight_bbox();
    for (int x = 0; x < inter.w; ++x) {
      _collision_pt.x = inter.x + x;
      for (int y = 0; y < inter.h; ++y) {
        _collision_pt.y = inter.y + y;
        // first check the test pt belongs to the tight bbox
        if (!point_inside_polygon(_collision_pt, aT)
            || !point_inside_polygon(_collision_pt, bT))
          continue;
        // transform test point in picture frame
        Point2d PA = world_pos2offset(_collision_pt);
        // check alpha
        if (_tex_ptr->get_alpha(PA) < minalpha)
          continue;
        Point2d PB = other.world_pos2offset(_collision_pt);
        if (other._tex_ptr->get_alpha(PB) < minalpha)
          continue;
        // matching pixel found
        return true;
      } // end loop y
    } // end loop x
    _collision_pt = Point2d(-1, -1);
    return false; // no matching pixel found
  }

protected:
  void update_children_positions() {
    for (unsigned int i = 0; i < _children.size(); ++i)
      _children[i].second.set_position( offset2world_pos( _children[i].first ) );
  }

  vision_utils::Timer _life_timer, _update_timer;
  Point2d _position, _accel, _speed;
  double _angle, _angspeed;
  double _tex_radius, _entity_radius, _rendering_scale;
  bool _compute_tight_bbox_needed;
  inline std::vector<Point2d> & get_tight_bbox() {
    compute_tight_bbox_if_needed();
    return _tight_bbox;
  }
  Texture* _tex_ptr;
  Point2d _collision_pt;
  std::vector<Point2d> _bbox_offset, _tight_bbox;
  std::vector< std::pair<Point2d, Entity> > _children;
}; // end class Entity

#endif // SDL_UTILS_H


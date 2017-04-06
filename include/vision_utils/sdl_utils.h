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

// vision_utils
#include <vision_utils/foo_point.h>
#include <vision_utils/timer.h>
#include <vision_utils/point_inside_polygon.h>
// Ros
#include <sensor_msgs/Image.h>
// C++
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <sstream>
#include <vector>

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

typedef vision_utils::FooPoint2i Point2i;
typedef vision_utils::FooPoint2d Point2d;

////////////////////////////////////////////////////////////////////////////////

inline Point2d rotate(const Point2d &pt, double angle) {
  double cosa = cos(angle), sina = sin(angle);
  Point2d ans;
  ans.x = ROTATE_COSSIN_X(pt.x, pt.y, cosa, sina);
  ans.y = ROTATE_COSSIN_Y(pt.x, pt.y, cosa, sina);
  return ans;
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

Uint32 color2int(const SDL_Color & c) {
#if SDL_BYTEORDER == SDL_BIG_ENDIAN
  return c.r << 16 | c.g << 8 | c.b;
#else
  return c.r | c.g << 8 | c.b << 16;
#endif
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
    return from_inner_surface(renderer, goalwidth, goalheight, goalscale);
  }// end from_file()

  //////////////////////////////////////////////////////////////////////////////

  bool from_ros_image(SDL_Renderer* renderer, const sensor_msgs::Image & img,
                      int goalwidth = -1, int goalheight = -1, double goalscale = -1) {
    DEBUG_PRINT("Texture::from_ros_image(), goal:(%i, %i, %g)\n",
                goalwidth, goalheight, goalscale);
    free();
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
    return from_inner_surface(renderer, goalwidth, goalheight, goalscale);
  } // end from_ros_image()

  //////////////////////////////////////////////////////////////////////////////

  inline bool from_surface(SDL_Surface* s,
                           SDL_Renderer* renderer) {
    free();
    _sdlsurface_raw = s;
    return from_inner_surface(renderer, -1, -1, -1);
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

    SDL_Point psdl = { p.x, p.y };
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
    return render_center(renderer, Point2i(p.x, p.y), scale, clip, angle_rad, center, flip);
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

  //////////////////////////////////////////////////////////////////////////////

  //! get raw fields of the texture. Only use if you know what you are doing.
  inline SDL_Surface* get_sdlsurface_raw()       { return _sdlsurface_raw; }
  //! get raw fields of the texture. Only use if you know what you are doing.
  inline SDL_Texture* get_sdltex()       { return _sdltex; }

  //////////////////////////////////////////////////////////////////////////////
private:
  //////////////////////////////////////////////////////////////////////////////

  bool from_inner_surface(SDL_Renderer* renderer,
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
        if (!vision_utils::point_inside_polygon(_collision_pt, aT)
            || !vision_utils::point_inside_polygon(_collision_pt, bT))
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


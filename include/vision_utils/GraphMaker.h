/*!
 * \file GraphMaker.h
 *
 * TODO descr
 *
 * \date Dec 18, 2010
 * \author Arnaud Ramey
 */

#ifndef GRAPHMAKER_H_
#define GRAPHMAKER_H_

// openCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// std
#include <vector>

#define TICK_SIZE 2
#define TICK_LABEL_SHIFT 15

template<class Xtype, class Ytype>
class GraphMaker {
public:
    typedef typename std::vector<std::pair<Xtype, Ytype> > Curve;

    GraphMaker();
    virtual ~GraphMaker();

    /*! set the buffer where we draw */
    void set_image(cv::Mat & image);

    /*!
     * set the coordinates of the window
     * @param xmin
     * @param xmax
     * @param xtickstep
     * @param ymin
     * @param ymax
     * @param ytickstep
     */
    void set_window(const Xtype & xmin, const Xtype & xmax,
                    const double & xtickstep,
                    const Ytype & ymin, const Ytype & ymax,
                    const double & ytickstep);

    /*!
     * add a curve to draw
     * @param curve
     *          the data
     * @param color
     *          which color to draw the curve with
     *          use CV_RGBB(r, g, b) to specify it
     * @param thickness
     *          thickness of the draw inpixels
     */
    void add_curve(const Curve & curve, const cv::Scalar & color,
                   const int thickness = 1);

    /*!
     * reset all the curves in memory
     */
    void remove_all_curves();

    /*!
     * draw the x and y axes on the image
     * @param axes_color
     *          use CV_RGBB(r, g, b) to specify it
     */
    void draw_axes(const cv::Scalar & axes_color);

    /*!
     * draw the curves
     * @param want_axes
     *          true to draw the axes on the graph
     * @param clear_image_before
     *          true to clear the image before drawing anything
     */
    void draw(bool want_axes = true, bool clear_image_before = true);

    /*!
     * draw a vertical line
     * @param x
     * @param color
     * @param thickness
     */
    void draw_vertical(const Xtype & x, const cv::Scalar & color,
                       const int thickness);

    /*!
     * draw a vertical line
     * @param y
     * @param color
     * @param thickness
     */
    void draw_horizontal(const Ytype & y, const cv::Scalar & color,
                         const int thickness);

private:

    void draw_curve(const Curve & curve, const cv::Scalar & color,
                    const int thickness);

    void recompute_convert_coeffs();
    double convert_x_a;
    double convert_x_b;
    double convert_y_a;
    double convert_y_b;
    int convert_coord_x(const Xtype & xReal);
    int convert_coord_y(const Ytype & yReal);

    cv::Mat image;
    std::vector<Curve> curves_to_draw;
    std::vector<cv::Scalar> curves_to_draw_colors;
    std::vector<int> curves_to_draw_thicknesses;

    Xtype xmin;
    Xtype xmax;
    Ytype ymin;
    Ytype ymax;
    double xtickstep;
    double ytickstep;
};

/* *****************************************************************************
 * implementation
 ******************************************************************************/
//#include "vision_utils/utils/debug_utils.h"

template<class Xtype, class Ytype>
GraphMaker<Xtype, Ytype>::GraphMaker() {
    image.create(1, 1, CV_8UC3);
}

template<class Xtype, class Ytype>
GraphMaker<Xtype, Ytype>::~GraphMaker() {
}

template<class Xtype, class Ytype>
inline void GraphMaker<Xtype, Ytype>::set_window(const Xtype & xmin,
                                                 const Xtype & xmax,
                                                 const double & xtickstep,
                                                 const Ytype & ymin,
                                                 const Ytype & ymax,
                                                 const double & ytickstep) {
    this->xmin = xmin;
    this->xmax = xmax;
    this->xtickstep = xtickstep;
    this->ymin = ymin;
    this->ymax = ymax;
    this->ytickstep = ytickstep;

    recompute_convert_coeffs();
}

template<class Xtype, class Ytype>
inline void GraphMaker<Xtype, Ytype>::recompute_convert_coeffs() {
    // recompute the coeffs
    convert_x_a = 1.f * image.cols / (xmax - xmin);
    convert_x_b = -1.f * convert_x_a * xmin;
    convert_y_a = 1.f * image.rows / (ymax - ymin);
    convert_y_b = -1.f * convert_y_a * ymin;
}

template<class Xtype, class Ytype>
inline void GraphMaker<Xtype, Ytype>::set_image(cv::Mat & image) {
    //maggieDebug2("set_image(%i x %i)", image.cols, image.rows);
    this->image = image;
    recompute_convert_coeffs();
}

template<class Xtype, class Ytype>
inline void GraphMaker<Xtype, Ytype>::add_curve(const Curve & curve,
                                                const cv::Scalar & color,
                                                const int thickness /*=1*/) {
    curves_to_draw.push_back(curve);
    curves_to_draw_colors.push_back(color);
    curves_to_draw_thicknesses.push_back(thickness);
}

template<class Xtype, class Ytype>
inline void GraphMaker<Xtype, Ytype>::remove_all_curves() {
    curves_to_draw.clear();
    curves_to_draw_colors.clear();
    curves_to_draw_thicknesses.clear();
}

template<class Xtype, class Ytype>
inline int GraphMaker<Xtype, Ytype>::convert_coord_x(const Xtype & xReal) {
    return (int) (convert_x_a * xReal + convert_x_b);
}

template<class Xtype, class Ytype>
inline int GraphMaker<Xtype, Ytype>::convert_coord_y(const Ytype & yReal) {
    return image.rows - (int) (convert_y_a * yReal + convert_y_b);
}

template<class Xtype, class Ytype>
inline void GraphMaker<Xtype, Ytype>::draw_axes(const cv::Scalar & axes_color) {
    /*
     * x axis
     */
    int x_axe_y = convert_coord_y(0);
    cv::line(image, cv::Point(0, x_axe_y), cv::Point(image.cols, x_axe_y),
         axes_color);
    // ticks
    int x_tick_y = (x_axe_y < 0 || x_axe_y > image.rows - 50 ? 0 : x_axe_y);
    int tick_x_index_min = -1 + (int) (xmin / xtickstep);
    int tick_x_index_max = 1 + (int) (xmax / xtickstep);
    for (int tick_x_index = tick_x_index_min; tick_x_index < tick_x_index_max; ++tick_x_index) {
        double tick_x = tick_x_index * xtickstep;
        //maggieDebug2("tick_x:%f, convert_coord_x(tick_x):%i", tick_x, convert_coord_x(tick_x));
        int tick_x_graph = convert_coord_x(tick_x);
        line(image, cv::Point(tick_x_graph, x_tick_y - TICK_SIZE), cv::Point(
                tick_x_graph, x_tick_y + TICK_SIZE), axes_color);
        std::ostringstream tick_x_stream;
        tick_x_stream << tick_x;
        putText(image, tick_x_stream.str(), cv::Point(tick_x_graph, x_tick_y
                                                      + TICK_LABEL_SHIFT), cv::FONT_HERSHEY_PLAIN, 1, axes_color);
    } // end loop tick_x_index

    /*
     * y axis
     */
    int y_axe_x = convert_coord_x(0);
    line(image, cv::Point(y_axe_x, 0), cv::Point(y_axe_x, image.rows),
         axes_color);
    // ticks
    int y_tick_x = (y_axe_x < 0 || y_axe_x > image.cols - 50 ? 0 : y_axe_x);
    int tick_y_index_min = -1 + (int) (ymin / ytickstep);
    int tick_y_index_may = 1 + (int) (ymax / ytickstep);
    for (int tick_y_index = tick_y_index_min; tick_y_index < tick_y_index_may; ++tick_y_index) {
        double tick_y = tick_y_index * ytickstep;
        //maggieDebug2("tick_y:%f, convert_coord_y(tick_y):%i", tick_y, convert_coord_y(tick_y));
        int tick_y_graph = convert_coord_y(tick_y);
        line(image, cv::Point(y_tick_x - TICK_SIZE, tick_y_graph), cv::Point(
                y_tick_x + TICK_SIZE, tick_y_graph), axes_color);
        std::ostringstream tick_y_stream;
        tick_y_stream << tick_y;
        putText(image, tick_y_stream.str(), cv::Point(y_tick_x
                                                      + TICK_LABEL_SHIFT, tick_y_graph), cv::FONT_HERSHEY_PLAIN, 1,
                axes_color);
    } // end loop tick_y_index
}

template<class Xtype, class Ytype>
inline void GraphMaker<Xtype, Ytype>::draw_curve(const Curve & curve,
                                                 const cv::Scalar & color, int thickness) {
    if (curve.size() < 1)
        return;

    int last_x = convert_coord_x(curve.front().first);
    int last_y = convert_coord_y(curve.front().second);

    for (typename Curve::const_iterator point = curve.begin(); point
                                                != curve.end(); ++point) {
        int current_x = convert_coord_x(point->first);
        int current_y = convert_coord_y(point->second);
        //maggieDebug2("x:%f, y:%f, current_x:%i, current_y:%i", point->first, point->second, current_x, current_y);
        line(image, cv::Point(last_x, last_y), cv::Point(current_x, current_y),
             color, thickness);
        last_x = current_x;
        last_y = current_y;
    }
}

//#include <iostream>
template<class Xtype, class Ytype>
inline void GraphMaker<Xtype, Ytype>::draw(bool want_axes /*= true*/,
                                           bool clear_image_before /*= true*/) {
    //maggieDebug2("draw(%i x %i)", image.cols, image.rows);
    //std::cout << xmin << ", " << xmax << std::endl;

    /* fill in white */
    if (clear_image_before)
        image.setTo(255);

    /* axes */
    if (want_axes)
        draw_axes(CV_RGB(0, 0, 0));

    /* curves */
    for (unsigned int curve_idx = 0; curve_idx < curves_to_draw.size(); ++curve_idx) {
        draw_curve(curves_to_draw.at(curve_idx), curves_to_draw_colors.at(
                curve_idx), curves_to_draw_thicknesses.at(curve_idx));
    }
}

template<class Xtype, class Ytype>
inline void GraphMaker<Xtype, Ytype>::draw_vertical(const Xtype & x,
                                                    const cv::Scalar & color,
                                                    const int thickness) {
    cv::line(image, cv::Point(convert_coord_x(x), 0), cv::Point(
            convert_coord_x(x), image.rows), color, thickness);
}

template<class Xtype, class Ytype>
inline void GraphMaker<Xtype, Ytype>::draw_horizontal(const Ytype & y,
                                                      const cv::Scalar & color,
                                                      const int thickness) {
    cv::line(image, cv::Point(0, convert_coord_y(y)), cv::Point(image.cols,
                                                                convert_coord_y(y)), color, thickness);
}

#endif /* GRAPHMAKER_H_ */


/*!
  \file        histogram_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/1

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

Some useful tools for handling with histograms
 */

#ifndef HISTOGRAM_UTILS_H
#define HISTOGRAM_UTILS_H

#include <opencv2/imgproc/imgproc.hpp>
#include "vision_utils/colormaps.h"
#include <gnuplot-cpp/gnuplot_i.hpp>
#include <stats/stats_utils.h>
#include "vision_utils/drawing_utils.h"
#include "vision_utils/layer_utils.h"

namespace histogram_utils {

typedef cv::MatND Histogram;

//! normalizes the histogram bins by scaling them so that the sum of the bins becomes equal to factor.
void normalize_hist(Histogram & hist, const double factor = 1.0) {
  cv::normalize(hist, hist, factor, 0, cv::NORM_L1);
} // end normalize_hist();

//////////////////////////////////////////////////////////////////////////////

//! Computes the 1D histogram of the first channel of the image
Histogram get_histogram(const cv::Mat &image,
                        const int & nbins, const double max_value,
                        const cv::Mat & mask = cv::Mat(),
                        bool want_normalize_hist = true) {
  Histogram hist;
  int histSize[] = {nbins};
  float hranges[] = { 0, max_value };
  const float* ranges[] = { hranges };
  int channels[] = {0};
  // Compute histogram
  cv::calcHist(&image,
               1, // histogram from 1 image only
               channels, // the channel used
               mask, // no mask is used
               hist, // the resulting histogram
               1, // it is a 1D histogram
               histSize, // number of bins
               ranges // pixel value range
               );
  if (want_normalize_hist)
    normalize_hist(hist);
  return hist;
} // end get_histogram()

////////////////////////////////////////////////////////////////////////////////

//! Computes the 1D histogram of the first channel of the image
Histogram get_hue_histogram(const cv::Mat3b &rgb,
                            cv::Mat3b& hsv_buffer,
                            cv::Mat1b& hue_buffer,
                            const int & nbins, const double max_value,
                            const cv::Mat & mask = cv::Mat(),
                            bool want_normalize_hist = true) {
  color_utils::rgb2hue(rgb, hsv_buffer, hue_buffer);
  return get_histogram(hue_buffer, nbins, max_value, mask, want_normalize_hist);
}


////////////////////////////////////////////////////////////////////////////////

void get_vector_of_histograms(const std::vector<cv::Mat> &images,
                              std::vector<Histogram> & hists,
                              const int & nbins, const double max_value,
                              const std::vector<cv::Mat> & masks = std::vector<cv::Mat>(),
                              bool want_normalize_hist = true) {
  bool use_masks = (masks.size() == images.size());
  unsigned int n_hists = images.size();
  hists.resize(n_hists);
  for (unsigned int img_idx = 0; img_idx < n_hists; ++img_idx)
    hists[img_idx] = get_histogram(images[img_idx], nbins, max_value,
                                   (use_masks ? masks[img_idx] : cv::Mat()),
                                   want_normalize_hist);
} // end get_vector_of_histograms();

////////////////////////////////////////////////////////////////////////////////

void get_vector_of_histograms(const cv::Mat &image,
                              std::vector<Histogram> & hists,
                              const int & nbins, const double max_value,
                              const std::vector<cv::Mat> & masks,
                              bool want_normalize_hist = true) {
  unsigned int n_hists = masks.size();
  hists.resize(n_hists);
  for (unsigned int mask_idx = 0; mask_idx < n_hists; ++mask_idx)
    hists[mask_idx] = get_histogram(image, nbins, max_value, masks[mask_idx], want_normalize_hist);
} // end get_vector_of_histograms();

////////////////////////////////////////////////////////////////////////////////

/*!
 * Get all the histograms of a one channel image
 * thanks to a collection of masks.
 * \param image
 *    one channel, 8 bytes
 * \param hists
 *    the output vector of Histogram , of size n_masks
 * \param nbins, max_value
 *    \see get_histogram()
 * \param multimask
 *   values of the multimask are 0 (no mask), 1, 2, .., n_hists
 * \param n_masks
 *   the number of masks in the multimask
 * \param want_normalize_hist
 */
void get_vector_of_histograms(const cv::Mat &image,
                              std::vector<Histogram> & hists,
                              const int nbins, const double max_value,
                              const cv::Mat & multimask,
                              const unsigned int n_masks,
                              bool want_normalize_hist = true) {
  hists.resize(n_masks);
  for (unsigned int mask_idx = 1; mask_idx <= n_masks; ++mask_idx) {
    //cv::Mat curr_mask = (multimask == (uchar) mask_idx);
    hists[mask_idx - 1] = get_histogram
                          (image, nbins, max_value, (multimask == mask_idx), want_normalize_hist);
  }
} // end get_vector_of_histograms();

////////////////////////////////////////////////////////////////////////////////

//! \return true if the histogram is empty (bins are full of zeros)
inline bool is_histogram_empty(const Histogram & hist) {
  return (hist.rows == 0 || cv::countNonZero(hist) == 0);
}

////////////////////////////////////////////////////////////////////////////////

bool hist_max(const Histogram & hist,
              double & max_bin_value, double & max_bin_index,
              bool skip_first_bin = false) {
  int nbins = hist.rows;
  if (nbins < (skip_first_bin ? 2 : 1)) {
    printf("max_bin_value(): histogram too small\n");
    return false;
  }
  max_bin_value = 0;
  max_bin_index = -1;
  for (int bin_idx = (skip_first_bin ? 1 : 0); bin_idx < nbins; ++bin_idx) {
    float bin_value = hist.at<float>(bin_idx);
    if (max_bin_value < bin_value) {
      max_bin_value = bin_value;
      max_bin_index = bin_idx;
    }
  } // end loop bin_idx
  return true;
}

inline std::string hue_hist_dominant_color_to_string(const Histogram & hist,
                                                     bool skip_first_bin = false) {
  if (is_histogram_empty(hist))
    return "empty";

  int nbins = hist.rows, nread_bins = (skip_first_bin ? nbins - 1 : nbins);
  double max_bin_value, max_bin_index;
  if (!hist_max(hist, max_bin_value, max_bin_index, skip_first_bin))
    return "histogram too small";

  // check distribution of colors
  double uniform_bin_value =
      (cv::norm(hist, cv::NORM_L1) - (skip_first_bin ? hist.at<float>(0) : 0))
      / nread_bins;
  if (max_bin_index < 0 || max_bin_value < 3 * uniform_bin_value)
    return "multicolor";
  // one dominant color
  float h = 180.f * max_bin_index / nbins;
  //  printf("h:%f\n", h);
  //  cv::Vec3b h2rgb = color_utils::hue2rgb<cv::Vec3b>(h);
  //  std::cout  << "h2rgb:" << h2rgb << std::endl;
  return color_utils::hue_to_string(h);
} // end hue_hist_dominant_color_to_string()

////////////////////////////////////////////////////////////////////////////////

inline std::string hist_to_string(const Histogram &hist) {
  std::ostringstream bin_content;
  int nbins = hist.rows;
  // float sum = 0;
  float sum = cv::norm(hist, cv::NORM_L1); // fast computing instead of manual summing
  for( int bin_idx = 0; bin_idx < nbins; bin_idx++ ) {
    float binVal = hist.at<float>(bin_idx);
    bin_content << "[" << bin_idx << ": " << binVal << "] ";
    // sum += binVal;
  }
  std::ostringstream out;
  out << "hist: " << nbins << " bins"
         // << ", sum:" << sum
      << ", norm: L1 (sum): " << sum
      << ", L2: " << cv::norm(hist, cv::NORM_L2)
      << ", INF: " << cv::norm(hist, cv::NORM_INF)
      << ", average value: " << sum / nbins
      << ", bin_content:" << bin_content.str();
  return out.str();
} // end hist_to_string()

////////////////////////////////////////////////////////////////////////////////

inline void write_histogram_to_file(const histogram_utils::Histogram & hist,
                                    const std::string & output_filename) {
  cv::FileStorage fs(output_filename, cv::FileStorage::WRITE);
  fs << "hist" << hist;
  fs.release();
}

inline histogram_utils::Histogram read_histogram_from_file
(const std::string & input_filename)
{
  histogram_utils::Histogram hist;
  cv::FileStorage fs(input_filename, cv::FileStorage::READ);
  fs["hist"] >> hist;
  fs.release();
  return hist;
}

////////////////////////////////////////////////////////////////////////////////

// Computes the 1D histogram and returns an image of it.
void histogram_to_image(const Histogram &hist,
                        cv::Mat3b & histImg,
                        const int width, const int height,
                        colormaps::RatioColormap ratio_colormap
                        = colormaps::ratio2grey){
  // Image on which to display histogram
  histImg.create(height, width);
  histImg.setTo(cv::Scalar::all(255));
  if (hist.empty())
    return;
  // Get min and max bin values
  double maxVal=0;
  double minVal=0;
  cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
  // special case: minVal = maxVal
  if (minVal == maxVal)
    maxVal = minVal + 1;
  int hpt = 0.1 * height; // set highest point at 90% of height
  // find the linear mapping   "y = alpha * x + beta" such as
  // 0 -> height
  // maxVal -> hpt
  double alpha = (hpt - height) / maxVal, beta = hpt - alpha * maxVal;
  int nbins = hist.rows;
  double bin_width = 1.f * width / nbins;
  // draw average
  int average_px = alpha / nbins + beta;
  cv::line(histImg, cv::Point(0, average_px), cv::Point(histImg.cols, average_px),
           cv::Scalar::all(150), 1);
  // write dominant color
  if (ratio_colormap == colormaps::ratio2hue)
    image_utils::draw_text_centered
        (histImg, hue_hist_dominant_color_to_string(hist, true),
         cv::Point(width / 2, hpt / 2), cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 0, 0));
  //    cv::putText(histImg, hue_hist_dominant_color_to_string(hist, true),
  //                cv::Point(10, hpt - 3), cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 0, 0));
  // Draw a vertical line for each bin
  for( int bin_idx = 0; bin_idx < nbins; bin_idx++ ) {
    float binVal = hist.at<float>(bin_idx);
    // printf("bin_idx:%i, binVal:%g\n", bin_idx, binVal);
    cv::Point p1(bin_idx * bin_width, beta); // a zero value
    cv::Point p2((bin_idx + 1) * bin_width, alpha * binVal + beta);
    // printf("p1:(%i, %i), p2:(%i, %i)\n", p1.x, p1.y, p2.x, p2.y);
    cv::rectangle(histImg, p1, p2, ratio_colormap(1.f * bin_idx / nbins), -1);
    cv::rectangle(histImg, p1, p2, cv::Scalar::all(0), 1);
  }
} // end histogram_to_image();

////////////////////////////////////////////////////////////////////////////////

void vector_of_histograms_to_image(const std::vector<Histogram> & hists,
                                   cv::Mat3b & histImg,
                                   const int width1, const int height1,
                                   colormaps::RatioColormap ratio_colormap
                                   = colormaps::ratio2grey,
                                   std::vector<bool>* refresh_mask = NULL){
  unsigned int n_hists = hists.size();
  // check refresh_mask
  bool use_refresh_mask = (refresh_mask != NULL);
  if (use_refresh_mask && refresh_mask->size() != n_hists) {
    printf("vector_of_histograms_to_image(): error, refresh_mask->size()=%i != n_hists=%i\n",
           refresh_mask->size(), n_hists);
    return;
  }
  // printf("vector_of_histograms_to_image(n_hists:%i)\n", n_hists);

  // check bounds
  int min_width = width1, min_height = height1 * n_hists;
  if (histImg.cols < min_width || histImg.rows < min_height)
    histImg.create(min_height, min_width); // rows, cols

  // draw each hist
  for (unsigned int hist_idx = 0; hist_idx < n_hists; ++hist_idx) {
    // if this histogram is not in refresh_mask, do nothing
    if (use_refresh_mask && !(*refresh_mask)[hist_idx])
      continue;
    // get ROI and draw histogram in it
    cv::Mat3b histImg_roi = histImg(cv::Rect(0, height1 * hist_idx, width1, height1));
    histogram_utils::histogram_to_image(hists[hist_idx], histImg_roi,
                                        width1, height1, ratio_colormap);
    // draw a separator line
    cv::line(histImg_roi, cv::Point(0, histImg_roi.rows - 1),
             cv::Point(histImg_roi.cols, histImg_roi.rows - 1), CV_RGB(0, 0, 0));
  } // end loop hist_idx
} // end vector_of_histograms_to_image();

////////////////////////////////////////////////////////////////////////////////

/*! distance between two histograms
 * \param h1
 *   First compared histogram.
 * \param h2
 *   Second compared histogram of the same size as H1.
 * \param remaps_to_0_1
 *  true for remapping value to 0 .. 1.
 *  In that case, h1 and h2 must be normalized
 * \param method
 *  cf cv::compareHist():
 *  CV_COMP_CHISQR: sums the normalized square difference between the bins
 *  CV_COMP_INTERSECT: simply compares, for each bin, the two values
 *  in each histogram, and keeps the minimum one.
 *  The similarity measure is then simply the sum of these minimum values.
 *  Consequently, two images having histograms with no colors in common would
 *  get an intersection value of 0, while two identical histograms would get a
 *  value equal to the total number of pixels.
 *  CV_COMP_CORREL: is based on the normalized cross-correlation operator used
 *  in signal processing to measure the similarity between two signals
 *  CV_COMP_BHATTACHARYYA: used in statistics to estimate the similarity between
 *  two probabilistic distributions
 * \return
 *   the value normalization
 *   if remaps_to_0_1, return 0 for identical histograms
 *                     and 1 for completely different histograms
 */
double distance_hists(const Histogram & h1,
                      const Histogram & h2,
                      const int method = CV_COMP_BHATTACHARYYA,
                      bool remaps_to_0_1 = true) {
  double raw_value = cv::compareHist(h1, h2, method);
  if (!remaps_to_0_1)
    return raw_value;
  switch (method) {
    case CV_COMP_INTERSECT: // 0 = very diff -> 1 = identical
      return 1. - raw_value;
    case CV_COMP_CHISQR: // 2 = very diff -> 0 = identical
      return raw_value / 2.;
    case CV_COMP_CORREL: //  -1 = opposite -> 0 = very diff -> 1 = identical
      return std::min(1., 1. - raw_value);
      //return (-raw_value + 1) / 2.f;
    case CV_COMP_BHATTACHARYYA: // 1 = very diff -> 0 = identical
    default:
      return raw_value;
  } // end switch (method)
} // end distance_hists()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Merge two histograms as an arithmetic average
 * \param h1, h2
 *    input histograms
 * \param out
 *    output histogram
 * \param want_normalize_hist
 *    true for normalizing the hist
 */
bool merge_histograms(const Histogram & h1,
                      const Histogram & h2,
                      Histogram & out,
                      double weight1 = 1,
                      double weight2 = 1,
                      bool want_normalize_hist = true) {
  if(h1.rows != h2.rows) { // check same number of bins
    printf("merge_histograms(): different nb of bins: %i, %i! Returning\n",
           h1.rows, h2.rows);
    return false;
  }
  // make average
  if (weight1 == 1 && weight2 == 1) // avoid consuming and useless multiplication
    out = h1 + h2;
  else
    out = weight1 * h1 + weight2 * h2;
  if (want_normalize_hist)
    normalize_hist(out);
  return true;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Merge two histograms as an arithmetic average
 * \param h1, h2
 *    input histograms
 * \param out
 *    output histogram
 * \param want_normalize_hist
 *    true for normalizing the hist
 */
bool merge_histograms(const std::vector<Histogram> & hists,
                      const std::vector<double> & weights,
                      Histogram & out,
                      bool want_normalize_hist = true) {
  unsigned int nhists = hists.size();
  if (weights.size() != nhists) { // check size consistency
    printf("merge_histograms(): %i weights, %i histograms! Returning\n",
           weights.size(), nhists);
    return false;
  }
  if (hists.size() == 0) {
    out = Histogram();
    return true;
  }

  out = weights.front() * hists.front();
  for (unsigned int hist_idx = 1; hist_idx < nhists; ++hist_idx) {
    if(out.rows != hists[hist_idx].rows) { // check same number of bins
      printf("merge_histograms(): different nb of bins: hist#%i:%i, out:%i! Returning\n",
             hist_idx, hists[hist_idx].rows, out.rows);
      return false;
    }
    if (weights[hist_idx] == 0) // skip weights equal to zero
      continue;
    out += weights[hist_idx] * hists[hist_idx];
  }
  if (want_normalize_hist)
    normalize_hist(out);
  return true;
}
////////////////////////////////////////////////////////////////////////////////

/*!
 * Compute the average distance of two vectors of histograms.
 * \brief distance_hists
 * \param hists1, hists2
 *  two vectors of histograms, of same size
 * \param method, remaps_to_0_1
 *   cf distance_hists()
 * \return
 *   the average distance between corresonding pairs (hists1, hists2)
 */
inline double distance_hist_vectors(const std::vector<Histogram> & hists1,
                                    const std::vector<Histogram> & hists2,
                                    const int method = CV_COMP_BHATTACHARYYA,
                                    bool remaps_to_0_1 = true) {
  unsigned int nhists1 = hists1.size();
  unsigned int nhists2 = hists2.size();
  if (nhists1 != nhists2) {
    printf("vectors of Histogram with a different number of histograms!"
           "%i != %i\n", nhists1, nhists2);
    return -1;
  }
  unsigned int nhists_non_empty = 0;
  double sum = 0;
  for (unsigned int hist_idx = 0; hist_idx < nhists1; ++hist_idx) {
    if (is_histogram_empty(hists1[hist_idx]) || is_histogram_empty(hists2[hist_idx]))
      continue;
    ++nhists_non_empty;
    sum += histogram_utils::distance_hists
           (hists1[hist_idx], hists2[hist_idx], method, remaps_to_0_1);
  } // end loop hist_idx

  return sum / nhists_non_empty;
} // end distance_hist_vectors()

////////////////////////////////////////////////////////////////////////////////

void histogram2vectors(const Histogram & h, const double max_value,
                       std::vector<double> & bins, std::vector<double> & freqs) {
  unsigned int nbins = h.rows;
  bins.resize(nbins);
  freqs.resize(nbins);
  double hstep = 1. * max_value / nbins;
  for (unsigned int bin_idx = 0; bin_idx < nbins; ++bin_idx) {
    bins[bin_idx] = hstep * (.5 + bin_idx);
    freqs[bin_idx] = h.at<float>(bin_idx);
    // printf("bins:%g, freqs:%g\n", bins[bin_idx], freqs[bin_idx]);
  } // end loop bin_idx
} // histogram2vectors();

////////////////////////////////////////////////////////////////////////////////

void mean_std_dev_modulo(const Histogram & h, const double max_value,
                         double & mean, double & std_dev) {
  // first generate a vector of hue values, another of freqs
  static std::vector<double> hues, freqs, hues_shifted;
  histogram2vectors(h, max_value, hues, freqs);
  // now call mean_std_dev();
  mean_std_dev_grouped_data_modulo(hues, freqs, 0., 180., hues_shifted, mean, std_dev);
} // mean_std_dev();

////////////////////////////////////////////////////////////////////////////////

/*! plot histogram in gnuplot
  \var mean, std_dev: if wanted, draw the PDF of the histogram
                      (obtained with mean_std_dev() )
  \var xmin, xmax: only useful if you want to extend the drawing of the gaussian PDF
  \var normalize_pdf: true to set the max value of the PDF
                      to the max value of the histogram (i.e., scale the PDF)
*/
void histogram2gnuplot(const Histogram & h, const double max_value,
                       const double mean = -1, const double std_dev = -1,
                       const double xmin = -1, const double xmax = -1,
                       bool normalize_pdf = true) {
  std::vector<double> hues, freqs;
  histogram2vectors(h, max_value, hues, freqs);
  // plot!
  Gnuplot plotter;
  if (xmin != -1 && xmax != -1)
    plotter.set_xrange(xmin, xmax);
  plotter.set_style("boxes");
  plotter.plot_xy(hues, freqs);
  if (mean != -1 && std_dev != -1) {
    std::ostringstream equa;
    if (normalize_pdf) {
      double max_bin_value, max_bin_index;
      if (hist_max(h, max_bin_value, max_bin_index, false))
        equa << max_bin_value << " * ";
    }
    equa << gaussian_pdf_as_string(mean, std_dev, true); // max value = 1
    plotter.set_style("lines");
    plotter.plot_equation(equa.str());
  }
  system_utils::wait_for_key();
} // end mean_std_dev2gnuplot();

} // end namespace histogram_utils

#endif // HISTOGRAM_UTILS_H

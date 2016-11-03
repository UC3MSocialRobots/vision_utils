/*!
  \file        get_vector_of_histograms.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/2
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
 */

#ifndef GET_VECTOR_OF_HISTOGRAMS_H
#define GET_VECTOR_OF_HISTOGRAMS_H
// std includes
#include <opencv2/core/core.hpp>
#include <vector>

namespace vision_utils {

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

} // end namespace vision_utils

#endif // GET_VECTOR_OF_HISTOGRAMS_H

/*!
  \file        cloud_viewer_gnuplot.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/11/27

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

#ifndef CLOUD_VIEWER_GNUPLOT_H
#define CLOUD_VIEWER_GNUPLOT_H

#include <vector>
#include <opencv2/core/core.hpp>
#include "vision_utils/file_io.h"
#include "vision_utils/system_utils.h"
#include "vision_utils/exec_system.h"

namespace vision_utils {

class CloudViewerGnuPlot {
public:
  CloudViewerGnuPlot() {
    data_filename = "/tmp/CloudViewerGnuPlot.dat";
    script_filename = "/tmp/CloudViewerGnuPlot.gnuplot";
  }

  template<class Pt3>
  void view_cloud(const std::vector<Pt3>& pointcloud,
                  std::string caption = "Point cloud",
                  bool blocking = true) {
    // http://dirsig.blogspot.com.es/2010/11/lidar-point-cloud-visualization.html
    int nsamples = pointcloud.size();
    std::ostringstream data, script_content;
    for (int sample_idx = 0; sample_idx < nsamples; ++sample_idx)
      data << pointcloud[sample_idx].x << ' '
           << pointcloud[sample_idx].y << ' '
           << pointcloud[sample_idx].z << ' '<< std::endl;
    save_file(data_filename, data.str());
    script_content << "gnuplot -e \""
                      // << "set xyplane at 1e-6; "
                   << "set grid; "
                   << "set view equal xyz; "
                   << "set xlabel 'x' font ',18'; "
                   << "set ylabel 'y' font ',18'; "
                   << "set zlabel 'z' font ',18'; "
                   << "set palette rgb 33,13,10; ";
    if (pointcloud.size() > 0)
      script_content << "splot '" << data_filename << "' using 1:2:3:3 with points lw 2 palette title '"
                     << caption << "'; ";
    else
      script_content << "splot 0 ; ";
    script_content  << "\"";
    if (blocking) // http://www.gnuplot.info/faq/faq.html#SECTION00094000000000000000
      script_content  << " -persist";
    exec_system(script_content.str());
  }

  template<class Pt3>
  void view_rgb_cloud(const std::vector<Pt3>& pointcloud,
                      const std::vector<cv::Vec3b>& pointcloud_RGB,
                      std::string caption = "Point cloud",
                      bool blocking = true) {
    // http://stackoverflow.com/questions/5914162/gnuplot-variable-colors-and-linewidths-for-2d-vector-plot
    int nsamples = pointcloud.size();
    std::ostringstream data, script_content;
    for (int sample_idx = 0; sample_idx < nsamples; ++sample_idx)
      data << pointcloud[sample_idx].x << ' '
           << pointcloud[sample_idx].y << ' '
           << pointcloud[sample_idx].z << ' '
           << (int) pointcloud_RGB[sample_idx][0] << ' '
           << (int) pointcloud_RGB[sample_idx][1] << ' '
           << (int) pointcloud_RGB[sample_idx][2] << ' '
           << std::endl;
    save_file(data_filename, data.str());
    script_content // << "set xyplane at 1e-6; "
        << "set grid; "
        << "set view equal xyz; "
        << "set xlabel 'x' font ',18'; "
        << "set ylabel 'y' font ',18'; "
        << "set zlabel 'z' font ',18'; "
        << "rgb(r,g,b) = int(r)*65536 + int(g)*256 + int(b); ";
    if (pointcloud.size() > 0)
      script_content << "splot '" << data_filename << "' using 1:2:3:(rgb($4,$5,$6)) "
                     << "with points lw 2 lc rgb variable title '"
                     << caption << "'; ";
    else
      script_content << "splot 0 ; ";
    save_file(script_filename, script_content.str());
    std::ostringstream command;
    command << " gnuplot -e \"load '" << script_filename << "'\"";
    if (blocking) // http://www.gnuplot.info/faq/faq.html#SECTION00094000000000000000
      command  << " -persist";
    exec_system(command.str());
  }

  std::string data_filename, script_filename;
}; // end class CloudViewer

} // end namespace vision_utils

#endif // CLOUD_VIEWER_GNUPLOT_H

/*!
  \file        KTP2PPL.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/18

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

\section Parameters
  - \b "~display"
      [bool, default: true]

\section Publications
  - \b "rgb", "depth", "user"
        [sensor_msgs/Image]
        The image

  - \b "ground_truth_ppl"
        [people_msgs::PeoplePoseList]
        The ground truth positions
 */
#include <image_transport/image_transport.h>
// AD
#include <string/find_and_replace.h>
#include <string/file_io.h>
#include <time/timer.h>
#include <point_clouds/blob_segmenter.h>
#include <color/color_utils.h>
#include <databases_io/database_player.h>
// people_msgs
#include <ppl_utils/rgb_depth_user2ppl.h>
#include <ppl_utils/ppl_attributes.h>

typedef people_msgs::PeoplePose PP;
typedef people_msgs::PeoplePoseList PPL;
typedef cv::Point3d Pt3d;
static const std::string CAMFRAME = "/openni_rgb_optical_frame", BASEFRAME = "/base_link";

class KTP2Imgs : public DatabasePlayer {
public:

  KTP2Imgs() {}

  //////////////////////////////////////////////////////////////////////////////

  bool go_to_next_frame() {
    ++_file_it;
    if (_file_it == _files.end()) // txt file finished
      return go_to_next_file();
    return load_current_frame();
  }


  //////////////////////////////////////////////////////////////////////////////

  void set_header(const std_msgs::Header & header) { _header = header; }
  inline const PPL & get_ground_truth_ppl() const { return _ground_truth_ppl; }
  inline const PPL & get_anonymous_ppl()    const { return _anonymous_ppl; }

  //////////////////////////////////////////////////////////////////////////////

  virtual char display() {
    _bgr.copyTo(_bgr_display);
    const std::vector<ImgData::UserData>* users_ptr = &(_file_it->second.usersdata);
    unsigned int nusers = users_ptr->size();
    for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
      const ImgData::UserData* userdata = &(users_ptr->at(user_idx));
      UserId user_id = userdata->userid;
      cv::Rect bbox = geometry_utils::shrink_rec(userdata->bbox, .3, .3);
      cv::Scalar color = color_utils::color_scalar<cv::Scalar>(user_id);
      std::swap(color[0], color[2]); // RGB -> BGR
      cv::rectangle(_bgr_display, bbox, color, 2);
    }
    cv::imshow("bgr_display", _bgr_display);
    return DatabasePlayer::display();
  } // end display
protected:

  //////////////////////////////////////////////////////////////////////////////

  bool load_single_video(const std::string & txt_filename) {
    printf("load_single_video('%s')\n", txt_filename.c_str());
    // parse 2D and 3D files
    std::string txt2d_filename = txt_filename;
    StringUtils::find_and_replace(txt2d_filename, "3D", "2D");
    std::string txt3d_filename = txt2d_filename;
    StringUtils::find_and_replace(txt3d_filename, "2D", "3D");
    if (!system_utils::file_exists(txt2d_filename)
        ||!system_utils::file_exists(txt3d_filename)) {
      printf("KTP2Imgs: Could not find files '%s' or '%s'\n",
             txt2d_filename.c_str(), txt3d_filename.c_str());
      return false;
    }
    // KTP_dataset_images/ground_truth/Still_gt2D.txt -> KTP_dataset_images/images/Still
    _img_folder = txt2d_filename + "/";
    StringUtils::find_and_replace(_img_folder, "ground_truth", "images");
    StringUtils::find_and_replace(_img_folder, "_gt2D.txt", "");
    if (!system_utils::directory_exists(_img_folder)) {
      printf("KTP2Imgs: Could not find  folder '%s'\n", _img_folder.c_str());
      return false;
    }

    for (unsigned int file = 0; file < 2; ++file) {
      bool use2D = (file == 0); // 2D
      unsigned int nfields_per_user = (use2D ? 5 : 4);
      std::string filename = (use2D ? txt2d_filename : txt3d_filename);
      // printf("KTP2Imgs: filename:'%s'\n", filename.c_str());
      std::vector<std::string> lines;
      StringUtils::retrieve_file_split(filename, lines, true);
      unsigned int nlines = lines.size();
      for (unsigned int line_idx = 0; line_idx < nlines; ++line_idx) {
        std::string line = lines[line_idx];
        // printf("KTP2PPL: line '%s'\n", line.c_str());
        StringUtils::find_and_replace(line, ":", "");
        StringUtils::find_and_replace(line, ",", "");
        std::vector<std::string> words;
        StringUtils::StringSplit(line, " ", &words);
        unsigned int nwords = words.size();
        // printf("nwords:%i\n", nwords);
        if ((nwords-1) % nfields_per_user != 0) {
          printf("KTP2PPL: incorrect line '%s'\n", line.c_str());
          return false;
        }
        Filename currfile = words[0];
        _files[currfile]; // create the ImgData
        unsigned int nusers = ((nwords-1) / nfields_per_user);
        for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
          unsigned int fieldspos = nfields_per_user*user_idx+1;
          UserId currid = StringUtils::cast_from_string<UserId>(words[fieldspos]);
          if (use2D) {
            cv::Rect* bbox = &(_files[currfile].get_userdata_by_id(currid)->bbox);
            bbox->x = StringUtils::cast_from_string<int>(words[fieldspos+1]);
            bbox->y = StringUtils::cast_from_string<int>(words[fieldspos+2]);
            bbox->width = StringUtils::cast_from_string<int>(words[fieldspos+3]);
            bbox->height = StringUtils::cast_from_string<int>(words[fieldspos+4]);
          } else { // 3D pos
            Pt3d* pos = &(_files[currfile].get_userdata_by_id(currid)->pos);
            //printf("word1:'%s'\n", words[fieldspos+1].c_str());
            pos->x = StringUtils::cast_from_string<double>(words[fieldspos+1]);
            pos->y = StringUtils::cast_from_string<double>(words[fieldspos+2]);
            pos->z = StringUtils::cast_from_string<double>(words[fieldspos+3]);
            //printf("currfile '%s', pos:'%s'\n", currfile.c_str(), geometry_utils::printP(*pos).c_str());
          } // end if (use2D)
        } // end loop user_idx

        if (!use2D) // sort users by increasing distance to avoid occlusions
          std::sort(_files[currfile].usersdata.begin(),_files[currfile].usersdata.end());

      } // end loop line_idx
      printf("KTP2Imgs: %i files after reading '%s' \n", _files.size(), filename.c_str());
    } // end loop file

    // print first example
    Filename firstfile = _files.begin()->first;
    ImgData::UserData firstdata = _files.begin()->second.usersdata.front();
    UserId firstid = firstdata.userid;
    printf("First entry: file %s, user %i, bbox %s, pt %s\n",
           firstfile.c_str(), firstid,
           geometry_utils::print_rect(firstdata.bbox).c_str(),
           geometry_utils::printP(firstdata.pos).c_str());
    _file_it = _files.begin();
    return load_current_frame();
  } // end load_single_video()

  //////////////////////////////////////////////////////////////////////////////

  bool load_current_frame() {
    // printf("KTP2Imgs: file:'%s'\n", _file_it->first.c_str());
    _has_rgb = _has_depth = _has_user = true;

    // read rgb
    // _file_it->first: eg: "1339064317.986969085"
    std::ostringstream rgb_filename;
    rgb_filename << _img_folder << "rgb/" << _file_it->first << ".jpg";
    // printf("KTP2Imgs: reading rgb:'%s'\n", rgb_filename.str().c_str());
    _bgr = cv::imread(rgb_filename.str(), CV_LOAD_IMAGE_COLOR);
    if (_bgr.empty()) {
      printf("KTP2Imgs: Could not load rgb:'%s'\n", rgb_filename.str().c_str());
      return false;
    }

    // read depth
    std::ostringstream depth_filename;
    depth_filename << _img_folder << "depth/" << _file_it->first << ".pgm";
    // printf("KTP2Imgs: reading depth:'%s'\n", depth_filename.str().c_str());
    _depth16 = cv::imread(depth_filename.str(), CV_LOAD_IMAGE_UNCHANGED);
    if (_depth16.empty()) {
      printf("KTP2Imgs: Could not load depth:'%s'\n", depth_filename.str().c_str());
      return false;
    }
    _depth16.convertTo(_depth32f, CV_32FC1, 1E-3);

    // computer user
    // printf("KTP2Imgs: building user\n");
    _user8.create(_depth32f.size());
    _user8.setTo(0);
    std::map<UserId, cv::Point3f> user_positions;
    const std::vector<ImgData::UserData>* users_ptr = &(_file_it->second.usersdata);
    unsigned int nusers = users_ptr->size();
    for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
      const ImgData::UserData* userdata = &(users_ptr->at(user_idx));
      UserId user_id = userdata->userid;
      user_positions[user_id] = userdata->pos; // keep the ground truth user pos
      if (userdata->pos.x == 0 && userdata->pos.y == 0 && userdata->pos.z == 0) {
        continue;
      }
      cv::Rect bbox = geometry_utils::shrink_rec(userdata->bbox, .3, .3);
      cv::Point seed = geometry_utils::rect_center<cv::Rect, cv::Point>(bbox);
      unsigned int try_idx = 0, max_ntries = 100;
      for (; try_idx < max_ntries; ++try_idx) {
        if (!std_utils::is_nan_depth(_depth32f(seed)) // real depth reading
            && _user8(seed) == 0 // seed not used before
            && _segmenter.find_blob(_depth32f, seed, _curr_user,
                                    BlobSegmenter::GROUND_PLANE_FINDER,
                                    NULL, false, // <- ground_recompute_plane
                                    DepthCanny::DEFAULT_CANNY_THRES1 / 6,
                                    DepthCanny::DEFAULT_CANNY_THRES2 / 4))
          break;
        // random seed
        seed.x = bbox.x + rand() % bbox.width;
        seed.y = bbox.y + rand() % bbox.height;
      } // end loop try_idx
      if (try_idx >= max_ntries)
        printf("KTP2Imgs: file:'%s', user_id %i:could not generate user mask\n",
               _file_it->first.c_str(), user_id);
      _user8.setTo(user_id, _curr_user);
    } // end loop user_idx

    // convert to PPL
    if (!_ppl_conv.convert(_bgr, _depth32f, _user8, NULL, &_header))
      return false;
    _ground_truth_ppl = _ppl_conv.get_ppl();
    _ground_truth_ppl.method = "ground_truth";
    _ground_truth_ppl.header.frame_id = BASEFRAME;
    // use real 3D positions
    nusers = _ground_truth_ppl.poses.size();
    for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
      PP* curr_pp = &(_ground_truth_ppl.poses[user_idx]);
      UserId currid = StringUtils::cast_from_string<int>(curr_pp->person_name);
      if (!user_positions.count(currid)) {
        printf("KTP2Imgs: file:'%s', user %i has no 3D pose!\n",
               _file_it->first.c_str(), currid);
        return false;
      }
      pt_utils::copy3(user_positions[currid], curr_pp->head_pose.position);
      // set headers
      curr_pp->header = _ground_truth_ppl.header;
      curr_pp->rgb.header = _ground_truth_ppl.header;
      curr_pp->depth.header = _ground_truth_ppl.header;
      curr_pp->user.header = _ground_truth_ppl.header;
    }

      // anonymise it
    _anonymous_ppl.method = "ktp2ppl";
    _anonymous_ppl = _ground_truth_ppl;
    for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
      PP* curr_pp = &(_anonymous_ppl.poses[user_idx]);
      ppl_utils::set_attribute(*curr_pp, "ground_truth_label", curr_pp->person_name);
      curr_pp->person_name = people_msgs::PeoplePose::NO_RECOGNITION_MADE;
    } // end loop user_idx
    return true;
  } // end load_current_frame()

  //////////////////////////////////////////////////////////////////////////////

  typedef unsigned int UserId;
  typedef std::string Filename;
  struct ImgData {
    struct UserData {
      UserId userid;
      cv::Rect bbox;
      Pt3d pos;
      bool operator < (const UserData& str) const {
        // user far: y = -4, user close: y = -1
        return (pos.y > str.pos.y);
      }
    }; // end class UserData
    std::vector<UserData> usersdata;
    UserData* get_userdata_by_id(const UserId & id) {
      for (unsigned int user_idx = 0; user_idx < usersdata.size(); ++user_idx) {
        if(usersdata[user_idx].userid == id)
          return &(usersdata[user_idx]);
      } // end loop user_idx
      usersdata.push_back(UserData());
      usersdata.back().userid = id;
      return &(usersdata.back());
    } // end get_userdata_by_id()
  }; // end class ImgData
  std::map<Filename, ImgData> _files;
  std::map<Filename, ImgData>::const_iterator _file_it;

  cv::Mat _depth16;
  std::string _img_folder;
  BlobSegmenter _segmenter;
  cv::Mat1b _curr_user;
  ppl_utils::RgbDepthUser2PPL _ppl_conv;

  std_msgs::Header _header;
  PPL _ground_truth_ppl, _anonymous_ppl;
  cv::Mat3b _bgr_display;
}; // end class KTP2Imgs

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static const int QUEUE_SIZE = 10;
int KTP2PPL(int argc, char **argv) {
  if (argc < 2) {
    printf("Synopsis: %s [onifile]\n", argv[0]);
    return -1;
  }
  std::string filename = argv[1];
  KTP2Imgs reader;
  bool repeat = false;
  if (!reader.from_file(filename, repeat))
    return -1;

  // create publishers
  ros::NodeHandle nh_public, nh_private("~");
  image_transport::ImageTransport transport(nh_public);
  image_transport::Publisher
      rgb_pub = transport.advertise("rgb", QUEUE_SIZE),
      depth_pub = transport.advertise("depth", QUEUE_SIZE),
      user_pub = transport.advertise("user", QUEUE_SIZE);
  ros::Publisher anon_ppl_pub = nh_public.advertise<PPL>("anon_ppl", 1),
      ground_truth_ppl_pub = nh_public.advertise<PPL>("ground_truth_ppl", 1);
  bool display = true;
  nh_private.param("display", display, display);
  // PPL stuff
  cv_bridge::CvImage depth_bridge, user_bridge, rgb_bridge;

  printf("KTP2PPL: publishing rgb on '%s', depth on '%s', user on '%s', truth PPL on '%s'\n",
         rgb_pub.getTopic().c_str(), depth_pub.getTopic().c_str(), user_pub.getTopic().c_str(),
         ground_truth_ppl_pub.getTopic().c_str());

  while (ros::ok()) {
    Timer timer;
    std_msgs::Header curr_header;
    curr_header.stamp = ros::Time::now();
    reader.set_header(curr_header);
    if (!reader.go_to_next_frame()) {
      printf("KTP2PPL: couldn't go_to_next_frame()!\n");
      break;
    }
    ROS_INFO_THROTTLE(10, "Time for go_to_next_frame(): %g ms.", timer.getTimeMilliseconds());
    const cv::Mat3b & bgr = reader.get_bgr();
    const cv::Mat1f & depth = reader.get_depth();
    const cv::Mat1b & user = reader.get_user();

    // publish rgb, depth, user
    curr_header.frame_id = CAMFRAME;
    rgb_bridge.header = depth_bridge.header = user_bridge.header = curr_header;
    rgb_bridge.image = bgr;
    rgb_bridge.encoding = sensor_msgs::image_encodings::BGR8;
    depth_bridge.image = depth;
    depth_bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    user_bridge.image = user;
    user_bridge.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    rgb_pub.publish(rgb_bridge.toImageMsg());
    depth_pub.publish(depth_bridge.toImageMsg());
    user_pub.publish(user_bridge.toImageMsg());
    ground_truth_ppl_pub.publish(reader.get_ground_truth_ppl());
    anon_ppl_pub.publish(reader.get_anonymous_ppl());

    // display
    if (display)
      reader.display();
    ros::spinOnce();
  } // end while(ros::ok())
  return 0;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  ros::init(argc, argv, "KTP2PPL");
  return KTP2PPL(argc, argv);
}

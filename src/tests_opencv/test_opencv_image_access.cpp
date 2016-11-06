
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "vision_utils/timer.h"
#include "vision_utils/cmatrix.h"
#include <vision_utils/img_path.h>
#include <set>

#define NTIMES 1000
#define NTIMES_RANDOM_READ 20

//#define VALUE_NB 1
#define VALUE_NB 255 * (col % 2)
//#define VALUE_NB rand() % 255

//#define VALUE_COLOR   cv::Vec3b(1, 2, 3)
#define VALUE_COLOR   cv::Vec3b(255 * (col % 2), 255 * (col % 2), 255 * (col % 2))
//#define VALUE_COLOR   cv::Vec3b(rand() % 255, rand() % 255, rand() % 255)

#define VALUE_3D   (dim1 + dim2 + dim3) % 255

////////////////////////////////////////////////////////////////////////////////

void iterate_whole_image_bw(cv::Mat1b & image) {
  int cols = image.cols, rows = image.rows;
  printf("\niterate_whole_image_bw(%i x %i)\n", cols, rows);

  vision_utils::Timer timer;

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (int row = 0; row < rows; ++row)
      for (int col = 0; col < cols; ++col)
        image(row, col)= VALUE_NB;
  timer.printTime_factor("Mat1b(row, col)", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (int row = 0; row < rows; ++row)
      for (int col = 0; col < cols; ++col)
        image.at<uchar>(row,col)= VALUE_NB;
  timer.printTime_factor("Mat1b.at<uchar>(row,col)", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time) {
    cv::Mat1b::iterator it = image.begin();
    for (int row = 0; row < rows; ++row)
      for (int col = 0; col < cols; ++col)
        *it++ = VALUE_NB;
  } // end loop time
  timer.printTime_factor("iterators", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (int row = 0; row < rows; ++row) {
      // get the address of row
      cv::Mat1b data = image.row(row);
      for (int col = 0; col < cols; ++col) {
        data(col) = VALUE_NB;
      } // end loop col
    } // end loop row
  timer.printTime_factor("Mat1b.row(row) with data(col)", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (int row = 0; row < rows; ++row) {
      // get the address of row
      uchar* data = image.ptr<uchar>(row);
      for (int col = 0; col < cols; ++col) {
        *data++ = VALUE_NB;
      } // end loop col
    } // end loop row
  timer.printTime_factor("Mat1b.ptr<uchar>(row) with data++", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (int row = 0; row < rows; ++row) {
      // get the address of row
      uchar* data = image.ptr<uchar>(row);
      for (int col = 0; col < cols; ++col) {
        data[col] = VALUE_NB;
      } // end loop col
    } // end loop row
  timer.printTime_factor("Mat1b.ptr<uchar>(row) with data[col]", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time) {
    //Some operations, like the above one, do not actually depend on the matrix shape,
    //they just process elements of a matrix one by one (or elements from multiple matrices
    //that are sitting in the same place, e.g. matrix addition). Such operations are called
    //element-wise and it makes sense to check whether all the input/output matrices are continuous,
    //i.e. have no gaps in the end of each row, and if yes, process them as a single long row:
    cols = (image.isContinuous() ? image.cols * image.rows : image.cols);
    rows = (image.isContinuous() ? 1 : image.rows);
    for (int row = 0; row < rows; ++row) {
      // get the address of row
      uchar* data = image.ptr<uchar>(row);
      for (int col = 0; col < cols; ++col) {
        data[col] = VALUE_NB;
      } // end loop col
    } // end loop row
  }
  timer.printTime_factor("Mat1b.ptr<uchar>(row) with data[col] and continuity optimizations", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time) {
    assert(image.isContinuous());
    uchar* data = image.ptr<uchar>(0);
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        data[row * cols + col] = VALUE_NB;
      } // end loop col
    } // end loop row
  }
  timer.printTime_factor("data[row * cols + col]", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time) {
    IplImage image_ipl = (IplImage) image;
    for (int row = 0; row < rows; ++row)
      for (int col = 0; col < cols; ++col)
        CV_IMAGE_ELEM(&image_ipl, uchar, row, col) = VALUE_NB;
  } // end loop time
  timer.printTime_factor("CV_IMAGE_ELEM(image_ipl, uchar, row, col)", NTIMES);
}

////////////////////////////////////////////////////////////////////////////////

void iterate_whole_image_color(cv::Mat3b & image) {
  int cols = image.cols, rows = image.rows;
  vision_utils::Timer timer;
  printf("\niterate_whole_image_color(%i x %i)\n", cols,  rows);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (int row = 0; row < rows; ++row)
      for (int col = 0; col < cols; ++col)
        image(row, col)= VALUE_COLOR;
  timer.printTime_factor("Mat3b(row, col)", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (int row = 0; row < rows; ++row)
      for (int col = 0; col < cols; ++col)
        image.at<cv::Vec3b>(row,col)= VALUE_COLOR;
  timer.printTime_factor("Mat3b.at<cv::Vec3b>(row,col)", NTIMES);

  for (int time = 0; time < NTIMES; ++time) {
    cv::Mat3b::iterator it = image.begin();
    for (int row = 0; row < rows; ++row)
      for (int col = 0; col < cols; ++col)
        *it++ = VALUE_COLOR;
  } // end loop time
  timer.printTime_factor("iterators", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (int row = 0; row < rows; ++row) {
      // get the address of row
      cv::Mat3b data = image.row(row);
      for (int col = 0; col < cols; ++col) {
        data(col) = VALUE_COLOR;
      } // end loop col
    } // end loop row
  timer.printTime_factor("Mat3b.row(row) with data(col)", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time) {
    for (int row = 0; row < rows; ++row) {
      // get the address of row
      uchar* data = image.ptr<uchar>(row);
      for (int col = 0; col < cols; ++col) {
        data[3 * col    ] = VALUE_COLOR[2]; // B
        data[3 * col + 1] = VALUE_COLOR[1]; // G
        data[3 * col + 2] = VALUE_COLOR[0]; // R
      } // end loop col
    } // end loop row
  }
  timer.printTime_factor("Mat3b.ptr<uchar>(row) with data[3 * col + x]", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (int row = 0; row < rows; ++row) {
      // get the address of row
      cv::Vec3b* data = image.ptr<cv::Vec3b>(row);
      for (int col = 0; col < cols; ++col) {
        *data++ = VALUE_COLOR;
      } // end loop col
    } // end loop row
  timer.printTime_factor("Mat3b.ptr<cv::Vec3b>(row) with data++", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (int row = 0; row < rows; ++row) {
      // get the address of row
      cv::Vec3b* data = image.ptr<cv::Vec3b>(row);
      for (int col = 0; col < cols; ++col) {
        data[col] = VALUE_COLOR;
      } // end loop col
    } // end loop row
  timer.printTime_factor("Mat3b.ptr<cv::Vec3b>(row) with data[col]", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time) {
    //Some operations, like the above one, do not actually depend on the matrix shape,
    //they just process elements of a matrix one by one (or elements from multiple matrices
    //that are sitting in the same place, e.g. matrix addition). Such operations are called
    //element-wise and it makes sense to check whether all the input/output matrices are continuous,
    //i.e. have no gaps in the end of each row, and if yes, process them as a single long row:
    cols = (image.isContinuous() ? image.cols * image.rows : image.cols);
    rows = (image.isContinuous() ? 1 : image.rows);
    for (int row = 0; row < rows; ++row) {
      // get the address of row
      cv::Vec3b* data = image.ptr<cv::Vec3b>(row);
      for (int col = 0; col < cols; ++col) {
        data[col] = VALUE_COLOR;
      } // end loop col
    } // end loop row
  } // end loop time
  timer.printTime_factor("Mat3b.ptr<cv::Vec3b>(row) with data[col] and continuity optimizations", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time) {
    IplImage image_ipl = (IplImage) image;
    for (int row = 0; row < rows; ++row)
      for (int col = 0; col < cols; ++col)
        CV_IMAGE_ELEM(&image_ipl, cv::Vec3b, row, col) = VALUE_COLOR;
  } // end loop time
  timer.printTime_factor("CV_IMAGE_ELEM(image_ipl, cv::Vec3b, row, col)", NTIMES);
}

////////////////////////////////////////////////////////////////////////////////

//

void iterate_whole_image_multidimensional() {
  unsigned int SIZE = 50;
  int dim[] = {SIZE, SIZE, SIZE};
  cv::Mat mat(3, dim, CV_8U);
  printf("process_mat_multidimensional(%i x %i x %i)\n", SIZE, SIZE, SIZE);
  vision_utils::Timer timer;

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (unsigned int dim1 = 0; dim1 < SIZE; ++dim1)
      for (unsigned int dim2 = 0; dim2 < SIZE; ++dim2)
        for (unsigned int dim3 = 0; dim3 < SIZE; ++dim3)
          mat.at<uchar>(dim1, dim2, dim3) = VALUE_3D;
  timer.printTime_factor("mat.at<uchar>(dim1, dim2, dim3)", NTIMES);

  timer.reset();
  unsigned int SIZE2 = SIZE * SIZE;
  for (int time = 0; time < NTIMES; ++time) {
    uchar* data = mat.data;
    for (unsigned int dim1 = 0; dim1 < SIZE; ++dim1)
      for (unsigned int dim2 = 0; dim2 < SIZE; ++dim2)
        for (unsigned int dim3 = 0; dim3 < SIZE; ++dim3)
          data[dim1 * SIZE2 + dim2 * SIZE + dim3] = VALUE_3D;
  }
  timer.printTime_factor("data[dim1 * SIZE2 + dim2 * SIZE + dim3]", NTIMES);

  // NAryMatIterator
  // from http://stackoverflow.com/questions/8809517/multi-dimensional-data-in-a-matrix-in-opencv-with-c
  timer.reset();
  for (int time = 0; time < NTIMES; ++time) {
    cv::Mat planes[1];
    cv::Mat* arrays[] = { &mat };
    cv::NAryMatIterator it((const cv::Mat**) arrays, planes, 1);
    // iterate - just one long big row
    // it.planes[*] (of type Mat) will be set to the current plane.
    // iterate through the matrix. on each iteration
    uchar* data = it.planes[0].ptr<uchar>(0);
    //    printf("it.planes[0]:'%s'\n",
    //           vision_utils::infosImage(it.planes[0]).c_str());
    for(unsigned int dim1 = 0; dim1 < SIZE; ++dim1) {
      for (unsigned int dim2 = 0; dim2 < SIZE; ++dim2) {
        for (unsigned int dim3 = 0; dim3 < SIZE; ++dim3) {
          //  printf("it.nplanes:%i, dim1:%i, dim2:%i, dim3:%i\n",
          //         it.nplanes, dim1, dim2, dim3);
          data[dim1 * SIZE2 + dim2 * SIZE + dim3] = VALUE_3D;
        } // end loop dim3
      } // end loop dim2
    } // end loop dim1
    ++it;
  } // end loop time
  timer.printTime_factor("NAryMatIterator it.planes[0].data[dim1 * SIZE2 + dim2 * SIZE + dim3]", NTIMES);

  //  timer.reset();
  //  for (int time = 0; time < NTIMES; ++time) {
  //    IplImage mat_ipl = mat;
  //    for (unsigned int dim1 = 0; dim1 < SIZE; ++dim1)
  //      for (unsigned int dim2 = 0; dim2 < SIZE; ++dim2)
  //        for (unsigned int dim3 = 0; dim3 < SIZE; ++dim3)
  //          CV_IMAGE_ELEM(&mat_ipl, uchar, dim1 * SIZE + dim2, dim3) = VALUE_3D;
  //  }
  //  timer.printTime_factor("CV_IMAGE_ELEM(&mat_ipl, uchar, dim1 * SIZE + dim2, dim3)", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (unsigned int dim1 = 0; dim1 < SIZE; ++dim1) {
      uchar* data = mat.ptr<uchar>(dim1);
      for (unsigned int dim2 = 0; dim2 < SIZE; ++dim2)
        for (unsigned int dim3 = 0; dim3 < SIZE; ++dim3)
          data[dim2 * SIZE + dim3] = VALUE_3D;
    } // end loop dim1
  timer.printTime_factor("mat.ptr<uchar>(dim1) [dim2 * SIZE + dim3]", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    for (unsigned int dim1 = 0; dim1 < SIZE; ++dim1)
      for (unsigned int dim2 = 0; dim2 < SIZE; ++dim2) {
        uchar* data = mat.ptr<uchar>(dim1, dim2);
        for (unsigned int dim3 = 0; dim3 < SIZE; ++dim3)
          data[dim3] = VALUE_3D;
      } // end loop dim2
  timer.printTime_factor("mat.ptr<uchar>(dim1, dim2) [dim3]", NTIMES);

} // end iterate_whole_image_multidimensional();

////////////////////////////////////////////////////////////////////////////////

void random_read_buffer(int cols, int rows, double ratio_read_wanted) {
  int nreads_wanted = ratio_read_wanted * cols * rows;
  printf("\nrandom_read_buffer(%i x %i, %i%%)\n", cols, rows,
         (int) (100 * ratio_read_wanted));

  vision_utils::Timer timer;

  timer.reset();
  for (int time = 0; time < NTIMES_RANDOM_READ; ++time) {
    std::set<unsigned int> prev_reads;
    int nreads = 0;
    while (nreads < nreads_wanted) {
      unsigned int col = rand() % cols, row = rand() % rows,
          key = row * cols + col;
      if (prev_reads.find(key) != prev_reads.end())
        continue;
      // <do stuff with row, col here...>
      prev_reads.insert(key); // mark key as read
      ++nreads;
    } // end while (nreadds < nreads_wanted)
  } // end loop time
  timer.printTime_factor("set<int>", NTIMES_RANDOM_READ);

  // cv::Mat1b
  timer.reset();
  for (int time = 0; time < NTIMES_RANDOM_READ; ++time) {
    cv::Mat1b prev_reads(rows, cols);
    prev_reads.setTo(0);
    uchar* prev_reads_data = prev_reads.ptr<uchar>(0);
    int nreads = 0;
    while (nreads < nreads_wanted) {
      unsigned int col = rand() % cols, row = rand() % rows,
          key = row * cols + col;
      if (prev_reads_data[key])
        continue;
      // <do stuff with row, col here...>
      prev_reads_data[key] = true; // mark key as read
      ++nreads;
    } // end while (nreadds < nreads_wanted)
  } // end loop time
  timer.printTime_factor("cv::Mat1b", NTIMES_RANDOM_READ);

  // bool[][]
  timer.reset();
  for (int time = 0; time < NTIMES_RANDOM_READ; ++time) {
    bool* prev_reads = new bool[cols * rows];
    memset(prev_reads, 0, cols * rows * sizeof(bool));
    int nreads = 0;
    while (nreads < nreads_wanted) {
      unsigned int col = rand() % cols, row = rand() % rows,
          key = row * cols + col;
      if (prev_reads[key])
        continue;
      // <do stuff with row, col here...>
      prev_reads[key] = true; // mark key as read
      ++nreads;
    } // end while (nreadds < nreads_wanted)
    delete prev_reads;
  } // end loop time
  timer.printTime_factor("bool[]", NTIMES_RANDOM_READ);

  // CMatrix
  timer.reset();
  for (int time = 0; time < NTIMES_RANDOM_READ; ++time) {
    vision_utils::CMatrix<bool> prev_reads(rows, cols);
    prev_reads.set_to_zero();
    int nreads = 0;
    while (nreads < nreads_wanted) {
      unsigned int col = rand() % cols, row = rand() % rows;
      if (prev_reads[row][col])
        continue;
      // <do stuff with row, col here...>
      prev_reads[row][col] = true; // mark key as read
      ++nreads;
    } // end while (nreadds < nreads_wanted)
  } // end loop time
  timer.printTime_factor("CMatrix<bool>", NTIMES_RANDOM_READ);

  // vector<bool>
  timer.reset();
  for (int time = 0; time < NTIMES_RANDOM_READ; ++time) {
    std::vector<bool> prev_reads (cols * rows, false);
    int nreads = 0;
    while (nreads < nreads_wanted) {
      unsigned int col = rand() % cols, row = rand() % rows,
          key = row * cols + col;
      if (prev_reads[key])
        continue;
      // <do stuff with row, col here...>
      prev_reads[key] = true; // mark key as read
      ++nreads;
    } // end while (nreadds < nreads_wanted)
  } // end loop time
  timer.printTime_factor("vector<bool>", NTIMES_RANDOM_READ);

  // vector<uchar>
  timer.reset();
  for (int time = 0; time < NTIMES_RANDOM_READ; ++time) {
    std::vector<uchar> prev_reads (cols * rows, false);
    int nreads = 0;
    while (nreads < nreads_wanted) {
      unsigned int col = rand() % cols, row = rand() % rows,
          key = row * cols + col;
      if (prev_reads[key])
        continue;
      // <do stuff with row, col here...>
      prev_reads[key] = true; // mark key as read
      ++nreads;
    } // end while (nreadds < nreads_wanted)
  } // end loop time
  timer.printTime_factor("vector<uchar>", NTIMES_RANDOM_READ);
} // end random_read_buffer();

////////////////////////////////////////////////////////////////////////////////

inline void test_vector() {
  std::vector<double> vec;
  for (unsigned int col = 0; col < 10000; ++col) {
    vec.push_back(1.f * col);
  } // end loop col
  printf("\ntest_vector(size:%li)\n", vec.size());


  vision_utils::Timer timer;
  for (int time = 0; time < NTIMES; ++time) {
    for (unsigned int col = 0; col < vec.size(); ++col) {
      vec.at(col) = VALUE_NB;
    } // end loop col
  } // end loop time
  timer.printTime_factor("at()", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time) {
    for (unsigned int col = 0; col < vec.size(); ++col) {
      vec[col] = VALUE_NB;
    } // end loop col
  } // end loop time
  timer.printTime_factor("[]", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time) {
    std::vector<double>::iterator it = vec.begin();
    for (unsigned int col = 0; col < vec.size(); ++col) {
      *it++ = VALUE_NB;
    } // end loop col
  } // end loop time
  timer.printTime_factor("iterator", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time) {
    double* data = vec.data();
    for (unsigned int col = 0; col < vec.size(); ++col) {
      *data++ = VALUE_NB;
    } // end loop col
  } // end loop time
  timer.printTime_factor("data()", NTIMES);
}

////////////////////////////////////////////////////////////////////////////////

template<class _Elem>
inline void test_fill_img(int w, int h, const _Elem & value) {
  printf("\ntest_fill_img(%i x %i)\n", w, h);
  vision_utils::Timer timer;
  cv::Mat_<_Elem> img(h, w); // rows, cols

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    img.setTo(255);
  timer.printTime_factor("img.setTo(255)", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    img = value;
  timer.printTime_factor("img = value", NTIMES);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    cv::rectangle(img, cv::Rect(0, 0, img.cols, img.rows), cv::Scalar::all(255), -1);
  timer.printTime_factor("cv::rectangle(img, bbox)", NTIMES);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  int idx = 1;
  if (argc < 2) {
    printf("%i: iterate_whole_image_bw()\n", idx++);
    printf("%i: iterate_whole_image_color()\n", idx++);
    printf("%i: iterate_whole_image_multidimensional()\n", idx++);
    printf("%i: random_read_buffer(640, 480, .1)\n", idx++);
    printf("%i: random_read_buffer(640, 480, .4)\n", idx++);
    printf("%i: random_read_buffer(640, 480, .9)\n", idx++);
    printf("%i: test_vector()\n", idx++);
    printf("%i: test_fill_img<uchar>()\n", idx++);
    printf("%i: test_fill_img<float>()\n", idx++);
    printf("%i: test_fill_img<cv::Vec3b>()\n", idx++);
    return -1;
  }

  int choice = 0;
  choice = atoi(argv[1]);

  idx = 1;
  if (choice == idx++) {
    cv::Mat1b image_bw = cv::imread(vision_utils::IMG_DIR() + "pz/pz01.jpg", CV_LOAD_IMAGE_GRAYSCALE);
    iterate_whole_image_bw(image_bw);
  }
  else if (choice == idx++) {
    cv::Mat3b image_color = cv::imread(vision_utils::IMG_DIR() + "pz/pz01.jpg", CV_LOAD_IMAGE_COLOR);
    iterate_whole_image_color(image_color);
  }
  else if (choice == idx++)
    iterate_whole_image_multidimensional();
  else if (choice == idx++)
    random_read_buffer(640, 480, .1);
  else if (choice == idx++)
    random_read_buffer(640, 480, .4);
  else if (choice == idx++)
    random_read_buffer(640, 480, .9);
  else if (choice == idx++)
    test_vector();
  else if (choice == idx++)
    test_fill_img<uchar>(640, 480, 255);
  else if (choice == idx++)
    test_fill_img<float>(640, 480, 255);
  else if (choice == idx++)
    test_fill_img<cv::Vec3b>(640, 480, cv::Vec3b(255, 255, 255));
  return 0;
}

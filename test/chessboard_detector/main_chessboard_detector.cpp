#include "core/buffers/focusEstimator1.h"
#include "core/patterndetection/chessBoardAssembler.h"
#include "core/patterndetection/chessBoardCornerDetector.h"
#include "core/patterndetection/chessBoardDetector.h"
#include "core/reflection/commandLineSetter.h"
#include "core/utils/utils.h"

#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#ifdef WITH_OPENCV
#include "openCvFileReader.h"
#endif
#ifdef WITH_LIBJPEG
#include "libjpegFileReader.h"
#endif
#ifdef WITH_LIBPNG
#include "libpngFileReader.h"
#endif

using namespace corecvs;
using std::cout;
using std::endl;
using std::string;

enum BoardType {
  WithMarkers,    /*!< new board with    markers, which parameters
                           overwrite board parameters from the file */
  WithoutMarkers, /*!< old board without markers, which parameters overwrite
                        board parameters from the file */
  Manual          /*!< board with parameters from the file */
};

static void usage(cchar *progName) {
  cout << "Chessboard detector utility." << endl
       << "Usage:" << endl
       << HelperUtils::getFileNameFromFilePath(progName)
       << " [--file=]image_path --out=path_boards --type=board_type "
          "[--width=W --height=H] [--quality] [--verbose]"
       << endl
       << " * image_path  - path to input image" << endl
       << " * path_boards - path to output text file with boards info" << endl
       << " * board_type  - 0=withMarkers(def), 1=withoutMarkers, 2=manual"
       << endl
       << " * W,H         - expected full pattern size for Manual board type"
       << endl
       << " * quality     - perform aux image-quality analisys" << endl
       << " * verbose     - print an extend info during the process" << endl;
}

#if 0
void readImage(const string &filename, DpImage &img) {
  cv::Mat im = cv::imread(filename);
  im.convertTo(im, CV_64FC1, 1.0 / 255.0);
  // XXX:  OPENCV DOES NOT SUPPORTS FP64 RGB->GRAY
  // TODO: BTW, I hope that it is always BGR order, is it correct?!
  img = DpImage(im.rows, im.cols);
  for (int i = 0; i < im.rows; ++i) {
    for (int j = 0; j < im.cols; ++j) {
      (img.element(i, j) = 0.299 * im.at<double>(i, j * 3 + 2) +
                           0.587 * im.at<double>(i, j * 3 + 1) +
                           0.114 * im.at<double>(i, j * 3));
    }
  }
}

void readImage(const string &filename, corecvs::RGB24Buffer &img) {
  cv::Mat im = cv::imread(filename);
  im.convertTo(im, CV_64FC1, 1.0);
  img = corecvs::RGB24Buffer(im.rows, im.cols);
  for (int i = 0; i < im.rows; ++i) {
    for (int j = 0; j < im.cols; ++j) {
      img.element(i, j) = corecvs::RGBColor(im.at<double>(i, j * 3 + 2),
                                            im.at<double>(i, j * 3 + 1),
                                            im.at<double>(i, j * 3));
    }
  }
}
#endif

static bool processImage(ChessboardDetector &detector, const string &filename,
                         bool testImageQuality, std::ostream &out) {
  // load image
  cout << "Processing: " << filename << endl;
#if 0
  DpImage img;
#else
  std::unique_ptr<corecvs::RGB24Buffer> img;
#endif
  img.reset(BufferFactory::getInstance()->loadRGB24Bitmap(filename));

  // call the main detection method
  bool result = detector.detectPattern(*img);

  // get the result observations: {point+projection}
  corecvs::ObservationList observations;
  detector.getPointData(observations);

  BoardCornersType board = detector.getBestBoard();
  if (result) {
    int bw = (int)board[0].size();
    int bh = (int)board.size();
    cout << filename << "\t- Best board: [" << bw << " x " << bh << "]" << endl;
  } else {
    cout << filename << "\t- No boards found." << endl;
  }

  streamsize oldPrec = out.precision(numeric_limits<double>::digits10);
  out << img->w << " " << img->h << " " << 1 << endl
      << observations.size() << " " << filename << endl;
  for (auto &o : observations) {
    out << o.u() << " " << o.v() << " " << o.x() << " " << o.y() << " " << 0
        << endl;
  }
  out.precision(oldPrec);

  if (result && testImageQuality) {
    cout << endl << "We will run Image Quality analysis" << endl;

    FocusEstimator1 focusEstimator;
    focusEstimator.setBoardInfo({board});
    focusEstimator.setInputImage(img.get());
    focusEstimator.operator()();
    focusEstimator.getResult().print();
  }

  return result;
}

int main(int argc, char **argv) {

#ifdef WITH_OPENCV
  init_opencv_reader_provider();
#endif
#ifdef WITH_LIBJPEG
  LibjpegFileReader::registerMyself();
  SYNC_PRINT(("Libjpeg support on\n"));
#endif
#ifdef WITH_LIBPNG
  LibpngFileReader::registerMyself();
  SYNC_PRINT(("Libpng support on\n"));
#endif

  CommandLineSetter s(argc, argv);
  bool help = s.getBool("help");
  if (help || argc < 2) {
    usage(argv[0]);
    return -1;
  }

  // detect which file is to process
  string filename = s.getOption("file");
  if (filename.empty() && HelperUtils::pathExists(argv[1])) {
    filename = argv[1];
  }
  if (!HelperUtils::pathExists(filename)) {
    cout << "File " << filename << " not found!" << endl;
    return -2;
  }

  // detect which output stream will be used
  string out_filename = s.getOption("out");
  std::ofstream outfile(out_filename);
  bool out_bad = out_filename.empty();
  if (!out_bad && !outfile.is_open()) {
    cout << "File " << out_filename
         << " couldn't be opened for writting! Gonna use stdout." << endl;
    out_bad = true;
  }
  std::ostream &out = out_bad ? cout : outfile;

  // detect some aux flags
  bool testImageQuality = s.getBool("quality");
  bool verbose = s.getBool("verbose");

  // detect board type and its possibly given parameters
  auto board_type = (BoardType)s.getInt("type", BoardType::WithMarkers);
  switch (board_type) {
  case BoardType::WithMarkers:
  case BoardType::WithoutMarkers:
  case BoardType::Manual:
    break;
  default:
    cout << "Invalid board type value: " << board_type << endl;
    return -3;
  }
  int W = s.getInt("width");
  int H = s.getInt("height");

  // detect few sets of parameters for the Detector
  CheckerboardDetectionParameters params;
  params.setCellSizeHor(1.);  // to get board's logic coords instead of in mm
  params.setCellSizeVert(1.);

  BoardAlignerParams alignerParams;
  switch (board_type) {
  case BoardType::WithMarkers:
    cout << "New board type requested" << endl;
    alignerParams = BoardAlignerParams::GetNewBoard();
    break;
  case BoardType::WithoutMarkers:
    cout << "Old board type requested" << endl;
    alignerParams = BoardAlignerParams::GetOldBoard();
    break;
  case BoardType::Manual:
  default:
    cout << "Manual board type requested" << endl;
    alignerParams.idealWidth = W;
    alignerParams.idealHeight = H;
    alignerParams.type = AlignmentType::FIT_WIDTH; // from cmdLine?
    break;
  }

  ChessBoardCornerDetectorParams cbparams;

  ChessBoardAssemblerParams cbap;
  // cbap.setHypothesisDimFirst(W);
  // cbap.setHypothesisDimSecond(H);

  // print all parameters info if need
  if (verbose) {
    cout << "We are using following configs" << endl;

    PrinterVisitor printer(2, 2);
    cout << "CheckerboardDetectionParameters:" << endl;
    params.accept(printer);
    cout << "BoardAlignerParams:" << endl;
    alignerParams.accept(printer);
    cout << "ChessBoardAssemblerParams:" << endl;
    cbap.accept(printer);
    cout << "ChessBoardCornerDetectorParams:" << endl;
    cbparams.accept(printer);
    cout << std::flush;
  }

  ChessboardDetector detector(params, alignerParams, cbparams, cbap);

  Statistics stats;
  detector.setStatistics(&stats);

  bool result = processImage(detector, filename, testImageQuality, out);

  // print statistics info if need
  if (verbose) {
    BaseTimeStatisticsCollector collector;
    collector.addStatistics(stats);
    collector.printAdvanced();
    cout << "result = " << result << endl;
  }

  return result ? 0 : 1; // 0 - found board; otherwise - 1
}

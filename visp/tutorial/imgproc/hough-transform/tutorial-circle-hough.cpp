#include <iostream>

// ViSP includes
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageDraw.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpTime.h>
#include <visp3/imgproc/vpCircleHoughTransform.h>
#include <visp3/imgproc/vpImgproc.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpVideoReader.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)

#include "drawingHelpers.h"

//! [Enum input]
typedef enum TypeInputImage
{
  FULL_DISKS = 0,
  HALF_DISKS = 1,
  QUARTER_DISKS = 2,
  USER_IMG = 3
}TypeInputImage;

std::string typeInputImageToString(const TypeInputImage &type)
{
  std::string name;
  switch (type) {
  case FULL_DISKS:
    name = "full_disks";
    break;
  case HALF_DISKS:
    name = "half_disks";
    break;
  case QUARTER_DISKS:
    name = "quarter_disks";
    break;
  case USER_IMG:
    name = "path/to/your/image";
  }
  return name;
}
//! [Enum input]

TypeInputImage typeInputImageFromString(const std::string &name)
{
  TypeInputImage type(USER_IMG);
  bool hasFound(false);
  for (unsigned int id = 0; id < USER_IMG && !hasFound; id++) {
    TypeInputImage candidate = (TypeInputImage)id;
    if (name == typeInputImageToString(candidate)) {
      type = candidate;
      hasFound = true;
    }
  }
  return type;
}

std::string getAvailableTypeInputImage(const std::string &prefix = "<", const std::string &sep = " , ", const std::string &suffix = ">")
{
  std::string list(prefix);
  for (unsigned int id = 0; id < USER_IMG; id++) {
    list += typeInputImageToString((TypeInputImage)id) + sep;
  }
  list += typeInputImageToString(USER_IMG) + suffix;
  return list;
}

//! [Draw disks]
void
drawDisk(vpImage<unsigned char> &I, const vpImagePoint &center, const unsigned int &radius,
         const unsigned int &borderColor, const unsigned int &fillingColor, const unsigned int &thickness, const unsigned int &bckg)
  //! [Draw disks]
{
  vpImageDraw::drawCircle(I, center, radius, borderColor, thickness);
  vp::floodFill(I,
    center,
    bckg,
    fillingColor,
    vpImageMorphology::CONNEXITY_4
  );
}

//! [Draw synthetic]
vpImage<unsigned char>
generateImage(const TypeInputImage &inputType)
//! [Draw synthetic]
{
  // // Image dimensions and background
  const unsigned int width = 640;
  const unsigned int height = 480;
  const unsigned int bckg = 0;

  // // Disks parameters
  const unsigned int circleColor = 128;
  const unsigned int circleRadius = 50;
  const unsigned int circleThickness = 1;

  // // Disks position when full circles
  const double topFull = height / 4;
  const double bottomFull = 3 * height / 4;
  const double leftFull = width / 4;
  const double rightFull = 3 * width / 4;

  // // Disks position when Half of circles
  const double topHalf = 1; // m_centerThresh(25) , m_radiusBinSize(10) , m_radiusRatioThresh(50)  , m_mergingDistanceThresh(15) , m_mergingRadiusDiffThresh(1.5 * (double) m_radiusBinSize)
  const double bottomHalf = height - 1;
  const double leftHalf = width / 4;
  const double rightHalf = 3 * width / 4;

  // // Disks position when Quarter of circles
  const double topQuarter = 1; // m_centerThresh(15) , m_radiusBinSize(10) , m_radiusRatioThresh(50)  , m_mergingDistanceThresh(15) , m_mergingRadiusDiffThresh(1.5 * (double) m_radiusBinSize)
  const double bottomQuarter = height - 1;
  const double leftQuarter = 1;
  const double rightQuarter = width - 1;
  vpImage<unsigned char> I_src(height, width, bckg);

  // // Selecting position of the disks depending on their visibility
  double top, left, bottom, right;
  switch (inputType) {
  case FULL_DISKS:
    top = topFull;
    left = leftFull;
    bottom = bottomFull;
    right = rightFull;
    break;
  case HALF_DISKS:
    top = topHalf;
    left = leftHalf;
    bottom = bottomHalf;
    right = rightHalf;
    break;
  case QUARTER_DISKS:
    top = topQuarter;
    left = leftQuarter;
    bottom = bottomQuarter;
    right = rightQuarter;
    break;
  default:
    throw(vpException(vpException::badValue, "Using other type of input than the one that has been implemented to generate disks."));
    break;
  }

  drawDisk(I_src, vpImagePoint(top, left), circleRadius, circleColor, circleColor, circleThickness, bckg);
  drawDisk(I_src, vpImagePoint(top, left), circleRadius / 2, circleColor / 2, circleColor / 2, circleThickness, circleColor);
  drawDisk(I_src, vpImagePoint(bottom, left), circleRadius, circleColor, circleColor, circleThickness, bckg);
  drawDisk(I_src, vpImagePoint(bottom, left), circleRadius / 2, circleColor / 2, circleColor / 2, circleThickness, circleColor);
  drawDisk(I_src, vpImagePoint(top, right), circleRadius, circleColor, circleColor, circleThickness, bckg);
  drawDisk(I_src, vpImagePoint(top, right), circleRadius / 2, circleColor / 2, circleColor / 2, circleThickness, circleColor);
  drawDisk(I_src, vpImagePoint(bottom, right), circleRadius, circleColor, circleColor, circleThickness, bckg);
  drawDisk(I_src, vpImagePoint(bottom, right), circleRadius / 2, circleColor / 2, circleColor / 2, circleThickness, circleColor);

  std::cout << "Done drawing" << std::endl << std::flush;
  return I_src;
}

bool test_detection(const vpImage<unsigned char> &I_src, vpCircleHoughTransform &detector, const int &nbCirclesToDetect, const bool &blockingMode, const bool &displayCanny)
{
  double t0 = vpTime::measureTimeMicros();
  //! [Run detection]
  std::vector<vpImageCircle> detectedCircles = detector.detect(I_src, nbCirclesToDetect);
  //! [Run detection]
  double tF = vpTime::measureTimeMicros();
  std::cout << "Process time = " << (tF - t0) * 0.001 << "ms" << std::endl << std::flush;
  vpImage<vpRGBa> I_disp;
  vpImageConvert::convert(I_src, I_disp);

  unsigned int id = 0;
  std::vector<vpColor> v_colors = { vpColor::red, vpColor::purple, vpColor::orange, vpColor::yellow, vpColor::blue };
  unsigned int idColor = 0;
  //! [Iterate detections]
  for (auto circleCandidate : detectedCircles) {
    vpImageDraw::drawCircle(I_disp, circleCandidate, v_colors[idColor], 2);
    std::cout << "Circle #" << id << ":" << std::endl;
    std::cout << "\tCenter: (" << circleCandidate.getCenter() << ")" << std::endl;
    std::cout << "\tRadius: (" << circleCandidate.getRadius() << ")" << std::endl;
    id++;
    idColor = (idColor + 1) % v_colors.size();
  }
  //! [Iterate detections]

  if (displayCanny) {
    vpImage<unsigned char> edgeMap = detector.getEdgeMap();
    drawingHelpers::display(edgeMap, "Edge map", true);
  }
  return drawingHelpers::display(I_disp, "Detection results", blockingMode);
}

int main(int argc, char **argv)
{
  const std::string def_input(typeInputImageToString(FULL_DISKS));
  const std::string def_jsonFilePath = std::string("");
  const int def_nbCirclesToDetect = -1;
  const int def_gaussianKernelSize = 5;
  const float def_gaussianSigma = 1.f;
  const int def_sobelKernelSize = 3;
#ifdef HAVE_OPENCV_IMGPROC
  const float def_lowerCannyThresh = 50.f;
  const float def_upperCannyThresh = 150.f;
#else
  const float def_lowerCannyThresh = 8.f;
  const float def_upperCannyThresh = 25.f;
#endif
  const int def_nbEdgeFilteringIter = 2;
  const std::pair<int, int> def_centerXlimits = std::pair<int, int>(0, 640);
  const std::pair<int, int> def_centerYlimits = std::pair<int, int>(0, 480);
  const unsigned int def_minRadius = 0;
  const unsigned int def_maxRadius = 1000;
  const int def_dilatationRepet = 1;
  const float def_centerThresh = -1.f;
  const float def_radiusThreshRatio = -1.f;
  const float def_circlePerfectness = 0.85f;
  const float def_centerDistanceThresh = 15.f;
  const float def_radiusDifferenceThresh = 15.f;

  std::string opt_input(def_input);
  std::string opt_jsonFilePath = def_jsonFilePath;
  int opt_nbCirclesToDetect = def_nbCirclesToDetect;
  int opt_gaussianKernelSize = def_gaussianKernelSize;
  float opt_gaussianSigma = def_gaussianSigma;
  int opt_sobelKernelSize = def_sobelKernelSize;
  float opt_lowerCannyThresh = def_lowerCannyThresh;
  float opt_upperCannyThresh = def_upperCannyThresh;
  int opt_nbEdgeFilteringIter = def_nbEdgeFilteringIter;
  std::pair<int, int> opt_centerXlimits = def_centerXlimits;
  std::pair<int, int> opt_centerYlimits = def_centerYlimits;
  unsigned int opt_minRadius = def_minRadius;
  unsigned int opt_maxRadius = def_maxRadius;
  int opt_dilatationRepet = def_dilatationRepet;
  float opt_centerThresh = def_centerThresh;
  float opt_radiusThreshRatio = def_radiusThreshRatio;
  float opt_circlePerfectness = def_circlePerfectness;
  float opt_centerDistanceThresh = def_centerDistanceThresh;
  float opt_radiusDifferenceThresh = def_radiusDifferenceThresh;
  bool opt_displayCanny = false;

  for (int i = 1; i < argc; i++) {
    std::string argName(argv[i]);
    if (argName == "--input" && i + 1 < argc) {
      opt_input = std::string(argv[i + 1]);
      i++;
    }
#ifdef VISP_HAVE_NLOHMANN_JSON
    else if (argName == "--config" && i + 1 < argc) {
      opt_jsonFilePath = std::string(argv[i + 1]);
      i++;
    }
#endif
    else if (argName == "--nb-circles" && i + 1 < argc) {
      opt_nbCirclesToDetect = atoi(argv[i + 1]);
      i++;
    }
    else if (argName == "--gaussian-kernel" && i + 1 < argc) {
      opt_gaussianKernelSize = atoi(argv[i + 1]);
      i++;
    }
    else if (argName == "--gaussian-sigma" && i + 1 < argc) {
      opt_gaussianSigma = static_cast<float>(atof(argv[i + 1]));
      i++;
    }
    else if (argName == "--sobel-kernel" && i + 1 < argc) {
      opt_sobelKernelSize = atoi(argv[i + 1]);
      i++;
    }
    else if (argName == "--canny-thresh" && i + 2 < argc) {
      opt_lowerCannyThresh = static_cast<float>(atof(argv[i + 1]));
      opt_upperCannyThresh = static_cast<float>(atof(argv[i + 2]));
      i += 2;
    }
    else if (argName == "--edge-filter" && i + 1 < argc) {
      opt_nbEdgeFilteringIter = atoi(argv[i + 1]);
      i++;
    }
    else if (argName == "--dilatation-repet" && i + 1 < argc) {
      opt_dilatationRepet = atoi(argv[i + 1]);
      i++;
    }
    else if (argName == "--radius-limits" && i + 2 < argc) {
      opt_minRadius = atoi(argv[i + 1]);
      opt_maxRadius = atoi(argv[i + 2]);
      i += 2;
    }
    else if (argName == "--center-thresh" && i + 1 < argc) {
      opt_centerThresh = static_cast<float>(atof(argv[i + 1]));
      i++;
    }
    else if (argName == "--center-xlim" && i + 2 < argc) {
      opt_centerXlimits = std::pair<int, int>(atoi(argv[i + 1]), atoi(argv[i + 2]));
      i += 2;
    }
    else if (argName == "--center-ylim" && i + 2 < argc) {
      opt_centerYlimits = std::pair<int, int>(atoi(argv[i + 1]), atoi(argv[i + 2]));
      i += 2;
    }
    else if (argName == "--radius-thresh" && i + 1 < argc) {
      opt_radiusThreshRatio = static_cast<float>(atof(argv[i + 1]));
      i++;
    }
    else if (argName == "--circle-perfectness" && i + 1 < argc) {
      opt_circlePerfectness = static_cast<float>(atof(argv[i + 1]));
      i++;
    }
    else if (argName == "--merging-thresh" && i + 2 < argc) {
      opt_centerDistanceThresh = static_cast<float>(atof(argv[i + 1]));
      opt_radiusDifferenceThresh = static_cast<float>(atof(argv[i + 2]));
      i += 2;
    }
    else if (argName == "--display-edge-map") {
      opt_displayCanny = true;
    }
    else if (argName == "--help" || argName == "-h") {
      std::cout << "NAME" << std::endl;
      std::cout << "\t" << argv[0] << "  Test program for the home-made Hough Circle Detection algorithm" << std::endl
        << std::endl;
      std::cout << "SYNOPSIS" << std::endl;
      std::cout << "\t" << argv[0]
        << "\t [--input " << getAvailableTypeInputImage() << "]" << std::endl
#ifdef VISP_HAVE_NLOHMANN_JSON
        << "\t [--config <path/to/json/file>] (default: " << (def_jsonFilePath.empty() ? "unused" : def_jsonFilePath) << ")" << std::endl
#endif
        << "\t [--nb-circles <number-circles-to-detect>] (default: " << def_nbCirclesToDetect << ")" << std::endl
        << "\t [--gaussian-kernel <kernel-size>] (default: " << def_gaussianKernelSize << ")" << std::endl
        << "\t [--gaussian-sigma <stddev>] (default: " << def_gaussianSigma << ")" << std::endl
        << "\t [--sobel-kernel <kernel-size>] (default: " << def_sobelKernelSize << ")" << std::endl
        << "\t [--canny-thresh <lower-canny-thresh upper-canny-thresh>] (default: " << def_lowerCannyThresh << " ; " << def_upperCannyThresh << ")" << std::endl
        << "\t [--edge-filter <nb-iter>] (default: " << def_nbEdgeFilteringIter << ")" << std::endl
        << "\t [--radius-limits <radius-min> <radius-max>] (default: min = " << def_minRadius << ", max = " << def_maxRadius << ")" << std::endl
        << "\t [--dilatation-repet <nb-repetitions>] (default: " << def_dilatationRepet << ")" << std::endl
        << "\t [--center-thresh <center-detection-threshold>] (default: " << (def_centerThresh < 0 ? "auto" : std::to_string(def_centerThresh)) << ")" << std::endl
        << "\t [--center-xlim <center-horizontal-min center-horizontal-max>] (default: " << def_centerXlimits.first << " , " << def_centerXlimits.second  << ")" << std::endl
        << "\t [--center-ylim <center-vertical-min center-vertical-max>] (default: " << def_centerYlimits.first << " , " << def_centerYlimits.second  << ")" << std::endl
        << "\t [--radius-thresh <radius-detection-threshold>] (default: " << (def_radiusThreshRatio < 0 ? "auto" : std::to_string(def_radiusThreshRatio)) << ")" << std::endl
        << "\t [--circle-perfectness <circle-perfectness-threshold>] (default: " << def_radiusThreshRatio << ")" << std::endl
        << "\t [--merging-thresh <center-distance-thresh> <radius-difference-thresh>] (default: centers distance threshold = " << def_centerDistanceThresh << ", radius difference threshold = " << def_radiusDifferenceThresh << ")" << std::endl
        << "\t [--display-edge-map]" << std::endl
        << "\t [--help, -h]" << std::endl
        << std::endl;

      std::cout << "DESCRIPTION" << std::endl
        << "\t--input" << std::endl
        << "\t\tPermit to choose the type of input of the Hough Circle Algorithm" << std::endl
        << "\t\tDefault: " << def_input << std::endl
        << std::endl
#ifdef VISP_HAVE_NLOHMANN_JSON
        << "\t--config" << std::endl
        << "\t\tPermit to configure the Hough Circle Algorithm using a JSON file." << std::endl
        << "\t\tDefault: " << (def_jsonFilePath.empty() ? "unused" : def_jsonFilePath) << std::endl
        << std::endl
#endif
        << "\t--nb-circles" << std::endl
        << "\t\tPermit to choose the number of circles we want to detect in the image" << std::endl
        << "\t\tThe results will be the circles having the greatest number of votes." << std::endl
        << "\t\tDefault: " << def_nbCirclesToDetect << std::endl
        << std::endl
        << "\t--gaussian-kernel" << std::endl
        << "\t\tPermit to set the size of the Gaussian filter used to smooth the input image and compute its gradients." << std::endl
        << "\t\tMust be an odd value." << std::endl
        << "\t\tDefault: " << def_gaussianKernelSize << std::endl
        << std::endl
        << "\t--gaussian-sigma" << std::endl
        << "\t\tPermit to set the standard deviation of the Gaussian filter." << std::endl
        << "\t\tMust be a positive value." << std::endl
        << "\t\tDefault: " << def_gaussianSigma << std::endl
        << std::endl
        << "\t--canny-thresh" << std::endl
        << "\t\tPermit to set the lower and upper thresholds of the Canny edge detector." << std::endl
        << "\t\tIf a value is negative, it will be automatically computed." << std::endl
        << "\t\tDefault: " << def_upperCannyThresh << std::endl
        << std::endl
        << "\t--edge-filter" << std::endl
        << "\t\tPermit to set the number of iteration of 8-neighbor filter iterations of the result of the Canny edge detector." << std::endl
        << "\t\tIf negative, no filtering is performed." << std::endl
        << "\t\tDefault: " << def_nbEdgeFilteringIter << std::endl
        << std::endl
        << "\t--radius-limits" << std::endl
        << "\t\tPermit to set the minimum and maximum radii of the circles we are looking for." << std::endl
        << "\t\tDefault: min = " << def_minRadius << ", max = " << def_maxRadius << std::endl
        << std::endl
        << "\t--dilatation-repet" << std::endl
        << "\t\tPermit to set the number of iterations of the dilatation operation used to detect the maxima of the centers votes." << std::endl
        << "\t\tMinimum tolerated value is 1." << std::endl
        << "\t\tDefault: " << def_dilatationRepet << std::endl
        << std::endl
        << "\t--center-thresh" << std::endl
        << "\t\tPermit to set the minimum number of votes a point must reach to be considered as a center candidate." << std::endl
        << "\t\tIf the input is a real image, must be a positive value." << std::endl
        << "\t\tOtherwise, if the input is a synthetic image and the value is negative, a fine-tuned value will be used." << std::endl
        << "\t\tDefault: " << (def_centerThresh < 0 ? "auto" : std::to_string(def_centerThresh)) << std::endl
        << std::endl
        << "\t--center-xlim" << std::endl
        << "\t\tPermit to set the minimum and maximum horizontal position to be considered as a center candidate." << std::endl
        << "\t\tThe search area is limited to [-maxRadius; +image.width + maxRadius]." << std::endl
        << "\t\tDefault: " << def_centerXlimits.first << " , " << def_centerXlimits.second << std::endl
        << std::endl
        << "\t--center-ylim" << std::endl
        << "\t\tPermit to set the minimum and maximum vertical position to be considered as a center candidate." << std::endl
        << "\t\tThe search area is limited to [-maxRadius; +image.height + maxRadius]." << std::endl
        << "\t\tDefault: " << def_centerYlimits.first << " , " << def_centerYlimits.second << std::endl
        << std::endl
        << "\t--radius-thresh" << std::endl
        << "\t\tPermit to to set the minimum number of votes per radian a radius must reach to be considered as a circle candidate a given pair (center candidate, radius candidate)." << std::endl
        << "\t\tDefault: " << (def_radiusThreshRatio < 0 ? "auto" : std::to_string(def_radiusThreshRatio)) << std::endl
        << std::endl
        << "\t--circle-perfectness" << std::endl
        << "\t\tPermit to set the set the circle perfectness threshold." << std::endl
        << "\t\tThis parameter is used during the radius candidates computation." << std::endl
        << "\t\tThe scalar product radius RC_ij . gradient(Ep_j) >=  m_circlePerfectness * || RC_ij || * || gradient(Ep_j) || to add a vote for the radius RC_ij." << std::endl
        << "\t\tDefault: " << def_circlePerfectness << std::endl
        << std::endl
        << "\t--merging-thresh" << std::endl
        << "\t\tPermit to set the thresholds used during the merging stage of the algorithm." << std::endl
        << "\t\tThe center distance threshold indicates the maximum distance the centers can be in order to be merged." << std::endl
        << "\t\tThe radius difference threshold indicates the maximum absolute difference between the two circle candidates in order to be merged." << std::endl
        << "\t\tTwo circle candidates must met these two conditions in order to be merged together." << std::endl
        << "\t\tDefault: centers distance threshold = " << def_centerDistanceThresh << ", radius difference threshold = " << def_radiusDifferenceThresh << std::endl
        << "\t--display-edge-map" << std::endl
        << "\t\tPermit to display the edge map used to detect the circles" << std::endl
        << "\t\tDefault: off" << std::endl
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  if (opt_centerThresh < 0 && opt_jsonFilePath.empty()) {
    // The user asked to use the parameter value that has been fine-tuned
    TypeInputImage inputType = typeInputImageFromString(opt_input);
    switch (inputType) {
    case TypeInputImage::FULL_DISKS:
#ifdef HAVE_OPENCV_IMGPROC
      opt_centerThresh = 100.;
#else
      opt_centerThresh = 75.;
#endif
      break;
    case TypeInputImage::HALF_DISKS:
#ifdef HAVE_OPENCV_IMGPROC
      opt_centerThresh = 50.;
#else
      opt_centerThresh = 25.;
#endif
      break;
    case TypeInputImage::QUARTER_DISKS:
#ifdef HAVE_OPENCV_IMGPROC
      opt_centerThresh = 25.;
#else
      opt_centerThresh = 15.;
#endif
      break;
    default:
      throw(vpException(vpException::badValue, "Missing center threshold value to use with actual pictures as input. See the help for more information."));
    }
  }

  if (opt_radiusThreshRatio < 0 && opt_jsonFilePath.empty()) {
    // The user asked to use the parameter value that has been fine-tuned
    TypeInputImage inputType = typeInputImageFromString(opt_input);
    switch (inputType) {
    case TypeInputImage::FULL_DISKS:
#ifdef HAVE_OPENCV_IMGPROC
      opt_radiusThreshRatio = 5.;
#else
      opt_radiusThreshRatio = 1.;
#endif
      break;
    case TypeInputImage::HALF_DISKS:
      opt_radiusThreshRatio = 2.;
      break;
    case TypeInputImage::QUARTER_DISKS:
      opt_radiusThreshRatio = 1.;
      break;
    default:
      throw(vpException(vpException::badValue, "Missing radius threshold value to use with actual pictures as input. See the help for more information."));
    }
  }

  //! [Algo params]
  vpCircleHoughTransform::vpCircleHoughTransformParameters
    algoParams(opt_gaussianKernelSize
      , opt_gaussianSigma
      , opt_sobelKernelSize
      , opt_lowerCannyThresh
      , opt_upperCannyThresh
      , opt_nbEdgeFilteringIter
      , opt_centerXlimits
      , opt_centerYlimits
      , opt_minRadius
      , opt_maxRadius
      , opt_dilatationRepet
      , opt_centerThresh
      , opt_radiusThreshRatio
      , opt_circlePerfectness
      , opt_centerDistanceThresh
      , opt_radiusDifferenceThresh
    );
  //! [Algo params]

  //! [Algo init]
  vpCircleHoughTransform detector;
  if (opt_jsonFilePath.empty()) {
    std::cout << "Initializing detector from the program arguments [...]" << std::endl;
    detector.init(algoParams);
  }
  else {
#ifdef VISP_HAVE_NLOHMANN_JSON
    std::cout << "Initializing detector from JSON file \"" << opt_jsonFilePath << "\", some of the program arguments will be ignored [...]" << std::endl;
    detector.initFromJSON(opt_jsonFilePath);
#else
    throw(vpException(vpException::functionNotImplementedError, "You must install nlohmann JSON library to use this feature, see https://visp-doc.inria.fr/doxygen/visp-daily/supported-third-parties.html#soft_tool_json for more information."));
#endif
  }
  //! [Algo init]
  std::cout << detector;

  vpImage<unsigned char> I_src;
  TypeInputImage inputType = typeInputImageFromString(opt_input);
  if (inputType == USER_IMG) {
    //! [Manage video]
    if (opt_input.find("%") != std::string::npos) {
      // The user wants to read a sequence of images from different files
      bool hasToContinue = true;
      vpVideoReader g;
      g.setFileName(opt_input);
      g.open(I_src);
      while (!g.end() && hasToContinue) {
        g.acquire(I_src);
        hasToContinue = test_detection(I_src, detector, opt_nbCirclesToDetect, false, opt_displayCanny);
        vpTime::wait(40);
      }
    }
    //! [Manage video]
    else {
      //! [Manage single image]
      // Check if opt_input exists
      if (!vpIoTools::checkFilename(opt_input)) {
        throw(vpException(vpException::ioError, "Input file \"" + opt_input + "\" does not exist !"));
      }
      // Read the image and perform detection on it
      vpImageIo::read(I_src, opt_input);
      test_detection(I_src, detector, opt_nbCirclesToDetect, true, opt_displayCanny);
      //! [Manage single image]
    }
  }
  else {
    //! [Manage synthetic image]
    I_src = generateImage(inputType);
    test_detection(I_src, detector, opt_nbCirclesToDetect, true, opt_displayCanny);
    //! [Manage synthetic image]
  }
  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "This tutorial needs to be build at least with cxx 11 standard!" << std::endl;
  return EXIT_SUCCESS;
}
#endif

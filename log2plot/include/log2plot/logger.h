#ifndef LOG2PLOT_LOGGER_H_
#define LOG2PLOT_LOGGER_H_

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <sstream>
#include <assert.h>
#include <optional>

#include <log2plot/defines.h>
#include <log2plot/container.h>
#include <log2plot/shape.h>

namespace log2plot
{

/// Close all files and call the Python interpreter on the given script path
inline void closePreviousPlots()
{
  [[maybe_unused]] const auto out = system("killall log2plot -q");
}

const double nan = std::nan("");

/// set some values of this vector to not a number
template<class T>
void setNaN(T& v, uint start = 0, uint end = 0)
{
  if(start == 0 && end == 0)
    end = v.size();
  for(uint i = start; i < end; ++i)
    v.operator[](i) = nan;
}

class Logger
{
protected:

  std::string file_path;

  // related to high-level functions
  unsigned int buff, subsamp, buff_count = 0, iter_count = 0;
  double * time{};
  std::string time_unit = "s";
  std::vector<double> time_buff;
  std::vector<double> steps, steps_timed;
  std::vector<std::vector<uint>> grouped_vars;

  // logger variables and pointer to the last entered
  std::vector<std::unique_ptr<GenericContainer> > logged_vars;
  GenericContainer* last{};

  bool first_update = true;
  uint nb_fixed_objects = 0;

  // write initial information in latest saved variable
  void writeInitialInfo(const std::string &name, const std::string &legend, const std::string &xlabel, const std::string &ylabel, const bool &keep_file);

  // build explicit legend from implicit
  static std::string buildLegend(const std::string &legend, const unsigned int len);

  // calls python to plot this / these files
  void plotFiles(const std::string &script_path, const std::vector<std::string> &files, bool verbose, bool display);

  //template <class Point>
  void showShape(const std::string &key, const Shape &shape, const Surface & surface = PointCloud())
  {
    last->writeInfo(key);
    last->writeInfo(shape.infos());
    if(const auto surface_info{surface.infos()}; !surface_info.empty())
    {
      last->writeInfo("    surface");
      last->writeInfo(surface_info);
    }
  }

public:
  // constructor with default values
  Logger(const std::string &_file_path = "", unsigned int _buffer = 10, unsigned int _subsampling = 1)
    :  file_path(_file_path), buff(_buffer), subsamp(_subsampling)
  {
    logged_vars.clear();
    steps.clear();
    steps_timed.clear();
  }

  ~Logger()
  {
    for(auto &var: logged_vars)
      var->close(steps, steps_timed);
  }

  // **** Parameter methods ****

  /// Set pointer to time variable
  inline void setTime(double &t, const std::string &unit = "s") {time = &t;time_unit = unit;}
  /// number of calls before flushing to file
  inline void setBuffer(unsigned int b) {buff = b;}
  /// if any subsampling
  inline void setSubSampling(unsigned int s) {subsamp = s;}
  /// path to files to be saved, can include any prefix for files
  inline void setSavePath(const std::string &file_path) {this->file_path = file_path;}

  // **** Functions to add new variables to be saved ****

  /// Save iteration-based vector
  template<class Vector>
  inline void save(Vector &v, const std::string &name, const std::string &legend, const std::string &ylabel, bool keep_file = true)
  {
    // add this to logged variables
    logged_vars.push_back(std::make_unique<Container<Vector>>(LogType::ITERATION, v));
    // and write initial info
    writeInitialInfo(name, buildLegend(legend, v.size()), "iterations", ylabel, keep_file);
  }

  /// Save time-based vector
  template<class Vector>
  inline void saveTimed(Vector &v, const std::string &name, const std::string &legend, const std::string &ylabel, bool keep_file = true)
  {
    // add this to logged variables
    logged_vars.push_back(std::make_unique<Container<Vector>>(LogType::TIME, v));
    // and write initial info
    writeInitialInfo(name, buildLegend(legend, v.size()), "time [" + time_unit + "]", ylabel, keep_file);
  }

  /// Save XY vector
  template<class Vector>
  inline void saveXY(Vector &v, const std::string &name, const std::string &legend,
                     const std::string &xlabel="X", const std::string &ylabel="Y", bool keep_file = true)
  {
    // add this to logged variables
    logged_vars.push_back(std::make_unique<Container<Vector>>(LogType::XY, v));
    // and write initial info
    writeInitialInfo(name, buildLegend(legend, v.size()/2), xlabel, ylabel, keep_file);
  }

  /// Save 3D pose or position
  template<class Vector>
  inline void save3Dpose(Vector &v, const std::string &name, const std::string &legend = "", bool invert = false, bool keep_file = true)
  {
    assert(v.size() == 3 || v.size() == 6);
    // add this to logged variables
    logged_vars.push_back(std::make_unique<Container<Vector>>(LogType::POSE, v));
    // and write initial info
    writeInitialInfo(name, "["+legend+"]", "", "", keep_file);
    if(invert)
      last->writeInfo("invertPose", "True");
  }

  /// save timed XY, only useful with a video output
  template<class Vector>
  inline void saveTimedXY(Vector &xy, const std::string &name, const std::string &legend,
                          const std::string &xlabel="X", const std::string &ylabel="Y", bool keep_file = true)
  {
    assert(xy.size() % 2 == 0);
    logged_vars.push_back(std::make_unique<Container<Vector>>(LogType::TIMED_XY, xy));
    writeInitialInfo(name, buildLegend(legend, 1), xlabel, ylabel, keep_file);
  }

  // **** End functions for new variables ****

  // tells to plot the next n added variables in the same plot
  void regroupNext(uint n)
  {
    const auto idx = static_cast<uint>(logged_vars.size());
    grouped_vars.push_back({});
    for(uint i = 0; i < n; ++i)
      grouped_vars.back().push_back(idx + i);
  }

  // **** Functions to specify metadata for the last registered variable ****

  // Units
  void setUnits(const std::string &units) {last->writeInfo("units", units);}
  // Line types
  void setLineType(const std::string &lineType) {last->writeInfo("lineType", lineType);}

  // Dashed steps
  void setSteps(std::vector<double> _steps)
  {
    last->setSteps(_steps);
  }

  // standard arguments
  void setPlotArgs(std::string args)
  {    
    last->writeInfo("args", args);
  }
  void showMovingShape(const Shape &shape, const Surface & surface = PointCloud())
  {
    showShape("movingObject", shape, surface);
  }

  inline void showFixedShape(const Shape &shape, const Surface & surface = PointCloud())
  {
    showShape("fixedObject"+std::to_string(++nb_fixed_objects), shape, surface);
  }

  template <class Point = std::vector<double>>
  inline void showFixedShape(const std::vector<Point> &points, const std::string &color = "", const std::string &legend = "", const Fully graph = Fully::Disconnected)
  {
    showFixedShape(Shape(points, color, legend, graph));
  }

#ifdef LOG2PLOT_WITH_DEPRECATED
  /// [deprecated, use Camera] 3D plot: show camera
  void showMovingCamera(const std::vector<double> &desired_pose = {}, const double &x = 1.5, const double &y = 1, const double &z = 4);
  /// [deprecated, use Shape] 3D plot: custom object with a (nx3) matrix
  virtual void showMovingObject(const std::vector<std::vector<double>> &M, const std::string &graph, const std::vector<double> &desired_pose = {});
  /// [deprecated, use Shape] 3D plot: fixed 2D-rectangle on Z=0
  void showFixedRectangle(const double &xm, const double &ym, const double &xM, const double &yM, const std::string &color = "", const std::string &legend = "");
  /// [deprecated, use Shape] any fixed object from a list of coordinates (related to object frame)
  template <class Point>
  void showFixedObject(const std::vector<Point> &M, const std::string &graph, const std::string &color = "", const std::string &legend = "")
  {
    std::stringstream ss;
    ss << "fixedObject" << ++nb_fixed_objects;
    last->writeInfo(ss.str(), "");
    last->writeInfo(Yaml("nodes", M, 1));
      last->writeInfo("    graph", graph);
    if(!color.empty())
    {
      if(color[0] != '\'')
        last->writeInfo("    color", "'" + color + "'");
      else
        last->writeInfo("    color", color);
    }
    if(!legend.empty())
      last->writeInfo("    legend", legend);
    last->flush();
  }
  /// [deprecated, use Shape]
  virtual void showFixedObject(const std::vector<std::vector<double>> &M, const std::string &graph, const std::string &color = "", const std::string &legend = "")
  {
    showFixedObject<std::vector<double>>(M, graph, color, legend);
  }
#endif

  // **** End metadata functions ****


  // Add a current moment for all iteration or time-based plots
  void writeStep()
  {
    steps.push_back(iter_count / subsamp);
    if(time)
      steps_timed.push_back(*time);
  }

  // add several time-steps for all iteration or time-based plots
  void writeStepsAll(const std::vector<double> &iterations, const std::vector<double> &times = {})
  {
    steps = iterations;
    steps_timed = times;
  }

  // Updates all saved variables
  virtual void update(const bool &flush = false);

  void plot(const std::string &script_path, bool verbose = false, bool display = true);
  void plot(bool verbose = false, bool display = true);

  // Close all files and call interpreter without actual plotting
  void generateFigures(const std::string &script_path, bool verbose = false)
  {
    plot(script_path, verbose, false);
  }
  void generateFigures(bool verbose = false)
  {
    plot(verbose, false);
  }
};
}

#endif /* LOG2PLOTLOGGER_H_ */

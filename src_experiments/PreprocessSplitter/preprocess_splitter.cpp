#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <sstream>

std::string filename;

void expect_string(std::string s, std::string expected)
{
  if (s != expected)
  {
    std::cout << "Expected string \"" << expected << "\", instead got \"" << s << "\", filename: " << filename << std::endl;
    exit(EXIT_FAILURE);
  }
}

void expect_next_string(std::ifstream &f, std::string expected)
{
  std::string next;
  f >> next;
  expect_string(next, expected);
}

void expect_next_strings(std::ifstream &f, std::vector<char *> strs)
{
  for (char * s : strs)
    expect_next_string(f, std::string(s));
}

void expect_next_strings(std::ifstream &f, std::string str)
{
  std::stringstream stream(str);
  while (!stream.eof())
  {
    std::string a;
    stream >> a;
    if (stream.fail())
      break;
    //std::cout << a << "\t\t" << str << std::endl;
    expect_next_string(f,a);
  }
}

void expect_next_strings(std::ifstream &f, char const * str)
{
  expect_next_strings(f, std::string(str));
}

template <class T>
void output_to_file(T value, std::string extension)
{
  std::ofstream f(filename + extension);
  f << value << std::endl;
  f.close();
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << "usage: " << argv[0] << " " << "filename" << std::endl;
    return -1;
  }

  filename = std::string(argv[1]);
  //std::cout << "Filename: " << filename << std::endl;
  std::ifstream f(filename);

  int num_corners;
  double corner_compute_time;
  double smoothening_straight_time;
  int corners_after_smoothening;
  double smoothed_graph_time;
  double nearby_corners_found_time;
  double complete_corner_graph_time;
  std::string degree_one_ratio;
  double degree_two_time;
  double relevant_points_time;
  int num_corners_with_relevant;
  int num_corners_without_relevant;
  double corners_with_relevant_time;
  int num_corners_with_next;
  int num_corners_without_next;
  double corners_with_next_time;
  std::string no_next_corner_ratio;
  double splittable_corners_time;
  double geometric_containers_time;
  double geometric_container_convert_time;
  double first_corner_push_time;
  double total_preprocessing_time;

  int num_nearby_min;
  int num_nearby_qt1;
  int num_nearby_med;
  int num_nearby_qt2;
  int num_nearby_max;
  double num_nearby_mean;
  int num_nearby_mode;

  int num_nearby_with_relevant_min;
  int num_nearby_with_relevant_qt1;
  int num_nearby_with_relevant_med;
  int num_nearby_with_relevant_qt2;
  int num_nearby_with_relevant_max;
  double num_nearby_with_relevant_mean;
  int num_nearby_with_relevant_mode;

  int num_nearby_with_next_min;
  int num_nearby_with_next_qt1;
  int num_nearby_with_next_med;
  int num_nearby_with_next_qt2;
  int num_nearby_with_next_max;
  double num_nearby_with_next_mean;
  int num_nearby_with_next_mode;

  int num_corners_saved;
  int num_nearby_corner_indices_with_relevant_saved;
  int num_nearby_corner_indices_with_next_saved;
  int num_exact_distances_saved;
  int first_corners_saved;
  int bounds_saved;
  double preprocessed_data_save_time;

  std::string junk;

  expect_next_strings(f, "Computing corners Number of corners:");
  f >> num_corners;
  expect_next_strings(f, "Corners computed in");
  f >> corner_compute_time;
  expect_next_strings(f, "Computing smoothed graph Smoothening straight:");
  std::getline(f, junk);
  expect_next_strings(f, "Smoothening straight took");
  f >> smoothening_straight_time;
  expect_next_strings(f, "Number of corners after smoothening:");
  f >> corners_after_smoothening;
  expect_next_strings(f, "Smoothed graph computed in");
  f >> smoothed_graph_time;
  expect_next_strings(f, "Finding nearby corners 0,");
  std::getline(f, junk);
  expect_next_strings(f, "Nearby corners found in");
  f >> nearby_corners_found_time;
  expect_next_strings(f, "Finding complete corner graph 0,");
  std::getline(f,junk);
  expect_next_strings(f, "0,");
  std::getline(f,junk);
  expect_next_strings(f, "Complete corner graph found in");
  f >> complete_corner_graph_time;
  expect_next_strings(f, "Finding degree two corners");
  f >> degree_one_ratio;
  expect_next_strings(f, "indices are degree one Degree two corners found in");
  f >> degree_two_time;
  expect_next_strings(f, "Finding relevant points Computing incoming to outgoing divdirections:");
  std::getline(f,junk);
  expect_next_strings(f, "Computing outgoing relevant points:");
  std::getline(f,junk);
  expect_next_strings(f, "Computing outgoing relevant corners:");
  std::getline(f,junk);
  expect_next_strings(f, "Relevant points found in");
  f >> relevant_points_time;
  expect_next_strings(f, "Finding nearby corners with relevant");
  f >> num_corners_with_relevant;
  expect_next_strings(f, "with relevant,");
  f >> num_corners_without_relevant;
  expect_next_strings(f, "without relevant Nearby corners with relevant found in");
  f >> corners_with_relevant_time;
  expect_next_strings(f, "Finding nearby corners with next");
  f >> num_corners_with_next;
  expect_next_strings(f, "with next,");
  f >> num_corners_without_next;
  expect_next_strings(f, "without next");
  expect_next_strings(f, "Nearby corners with next found in");
  f >> corners_with_next_time;
  expect_next_strings(f, "Finding splittable corners");
  f >> no_next_corner_ratio;
  expect_next_strings(f, "corners have no next corner in important directions Splittable corners found in");
  f >> splittable_corners_time;
  expect_next_strings(f, "Finding geometric containers 0,");
  std::getline(f,junk);
  expect_next_strings(f, "Geometric containers found in");
  f >> geometric_containers_time;
  expect_next_strings(f, "Converting geometric containers Converting outgoing geometric containers to incoming geometric containers:");
  std::getline(f,junk);
  expect_next_strings(f, "Geometric containers converted in");
  f >> geometric_container_convert_time;
  expect_next_strings(f, "Pushing first corners 0,");
  std::getline(f,junk);
  expect_next_strings(f, "First corners pushed in");
  f >> first_corner_push_time;
  expect_next_strings(f, "Preprocessing complete Total preprocessing time:");
  f >> total_preprocessing_time;
  expect_next_strings(f, "Histogram for number of corners relevant to corner in any");
  std::getline(f,junk); // read "direction:" and newline
  std::getline(f,junk); // read histogram

  expect_next_strings(f, "Num nearby corners histogram:");
  std::getline(f,junk);
  expect_next_strings(f, "Num nearby corners stats:");
  expect_next_strings(f, "Min:");
  f >> num_nearby_min;
  expect_next_strings(f, "Qt1:");
  f >> num_nearby_qt1;
  expect_next_strings(f, "Median:");
  f >> num_nearby_med;
  expect_next_strings(f, "Qt2:");
  f >> num_nearby_qt2;
  expect_next_strings(f, "Max:");
  f >> num_nearby_max;
  expect_next_strings(f, "Mean:");
  f >> num_nearby_mean;
  expect_next_strings(f, "Mode:");
  f >> num_nearby_mode;

  expect_next_strings(f, "Num nearby corners with relevant histogram:");
  std::getline(f,junk);
  expect_next_strings(f, "Num nearby corners with relevant stats:");
  expect_next_strings(f, "Min:");
  f >> num_nearby_with_relevant_min;
  expect_next_strings(f, "Qt1:");
  f >> num_nearby_with_relevant_qt1;
  expect_next_strings(f, "Median:");
  f >> num_nearby_with_relevant_med;
  expect_next_strings(f, "Qt2:");
  f >> num_nearby_with_relevant_qt2;
  expect_next_strings(f, "Max:");
  f >> num_nearby_with_relevant_max;
  expect_next_strings(f, "Mean:");
  f >> num_nearby_with_relevant_mean;
  expect_next_strings(f, "Mode:");
  f >> num_nearby_with_relevant_mode;

  expect_next_strings(f, "Num nearby corners with next histogram:");
  std::getline(f,junk);
  expect_next_strings(f, "Num nearby corners with next stats:");
  expect_next_strings(f, "Min:");
  f >> num_nearby_with_next_min;
  expect_next_strings(f, "Qt1:");
  f >> num_nearby_with_next_qt1;
  expect_next_strings(f, "Median:");
  f >> num_nearby_with_next_med;
  expect_next_strings(f, "Qt2:");
  f >> num_nearby_with_next_qt2;
  expect_next_strings(f, "Max:");
  f >> num_nearby_with_next_max;
  expect_next_strings(f, "Mean:");
  f >> num_nearby_with_next_mean;
  expect_next_strings(f, "Mode:");
  f >> num_nearby_with_next_mode;  

  expect_next_strings(f, "Saving preprocessed data");
  f >> num_corners_saved;
  expect_next_strings(f, "corners saved");
  f >> num_nearby_corner_indices_with_relevant_saved;
  expect_next_strings(f, "nearby corner indices with relevant saved");
  f >> num_nearby_corner_indices_with_next_saved;
  expect_next_strings(f, "nearby corner indices with next saved");
  f >> num_exact_distances_saved;
  expect_next_strings(f, "exact distances saved");
  f >> first_corners_saved;
  expect_next_strings(f, "first corners saved");
  f >> bounds_saved;
  expect_next_strings(f, "bounds saved Preprocessed data saved in");
  f >> preprocessed_data_save_time;

  // VVVV double corner_compute_time;
  // VVVV double smoothening_straight_time;
  // BBBB double smoothed_graph_time;
  // VVVV double nearby_corners_found_time;
  // VVVV double complete_corner_graph_time;
  // XXXX double degree_two_time;
  // VVVV double relevant_points_time;
  // VVVV double corners_with_relevant_time;
  // VVVV double corners_with_next_time;
  // XXXX double splittable_corners_time;
  // VVVV double geometric_containers_time;
  // VVVV double geometric_container_convert_time;
  // VVVV double first_corner_push_time;

  output_to_file(num_corners, ".stats.num_corners");
  output_to_file(corner_compute_time, ".time.corner");
  output_to_file(complete_corner_graph_time, ".time.ccg");
  output_to_file(smoothed_graph_time, ".time.smooth_graph");
  output_to_file(nearby_corners_found_time, ".time.nearby_corners");
  output_to_file(relevant_points_time + corners_with_relevant_time + corners_with_next_time, ".time.relevant");
  output_to_file(geometric_containers_time + geometric_container_convert_time, ".time.geometric_container");
  output_to_file(first_corner_push_time, ".time.push_corner");

  output_to_file(num_nearby_min, ".nearby.min");
  output_to_file(num_nearby_qt1, ".nearby.qt1");
  output_to_file(num_nearby_med, ".nearby.med");
  output_to_file(num_nearby_qt2, ".nearby.qt2");
  output_to_file(num_nearby_max, ".nearby.max");
  output_to_file(num_nearby_mean, ".nearby.mean");
  output_to_file(num_nearby_mode, ".nearby.mode");

  output_to_file(num_nearby_with_relevant_min, ".with_relevant.min");
  output_to_file(num_nearby_with_relevant_qt1, ".with_relevant.qt1");
  output_to_file(num_nearby_with_relevant_med, ".with_relevant.med");
  output_to_file(num_nearby_with_relevant_qt2, ".with_relevant.qt2");
  output_to_file(num_nearby_with_relevant_max, ".with_relevant.max");
  output_to_file(num_nearby_with_relevant_mean, ".with_relevant.mean");
  output_to_file(num_nearby_with_relevant_mode, ".with_relevant.mode");

  output_to_file(num_nearby_with_next_min, ".with_next.min");
  output_to_file(num_nearby_with_next_qt1, ".with_next.qt1");
  output_to_file(num_nearby_with_next_med, ".with_next.med");
  output_to_file(num_nearby_with_next_qt2, ".with_next.qt2");
  output_to_file(num_nearby_with_next_max, ".with_next.max");
  output_to_file(num_nearby_with_next_mean, ".with_next.mean");
  output_to_file(num_nearby_with_next_mode, ".with_next.mode");

  return 0;
}

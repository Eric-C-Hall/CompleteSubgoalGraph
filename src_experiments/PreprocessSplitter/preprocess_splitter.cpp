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

  std::ofstream num_corner_f(filename + ".num_corners");
  num_corner_f << num_corners << std::endl;

  std::ofstream corner_f(filename + ".corner");
  corner_f << corner_compute_time << std::endl;
  corner_f.close();

  std::ofstream ccg_f(filename + ".ccg");
  ccg_f << complete_corner_graph_time << std::endl;
  ccg_f.close();

  std::ofstream smooth_graph_f(filename + ".smooth_graph");
  smooth_graph_f << smoothed_graph_time << std::endl;
  smooth_graph_f.close();

  std::ofstream nearby_corner_f(filename + ".nearby_corners");
  nearby_corner_f << nearby_corners_found_time << std::endl;
  nearby_corner_f.close();

  std::ofstream relevant_f(filename + ".relevant");
  relevant_f << relevant_points_time + corners_with_relevant_time + corners_with_next_time << std::endl;
  relevant_f.close();

  std::ofstream geometric_container_f(filename + ".geometric_container");
  geometric_container_f << geometric_containers_time + geometric_container_convert_time << std::endl;
  geometric_container_f.close();

  std::ofstream push_corner_f(filename + ".push_corner");
  push_corner_f << first_corner_push_time << std::endl;
  push_corner_f.close();

  return 0;
}

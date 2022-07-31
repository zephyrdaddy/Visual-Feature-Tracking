#include <ctime>
#include <string>
#include <iostream>
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/features2d.hpp"
#include "opencv4/opencv2/imgproc.hpp"


std::string get_greet(const std::string& who) {
  return "Hello " + who;
}

void print_localtime() {
  std::time_t result = std::time(nullptr);
  std::cout << std::asctime(std::localtime(&result));
}

int main(int argc, char** argv) {
  std::string who = "world";
  if (argc > 1) {
    who = argv[1];
  }
  std::cout << "Bazel docker works\n";
  std::cout << get_greet(who) << std::endl;
  print_localtime();
	
  cv::Mat src;

      
	return 0;
}

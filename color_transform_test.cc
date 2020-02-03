#include "color_transform.hpp"
#include "file_interface.hpp"

int main(int argc, char** argv) {
  FileInterface file_interface;
  ColorTransform color_transform;
  color_transform.setInterface(&file_interface);
  color_transform.process();
  return 0;
}
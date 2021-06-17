#include <roseus_bt/package_generator.h>

int main(int argc, char** argv)
{
  RoseusBT::PackageGenerator pg(argv[1], argv[2], "test_node", "mynode", "Guilherme Affonso");
  pg.write_all_files();

  return 0;
}
  

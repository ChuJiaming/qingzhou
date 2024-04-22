#include <qingzhou_nav/qingzhou_nav.h>
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "qingzhou_nav_node");
  ros::NodeHandle nh;
  qingzhou_nav node(nh);
  node.run();
}

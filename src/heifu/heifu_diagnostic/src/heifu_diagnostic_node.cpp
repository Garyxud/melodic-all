#include "heifu_diagnostic/Heifu_diagnostic.hpp"

using namespace HD;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heifu_diagnostic_node");
    HD::Heifu_diagnostic heifuDiagnostic;
    heifuDiagnostic.run();
    return 0;
}

// Copyright (c) 2019-2020 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "psen_scan/scanner.h"
#include "psen_scan/scanner_communication_interface.h"
#include "psen_scan/psen_scan_udp_interface.h"
#include "psen_scan/ros_parameter_handler.h"
#include "psen_scan/ros_scanner_node.h"
#include "psen_scan/psen_scan_fatal_exception.h"
#include "psen_scan/default_parameters.h"

using namespace psen_scan;
typedef std::unique_ptr<PSENscanUDPInterface> PSENscanUDPptr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "psen_scan_node");
  ros::NodeHandle pnh("~");

  try
  {
    psen_scan::RosParameterHandler param_handler(pnh);
    PSENscanUDPptr udp_interface =
        PSENscanUDPptr(new PSENscanUDPInterface(param_handler.getSensorIP(), param_handler.getHostUDPPort()));

    std::unique_ptr<Scanner> scanner = std::unique_ptr<Scanner>(new Scanner(param_handler.getSensorIP(),
                                                                            param_handler.getHostIP(),
                                                                            param_handler.getHostUDPPort(),
                                                                            param_handler.getPassword(),
                                                                            param_handler.getAngleStart(),
                                                                            param_handler.getAngleEnd(),
                                                                            std::move(udp_interface)));
    ROSScannerNode ros_scanner_node(pnh,
                                    DEFAULT_PUBLISH_TOPIC,
                                    param_handler.getFrameID(),
                                    param_handler.getSkip(),
                                    param_handler.getXAxisRotation(),
                                    std::move(scanner));
    ros_scanner_node.processingLoop();
  }
  catch (PSENScanFatalException& e)
  {
    std::cerr << e.what() << std::endl;
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}

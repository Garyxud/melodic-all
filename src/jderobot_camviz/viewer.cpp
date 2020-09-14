/*
 *
 *  Copyright (C) 1997-2009 JDERobot Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/. 
 *
 *  Authors : David Lobato Bravo <dav.lobato@gmail.com>
 *
 */

#include "viewer.h"
#include <iostream>
#include <cmath>
#include <string>

namespace camViz {

  Viewer::Viewer()
    : gtkmain(0, 0), frameCount(0) {

    std::cout << "Loading glade\n";

  Glib::RefPtr<Gtk::Builder> builder= Gtk::Builder::create();


try
  {
    builder->add_from_file("camViz.glade");
  }
  catch(const Glib::FileError& ex)
  {
    std::cerr << "FileError: " << ex.what() << std::endl;
    //return 1;
  }
  catch(const Gtk::BuilderError& ex)
  {
    std::cerr << "BuilderError: " << ex.what() << std::endl;
    //return 1;
  }

    builder->get_widget("image", gtkimage);

    builder->get_widget("mainwindow",mainwindow);
    builder->get_widget("fpslabel",fpslabel);

  }
    

  Viewer::~Viewer() {}

  bool Viewer::isVisible(){
    return mainwindow->is_visible();
  }



  void Viewer::display(cv::Mat imageRGB)
  {

	  if (!imageRGB.empty()){

		  Glib::RefPtr<Gdk::Pixbuf> imgBuff =
			Gdk::Pixbuf::create_from_data((const guint8*) imageRGB.data,Gdk::COLORSPACE_RGB,false,8,imageRGB.cols,imageRGB.rows,imageRGB.step);

		  gtkimage->clear();
		  gtkimage->set(imgBuff);

	  }
	  mainwindow->resize(1,1);
	  while (gtkmain.events_pending())
		  gtkmain.iteration();

  }
    
  void
  Viewer::displayFrameRate(int rate)
  {
      std::stringstream fpsString;
      fpsString << "fps = " << rate;
      fpslabel->set_label(fpsString.str());
  }


}

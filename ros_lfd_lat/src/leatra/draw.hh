#ifndef DRAW_H
#define DRAW_H

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include "stringhelp.hh"
#include "leatra.hh"
#include "litera.hh"

/**
 * @author  Heiko Posenauer
 * @date 02.02.2012
 * @version 2.0
 *
 * The class draw provides functionality to make the results of a constraints_fusion visible.
 */

class draw{
private:
  
  int group_gnuscript(ndmapSetGroup group, std::string dir, std::string script);
  int group_latex(ndmapSetGroup group, std::string dir, std::string latex);
      
public:
  
  int graph_std(ndmapSetGroup group);
  int graph_all(ndmapSetGroup group);
};

#endif

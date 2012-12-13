#include "stringhelp.hh"

/**
 *      The c-function intTOstring converts the parameter "int number" to a c-string. 
 */
std::string intTOstring(int number)
{
   std::ostringstream tmp; 
   tmp << number;    
   return tmp.str(); 
  
}

/**
 *      The c-function doubleTOstring converts the parameter "double number" to a c-string. 
 */
std::string doubleTOstring(double number)
{
   std::ostringstream tmp; 
   tmp << number;    
   return tmp.str(); 
  
}


double stringTOdouble(std::string number)
{
  std::istringstream tmp;
  tmp.str(number);
  double x;
  tmp >> x;
  return x;
  
}
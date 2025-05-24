#include <string>
using std::string;

# include "coordinates.h"

string process_coords(string coords){
    // Don't forget to do the error handling
      stringstream ss(coords);

      string t;

      char del = ';';
      
      while (getline(ss, t, del))
          cout << "\"" << t << "\"" << " ";
      return;
  }
#include <string>
using std::string;
#include <tuple>
#include <bits/stdc++.h>
using namespace std;

#ifndef COORDINATES
#define COORDINATES

// Perso comment: Let's say that the formatting is as follows:
// "[deg1];[deg2]" ex."[25];[40]"

namespace coordinates {
    std::tuple<double, double> process_coords(string coords)
    {
      // Don't forget to do the error handling
        stringstream ss(coords);

        string t;

        char del = ';';
        
        while (getline(ss, t, del))
            cout << "\"" << t << "\"" << " ";
        return;
    }
}

#endif
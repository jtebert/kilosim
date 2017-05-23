#include <fstream>
#include <sstream>
#include "vars.h"
#include "shapes.h">


/*std::vector<polygon_t> gen_color_squares(std::string filename) {
    // Generate the list of polygons (colored squares) included in the given file

    if (stat(filename.c_str(), &info) != 0) {
        printf("Error: Shapes file '%s' does not exist.", filename.c_str());
        exit(1);
    }

    std::vector<polygon_t> polys;
    std::ifstream infile(filename);
    float x1, y1, x2, y2, x3, y3, x4, y4, c1, c2, c3;
    std::string line;

    while (infile >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> c1 >> c2 >> c3) {
        polygon_t new_poly = {{{x1,y1}, {x2,y2}, {x3,y3}, {x4,y4}}, {c1, c2, c3}};
        polys.push_back(new_poly);
    }
    return polys;
}*/


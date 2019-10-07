#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
    float start_x, start_y, end_x, end_y;
    bool out_of_range; 
    std::cout << "|===================================================|" << std::endl;
    std::cout << "Please enter the start coordinate and end coordinate" << std::endl;
    std::cout << "Your start/end point must be within the range [0,0] to [100,100]" << std::endl;
    std::cout << "Syntax [start x] [start y] [end x] [end y] " << std::endl;
    do {
        std::cin >> start_x >> start_y >> end_x >> end_y;
        // checking the range of start(x,y) and end(x,y)
        if ((start_x < 0 || start_x > 100) ||
            (start_y < 0 || start_y > 100) || 
            (end_x < 0 || end_x > 100) || 
            (end_y < 0 || end_y > 100) )
        {
            out_of_range = 1; 
        }
        else out_of_range = 0;

        if (!out_of_range){
            std::cout << "You entered" << std::endl;
            std::cout << "START=[" << start_x << "," << start_y << "]"<< " and END=[" <<  end_x << "," << end_y << "]"<< std::endl;
            std::cout << "|===================================================|" << std::endl;
        }
        else 
        {
            std::cout << "You entered the out of range start/end point" << std::endl;
            std::cout << "START=[" << start_x << "," << start_y << "]"<< " and END=[" <<  end_x << "," << end_y << "]"<< std::endl;
            std::cout << "Your start/end point must be within the range [0,0] to [100,100]" << std::endl;
            std::cout << "Please re-enter with the following syntax [start x] [start y] [end x] [end y] " << std::endl;
        }
    } while (out_of_range); 
    
    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}

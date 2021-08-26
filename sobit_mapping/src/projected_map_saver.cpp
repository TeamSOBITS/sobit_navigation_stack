#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
// Ref : https://github.com/ros-planning/navigation/blob/noetic-devel/map_server/src/map_saver.cpp

using namespace std;

/**
 * @brief Map generation node.
 */
class MapGenerator
{
  private : 
    nav_msgs::OccupancyGrid map_;
  public:
    MapGenerator(const std::string& mapname, int threshold_occupied, int threshold_free)
      : mapname_(mapname), saved_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free) {
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("projected_map", 1, &MapGenerator::mapCallback, this);
    }
    bool saveMap() {
      ROS_INFO("Received a %d X %d map @ %.3f m/pix", map_.info.width, map_.info.height, map_.info.resolution);

      std::string mapdatafile = mapname_ + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out) {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return false;
      }

      fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n", map_.info.resolution, map_.info.width, map_.info.height);
      for(unsigned int y = 0; y < map_.info.height; y++) {
        for(unsigned int x = 0; x < map_.info.width; x++) {
          unsigned int i = x + (map_.info.height - y - 1) * map_.info.width;
          if (map_.data[i] >= 0 && map_.data[i] <= threshold_free_) { // [0,free)
            fputc(254, out);
          } else if (map_.data[i] >= threshold_occupied_) { // (occ,255]
            fputc(000, out);
          } else { //occ [0.25,0.65]
            fputc(205, out);
          }
        }
      }

      fclose(out);

      std::string mapmetadatafile = mapname_ + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

            /*
      resolution: 0.100000
      origin: [0.000000, 0.000000, 0.000000]
      #
      negate: 0
      occupied_thresh: 0.65
      free_thresh: 0.196
            */

      geometry_msgs::Quaternion orientation = map_.info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map_.info.resolution, map_.info.origin.position.x, map_.info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("Done\n");
      saved_map_ = true;
      return true;
    }

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map) {
      map_ = *map;
    }

    std::string mapname_;
    ros::Subscriber map_sub_;
    bool saved_map_;
    int threshold_occupied_;
    int threshold_free_;

};

int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

#define USAGE "Usage: \n" \
              "  projected_map_saver -h\n"\
              "  projected_map_saver [--occ <threshold_occupied>] [--free <threshold_free>] [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "projected_map_saver");
  std::string mapname = "projected_map";
  int threshold_occupied = 65;
  int threshold_free = 25;

  for(int i=1; i<argc; i++) {
    if(!strcmp(argv[i], "-h")) {
      puts(USAGE);
      return 0;
    } else if(!strcmp(argv[i], "-f")) {
      if(++i < argc) mapname = argv[i];
      else {
        puts(USAGE);
        return 1;
      }
    } else if (!strcmp(argv[i], "--occ")) {
      if (++i < argc) {
        threshold_occupied = std::atoi(argv[i]);
        if (threshold_occupied < 1 || threshold_occupied > 100) {
          ROS_ERROR("threshold_occupied must be between 1 and 100");
          return 1;
        }
      } else {
        puts(USAGE);
        return 1;
      }
    } else if (!strcmp(argv[i], "--free")) {
      if (++i < argc) {
        threshold_free = std::atoi(argv[i]);
        if (threshold_free < 0 || threshold_free > 100) {
          ROS_ERROR("threshold_free must be between 0 and 100");
          return 1;
        }
      } else {
        puts(USAGE);
        return 1;
      }
    } else {
      puts(USAGE);
      return 1;
    }
  }

  if (threshold_occupied <= threshold_free) {
    ROS_ERROR("threshold_free must be smaller than threshold_occupied");
    return 1;
  }
  time_t t = time(nullptr);
  const tm* localTime = localtime(&t);
  std::stringstream s;
  ros::NodeHandle pnh("~");
  std::string map_save_path = pnh.param<std::string>( "map_save_path", "/map" );
  mapname = map_save_path + "/" + mapname 
            + "_" + to_string(localTime->tm_mon + 1) 
            + "_" + to_string(localTime->tm_mday)
            + "_" + to_string(localTime->tm_hour)
            + "_" + to_string(localTime->tm_min);
  MapGenerator mg(mapname, threshold_occupied, threshold_free);
  ROS_INFO("\nEnter 's' in the terminal to save the map.\n");
  while( ros::ok()) {
    ros::spinOnce();
    if (kbhit()) {
      int key = getchar();
      if ( key == 's') {
          ROS_INFO("\n[ Save projected_map ]");
          mg.saveMap();
          break;
      }
    }
  }
  return 0;
}

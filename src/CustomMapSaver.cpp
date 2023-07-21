/*
 * map_saver
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
#include "RoboticsProject_2/save_map.h"
#include <ros/package.h>



using namespace std;

/**
 * @brief Map generation node.
 */
class MapGenerator
{
  private:
    nav_msgs::OccupancyGridConstPtr map;
    nav_msgs::Path path;
    long fileInitialOffset;
    unsigned int dimX,dimY;
    double mapRes,mapOriginX,mapOriginY;
    void calculateOffset(FILE* out){
      fseek(out,0L,SEEK_SET);
      char str[5]; 
      do{
        fgets(str,sizeof(char)*5,out);
      }while(strcasecmp(str,"255\n"));
      
      fileInitialOffset = ftell(out);
      std::cout <<"position: "<< fileInitialOffset;
    }
  
    void addPoint(FILE* out,unsigned int x,unsigned int y,char color){
      fseek(out,(fileInitialOffset+(x+dimX*y)),SEEK_SET);
      fputc(color, out);
    }
    void addBigPoint(FILE* out,unsigned int x,unsigned int y,int dim,char color){
      for(int a =(x-(dim/2));a<(x+(dim/2));a++){
        for(int b =(y-(dim/2));b<(y+(dim/2));b++){
          addPoint(out,a,b,color);
        }
      }
    }
    void drawLine(FILE* out,unsigned int originX, unsigned int originY, unsigned int endpointX, unsigned int endpointY) {
        int deltaY = endpointY - originY;
        int deltaX = endpointX - originX;
        float error = 0;

        // Note the below fails for completely vertical lines.
        if(deltaX != 0){
          float deltaError = abs((float)(deltaY / deltaX));
          if(deltaError <= 1){
            unsigned int Y = originY;
            unsigned int X;
            if (deltaX>0){

              for (X=originX; X<=endpointX; X++) {
                addBigPoint(out,X,Y,2,111);
                error += deltaError;
                if (error >= 0.5) {
                    Y += deltaY/abs(deltaY);
                    error -= 1.0;
                }
              }
              
            }else{
              for (X=originX; X>=endpointX; X--) {
                addBigPoint(out,X,Y,2,111);
                error += deltaError;
                if (error >= 0.5) {
                    Y += deltaY/abs(deltaY);
                    error -= 1.0;
                }
              }
            }
          if(deltaY < 0){
            for(; Y>=endpointY; Y--){
              addBigPoint(out,X,Y,2,111);
            }
          }else{
            for(; Y<=endpointY; Y++){
              addBigPoint(out,X,Y,2,111);
            }
          }
            
            
          
          }else{
            unsigned int X = originX;
            unsigned int Y;
            if (deltaY>0){
              for (Y=originY; Y<=endpointY; Y++) {
                addBigPoint(out,X,Y,2,111);
                error += 1/deltaError;
                if (error >= 0.5) {
                    X += deltaX/abs(deltaX);
                    //addPoint(out,X,Y,111);
                    error -= 1.0;
                }
              }
              
            }else{
              for (Y=originY; Y>=endpointY; Y--) {
                addBigPoint(out,X,Y,2,111);
                error += 1/deltaError;
                if (error >= 0.5) {
                    X += deltaX/abs(deltaX);
                    //addPoint(out,X,Y,111);
                    error -= 1.0;
                }
              }
              
            }
            if(deltaX < 0){
              for(; X>=endpointX; X--){
                addBigPoint(out,X,Y,2,111);
              }
            }else{
              for(; X<=endpointX; X++){
                addBigPoint(out,X,Y,2,111);
              }
            }
          }

          
        }else{
          if (deltaY>0){
            for(unsigned int Y=originY; Y<=endpointY; Y++){
              addBigPoint(out,originX,Y,2,111);
            }
          }else{
            for(unsigned int Y=originY; Y>=endpointY; Y--){
              addBigPoint(out,originX,Y,2,111);
            }
          }
          
        }
        
    }
    void addTrajectory(FILE* out){
      calculateOffset(out);
      unsigned int ox = abs(mapOriginX/mapRes);
      unsigned int oy = dimY - abs(mapOriginY/mapRes);
      std::cout<<"origin x: "<<ox<<" y: "<<oy<<endl;
      addBigPoint(out,ox,oy,6,80);
      for (int p = 1; p<path.poses.size();p++){
        unsigned int spx = ox + (path.poses[p-1].pose.position.x/mapRes);
        unsigned int spy = oy - (path.poses[p-1].pose.position.y/mapRes);
        unsigned int epx = ox + (path.poses[p].pose.position.x/mapRes);
        unsigned int epy = oy - (path.poses[p].pose.position.y/mapRes);
        
        //std::cout<<"point: "<<p<<" x: "<<px<<" y: "<<py<<endl;
        drawLine(out,spx,spy,epx,epy);
        //addBigPoint(out,spx,spy,4,80);
      }
      //drawLine(out,300,200,300,300);
    }

  public:
    MapGenerator(int threshold_occupied, int threshold_free)
      : saved_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free)
    {
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
      path_sub = n.subscribe("path", 1, &MapGenerator::pathCallback, this);
      saveMapService = n.advertiseService("save_map" , &MapGenerator::saveMap, this);

    }
    void pathCallback(const nav_msgs::Path& newPath){
      path = newPath;
    }
    
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& newMap){
      map = newMap;
    }

    bool saveMap(RoboticsProject_2::save_map::Request  &req,
                      RoboticsProject_2::save_map::Response &res){
      std::string mapname_ = ros::package::getPath("RoboticsProject_2") + "/"+ req.mapname;
      ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map->info.width,
               map->info.height,
               map->info.resolution);
      dimX = map->info.width;
      dimY = map->info.height;
      mapRes = map->info.resolution;
      mapOriginX = map->info.origin.position.x;
      mapOriginY = map->info.origin.position.y;


      std::string mapdatafile = mapname_ + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w+");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return false;
      }

      fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] >= 0 && map->data[i] <= threshold_free_) { // [0,free)
            fputc(254, out);
          } else if (map->data[i] >= threshold_occupied_) { // (occ,255]
            fputc(000, out);
          } else { //occ [0.25,0.65]
            fputc(205, out);
          }
        }
      }
      addTrajectory(out);

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

      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("Done\n");
      saved_map_ = true;
      return true;
    }

    std::string mapname_;
    ros::Subscriber map_sub_;
    ros::Subscriber path_sub;
    ros::ServiceServer saveMapService;
    bool saved_map_;
    int threshold_occupied_;
    int threshold_free_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "custom_map_saver");
  int threshold_occupied = 65;
  int threshold_free = 25;


  MapGenerator mg(threshold_occupied, threshold_free);

  while(ros::ok())
    ros::spinOnce();

  return 0;
}


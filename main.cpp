#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>

using namespace cv;
using namespace std;

const int MAP_W = 680;
const int MAP_H = 400;

const auto PI = acos(-1.0);

struct Vector2i {
  int x;
  int y;
};
typedef struct Vector2i Vector2i;

struct Vector2f {
  float x;
  float y;
};
typedef struct Vector2f Vector2f;

enum TileType {
  EMPTY,
  BLOCKADE
};

//map data, 1 represents wall, 0 - no wall
int map[MAP_H][MAP_W];

Vector2i robotMapPos = {400, 300};
float robotAngle = -20;

Vector2f robotWorldPos;

Vector2i tileSize = {1, 1};

float deg2rad(float deg) {
  return deg * PI / 180;
}


void drawMap(Mat img) {
  for (int y = 0; y < MAP_H; y++) {
    for (int x = 0; x < MAP_W; x++) {
      cv::Point2i P1 = {x*tileSize.x, y*tileSize.y};
      cv::Point2i P2 = {P1.x+tileSize.x, P1.y+tileSize.y};

      //we need to check by [y][x] to draw correctly because of array structure
      if (map[y][x] == TileType::BLOCKADE) {
        cv::rectangle(img, P1, P2, Scalar(0, 0, 0), CV_FILLED);
      } else {
        cv::rectangle(img, P1, P2, Scalar(255, 255, 255), CV_FILLED);
      }
    }
  }
}


//get raycast closest hit point
Vector2f getDistToClosestHitPoint(float angle, Vector2i rayMapPos, Vector2f rayWorldPos) {
  Vector2f rayDir = {cos(angle), sin(angle)};

  float dyh = 0; //dist y to next horizontal tile
  float dxh = 0; //dist x to next horizontal tile

  if (rayWorldPos.y == rayMapPos.y * tileSize.y) {
    dyh = tileSize.y;
  } else {
    if (rayDir.y < 0) dyh = rayWorldPos.y - (rayMapPos.y * tileSize.y);
    else dyh = (rayMapPos.y + 1) * tileSize.y - rayWorldPos.y;
  }

  dxh = dyh / tan(angle);
  if (rayDir.y < 0) //invert distances values when pointing upwards
  {
    dxh = -dxh;
    dyh = -dyh;
  }

  float dyv = 0; //dist y to next vertical tile
  float dxv = 0; //dist x to next vertical tile

  if (rayWorldPos.x == rayMapPos.x * tileSize.x) {
    dxv = tileSize.x;
  } else {
    if (rayDir.x < 0) dxv = rayWorldPos.x - (rayMapPos.x * tileSize.x);
    else dxv = (rayMapPos.x + 1) * tileSize.x - rayWorldPos.x;
  }

  dyv = dxv * tan(angle);
  if (rayDir.x < 0) //invert distances values when pointing upwards
  {
    dxv = -dxv;
    dyv = -dyv;
  }

  //calc squares and compare them
  float sqrLenHor = dxh * dxh + dyh * dyh;
  float sqrLenVer = dxv * dxv + dyv * dyv;

  //select distances which squares are lower
  float dx = sqrLenHor < sqrLenVer ? dxh : dxv;
  float dy = sqrLenHor < sqrLenVer ? dyh : dyv;

  return {dx, dy};
}


void visualizePlayerRaycast(Mat img) {

  //get robot rotation angle and direction vector
  float angle = deg2rad(robotAngle);

  Vector2f dir = {cos(angle), sin(angle)};

  std::cout << "Dir of ray is " << dir.x << ":" << dir.y << std::endl;

  //get distance to first hit point
  Vector2f dist = getDistToClosestHitPoint(angle, robotMapPos, robotWorldPos);

  std::cout << "Position of robot in map " << robotMapPos.x << ":" << robotMapPos.y << std::endl;
  std::cout << "Position of robot in world " << robotWorldPos.x << ":" << robotWorldPos.y << std::endl;

  std::cout << "distance to first hit point is " << dist.x << ":" << dist.y << std::endl;

  //first ray hit position coordinates
  Vector2f rayWorldPos = {robotWorldPos.x + dist.x, robotWorldPos.y + dist.y};
  Vector2i rayPosMap = {int(rayWorldPos.x / tileSize.x),
                        int(rayWorldPos.y / tileSize.y)}; //just divide world coordinates by tile size

  std::cout << "Position of ray in world " << rayWorldPos.x << ":" << rayWorldPos.y << std::endl;
  std::cout << "Position of ray in map " << rayPosMap.x << ":" << rayPosMap.y << std::endl;

  bool hit = false;

  //raycast loop
  while (!hit) {
    // draw small green circle at center of object detected
//    cv::circle(img,            // draw on original image
//               cv::Point(int(rayWorldPos.x), int(rayWorldPos.y)),  // center point of circle
//               5,                // radius of circle in pixels
//               cv::Scalar(0, 0, 255),           // draw green
//               CV_FILLED);              // thickness

    //out of array range exceptions handling
    if (rayPosMap.x < 0 || rayPosMap.x >= MAP_W || rayPosMap.y < 0 || rayPosMap.y >= MAP_H) break;

    //checking that actually hit side is wall side
    int hitTileX = rayPosMap.x;
    int hitTileY = rayPosMap.y;

    //fix checking walls when hit them on their right or bottom side, check walls earlier them
    if (rayWorldPos.x == rayPosMap.x * tileSize.x && dir.x < 0) //hit wall left side
    {
      hitTileX--;
    }

    if (rayWorldPos.y == rayPosMap.y * tileSize.y && dir.y < 0) //hit wall up side
    {
      hitTileY--;
    }

    if (map[hitTileY][hitTileX] == BLOCKADE) {
      hit = true; //end raycasting loop
      double d = sqrt(pow(rayWorldPos.x - robotWorldPos.x, 2) + pow(rayWorldPos.y - robotWorldPos.y, 2));
      std::cout << "wall is position " << hitTileX << ":" << hitTileY << std::endl;
      std::cout << "distance before hitting wall " << d << std::endl;
      cv::line(img, cv::Point(robotWorldPos.x, robotWorldPos.y), Point(rayWorldPos.x, rayWorldPos.y), Scalar(255, 0, 0), 1, CV_AA);
    } else {
      //move ray to next closest horizontal or vertical side
      Vector2f dist = getDistToClosestHitPoint(angle, {rayPosMap.x, rayPosMap.y}, {rayWorldPos.x, rayWorldPos.y});

      //apply new move
      rayWorldPos.x += dist.x;
      rayWorldPos.y += dist.y;

      //update map positions
      rayPosMap.x = (int) rayWorldPos.x / tileSize.x;
      rayPosMap.y = (int) rayWorldPos.y / tileSize.y;
    }
  }
}




int main() {

  string line;
  ifstream myfile ("Assignment_04_Grid_Map.pbm");
  if (myfile.is_open()) {
    int k = 0;
    int x = 0;
    int y = 0;
    while (getline(myfile, line)) {
      k++;
      if (k < 3) continue;

      for(char& c : line) {
        int v = c - '0';
        map[y][x] = v;

        x++;
        x%=MAP_W;
        if (x==0) {
          y++;
        }
      }
    }
    std::cout << x << " - " << y << std::endl;
    myfile.close();
  }

  std::cout << map[robotMapPos.y][robotMapPos.x] << std::endl;
  robotWorldPos.x = robotMapPos.x * tileSize.x + 2;
  robotWorldPos.y = robotMapPos.y * tileSize.y + 2;

  Mat src(MAP_H*tileSize.x, MAP_W*tileSize.y, CV_8UC3, Scalar(0, 0, 0));


//  src = imread( "Assignment_04_Grid_Map.png", 1 );

//  if (src.empty()) {
//    std::cout << "error: frame can't read \n";      // print error message
//    return  1;               // jump out of loop
//  }

  char* source_window = "Raycasting";
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );

  drawMap(src);

  int k = 0;
  robotAngle-=125;
  int u = 0;
  while (k != 'q')  {
    visualizePlayerRaycast(src);
    imshow( source_window, src );
//    imshow( source_window, src );
    if (u < 250) robotAngle += 2;
    k = cv::waitKey(100);
    u+=2;
  }

  return 0;
}
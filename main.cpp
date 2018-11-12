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
Mat img;

Vector2i robotMapPos = {40, 180};
float robotAngle = 125;

Vector2f robotWorldPos;

Vector2i tileSize = {4, 4};

float deg2rad(float deg) {
  return deg * PI / 180;
}

void drawMap(Mat img) {
  for (int y = 0; y < MAP_H; y++) {
    for (int x = 0; x < MAP_W; x++) {
      cv::Point2i P1 = {x * 1, y * 1};
      cv::Point2i P2 = {P1.x + 1, P1.y + 1};

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

double robotRaycast(bool draw) {

  //get robot rotation angle and direction vector
  float angle = deg2rad(robotAngle);

  Vector2f dir = {cos(angle), sin(angle)};

  //get distance to first hit point
  Vector2f dist = getDistToClosestHitPoint(angle, robotMapPos, robotWorldPos);

  //first ray hit position coordinates
  Vector2f rayWorldPos = {robotWorldPos.x + dist.x, robotWorldPos.y + dist.y};
  Vector2i rayMapPos = {int(rayWorldPos.x / tileSize.x),
                        int(rayWorldPos.y / tileSize.y)}; //just divide world coordinates by tile size

  //raycast loop
  while (true) {
    //out of array range exceptions handling
    if (rayMapPos.x < 0 || rayMapPos.x >= MAP_W || rayMapPos.y < 0 || rayMapPos.y >= MAP_H) break;

    //checking that actually hit side is wall side
    int hitTileX = rayMapPos.x;
    int hitTileY = rayMapPos.y;

    //fix checking walls when hit them on their right or bottom side, check walls earlier them
    if (rayWorldPos.x == rayMapPos.x * tileSize.x && dir.x < 0) //hit wall left side
    {
      hitTileX--;
    }

    if (rayWorldPos.y == rayMapPos.y * tileSize.y && dir.y < 0) //hit wall up side
    {
      hitTileY--;
    }

    if (map[hitTileY][hitTileX] == BLOCKADE) {
      double d = sqrt(pow(rayWorldPos.x - robotWorldPos.x, 2) + pow(rayWorldPos.y - robotWorldPos.y, 2));
//      std::cout << "wall is position " << hitTileX << ":" << hitTileY << std::endl;
//      std::cout << "distance before hitting wall " << d << std::endl;
      if (draw)
        cv::line(img, cv::Point(robotMapPos.x, robotMapPos.y), Point(rayMapPos.x, rayMapPos.y), Scalar(255, 0, 0), 1,
                 CV_AA);
      return d;
    } else {
      //move ray to next closest horizontal or vertical side
      Vector2f dist = getDistToClosestHitPoint(angle, {rayMapPos.x, rayMapPos.y}, {rayWorldPos.x, rayWorldPos.y});

      //apply new move
      rayWorldPos.x += dist.x;
      rayWorldPos.y += dist.y;
      double d = sqrt(pow(rayWorldPos.x - robotWorldPos.x, 2) + pow(rayWorldPos.y - robotWorldPos.y, 2));
      if (d > 1200) {
        rayWorldPos.x -= dist.x;
        rayWorldPos.y -= dist.y;
//        std::cout << "No wall found until 12m. Stopping on tile " << hitTileX << ":" << hitTileY << std::endl;
        if (draw)
          cv::line(img, cv::Point(robotMapPos.x, robotMapPos.y), Point(rayMapPos.x, rayMapPos.y), Scalar(255, 0, 0), 1,
                   CV_AA);
        return 1200;
      }

      //update map positions
      rayMapPos.x = (int) rayWorldPos.x / tileSize.x;
      rayMapPos.y = (int) rayWorldPos.y / tileSize.y;
    }
  }
}

void getRobotRaycast(double (&scan)[126]) {
  float startingAngle = robotAngle;
  robotAngle -= 125;
  int u = 0, i = 0;
  robotWorldPos.x = robotMapPos.x * tileSize.x + 2;
  robotWorldPos.y = robotMapPos.y * tileSize.y + 2;
  robotAngle = startingAngle - 125;

  while (u < 250) {
    double d = robotRaycast(false);
    scan[i] = d;
    i++;
    robotAngle += 2;
    u += 2;
  }
  robotAngle = startingAngle;
}

int main() {

  string line;
  ifstream myfile("Assignment_04_Grid_Map.pbm");
  if (myfile.is_open()) {
    int k = 0;
    int x = 0;
    int y = 0;
    while (getline(myfile, line)) {
      k++;
      if (k < 3) continue;

      for (char &c : line) {
        int v = c - '0';
        map[y][x] = v;

        x++;
        x %= MAP_W;
        if (x == 0) {
          y++;
        }
      }
    }
    myfile.close();
  }

  std::cout << map[robotMapPos.y][robotMapPos.x] << std::endl;
  robotWorldPos.x = robotMapPos.x * tileSize.x + 2;
  robotWorldPos.y = robotMapPos.y * tileSize.y + 2;

  Mat src(MAP_H, MAP_W, CV_8UC3, Scalar(0, 0, 0));
  img = src;

  char *source_window = const_cast<char *>("Raycasting");
  namedWindow(source_window, CV_WINDOW_AUTOSIZE);

  drawMap(src);
  imshow(source_window, src);

  // Assignemt 4.1

  double raycast[126];
  std::cout << "Getting raycast from initial robot position" << std::endl;
  getRobotRaycast(raycast);
  std::cout << "Raycast : (";
  for (double j : raycast) {
    std::cout << j <<";";
  }
  std::cout << ")" << std::endl;


  int k = 0;
  float startingAngle = robotAngle;
  robotAngle -= 125;
  int u = 0, i = 0;
  while (k != 'q') {
    bool move = false;
    if (k == 81) {
      robotMapPos.x -= 2;
      move = true;
    }
    if (k == 82) {
      robotMapPos.y -= 2;
      move = true;
    }
    if (k == 83) {
      robotMapPos.x += 2;
      move = true;
    }
    if (k == 84) {
      robotMapPos.y += 2;
      move = true;
    }
    if (move) {
      robotWorldPos.x = robotMapPos.x * tileSize.x + 2;
      robotWorldPos.y = robotMapPos.y * tileSize.y + 2;
      robotAngle = startingAngle - 125;
      u = 0;
      drawMap(src);
    }
    if (u < 250) {
      robotRaycast(true);
      i++;
      imshow(source_window, src);
      robotAngle += 2;
    }
    k = cv::waitKey(1);
    u += 2;
  }

  return 0;
}
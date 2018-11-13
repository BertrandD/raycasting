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

struct Raycast {
  float distX;
  float distY;
  double d;
};

enum TileType {
  EMPTY,
  BLOCKADE
};

//map data, 1 represents wall, 0 - no wall
int map[MAP_H][MAP_W];
Mat img;

Vector2i robotMapPos = {334, 180};
float robotAngle = 195;

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


struct Raycast robotRaycast(bool draw, double max) {

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
      // Wall is position hitTileX:hitTileY
      // Distance before hitting wall : d ;
      double d = sqrt(pow(rayWorldPos.x - robotWorldPos.x, 2) + pow(rayWorldPos.y - robotWorldPos.y, 2));
      if (draw)
        cv::line(img, cv::Point(robotMapPos.x, robotMapPos.y), Point(rayMapPos.x, rayMapPos.y), Scalar(255, 0, 0), 1,
                 CV_AA);
      return {rayWorldPos.x - robotWorldPos.x, rayWorldPos.y - robotWorldPos.y, d};
    } else {
      //move ray to next closest horizontal or vertical side
      Vector2f dist = getDistToClosestHitPoint(angle, {rayMapPos.x, rayMapPos.y}, {rayWorldPos.x, rayWorldPos.y});

      //apply new move
      rayWorldPos.x += dist.x;
      rayWorldPos.y += dist.y;
      double d = sqrt(pow(rayWorldPos.x - robotWorldPos.x, 2) + pow(rayWorldPos.y - robotWorldPos.y, 2));
      if (d > max) {
        rayWorldPos.x -= dist.x;
        rayWorldPos.y -= dist.y;
        // No wall found until 12m. Stopping on tile hitTileX:hitTileY
        if (draw)
          cv::line(img, cv::Point(robotMapPos.x, robotMapPos.y), Point(rayMapPos.x, rayMapPos.y), Scalar(255, 0, 0), 1,
                   CV_AA);
        return {rayWorldPos.x - robotWorldPos.x, rayWorldPos.y - robotWorldPos.y, max};
      }

      //update map positions
      rayMapPos.x = (int) rayWorldPos.x / tileSize.x;
      rayMapPos.y = (int) rayWorldPos.y / tileSize.y;
    }
  }
}

double getDistanceToClosestObstacle() {
  float startingAngle = robotAngle;
  int u = 0;
  double max = 600;
  robotWorldPos.x = robotMapPos.x * tileSize.x + 2;
  robotWorldPos.y = robotMapPos.y * tileSize.y + 2;

  while (u <= 360) {
    struct Raycast r = robotRaycast(false, max);
    if (r.d < max) max = r.d;
    robotAngle += 5;
    u += 5;
  }
  robotAngle = startingAngle;

  return max;
}

// p(z|x,m)
double getLikelihoodForMeasurements(struct Raycast raycast[125], double likelihood[MAP_H][MAP_W]) {
  double p_z_given_x_m = 0;
  Vector2f dir = {cos(deg2rad(robotAngle)), sin(deg2rad(robotAngle))};
  // Over all z_k
  for (int k = 0; k < 125; ++k) {
    struct Raycast z_k = raycast[k];
    if (z_k.d >= 1200) continue;
    auto distX = static_cast<float>(dir.x * z_k.d);
    auto distY = static_cast<float>(dir.y * z_k.d);

    Vector2f rayWorldPos = {robotWorldPos.x + distX, robotWorldPos.y + distY};
    Vector2i rayMapPos = {int(rayWorldPos.x / tileSize.x),
                          int(rayWorldPos.y / tileSize.y)}; //just divide world coordinates by tile size

    if (!(rayMapPos.x < 0 || rayMapPos.x >= MAP_W || rayMapPos.y < 0 || rayMapPos.y >= MAP_H)){
      if (p_z_given_x_m == 0) p_z_given_x_m = 1;
      // q = q * (z_hit * prob(dist, sigma_hit) )
      p_z_given_x_m *= 1 * likelihood[rayMapPos.y][rayMapPos.x];
    }
  }

  return p_z_given_x_m;
}

double getMaxLikelihoodForRobotPositionOverAllOrientations(struct Raycast raycast[125], double likelihood[MAP_H][MAP_W]) {
  float startingAngle = robotAngle;
  int u = 0;
  double max = 0;
  robotAngle = 0;

  while (u <= 360) {
    if (u == 92) {
      std::cout <<"";
    }
    if (max > 100) {
      std::cout <<"";
    }
    double p = getLikelihoodForMeasurements(raycast, likelihood);
    if (p > max) max = p;
    robotAngle += 2;
    u += 2;
  }
  robotAngle = startingAngle;

  return max;
}

void getRobotRaycast(struct Raycast (&scan)[125]) {
  float startingAngle = robotAngle;
  robotAngle -= 125;
  int u = 0, i = 0;
  robotWorldPos.x = robotMapPos.x * tileSize.x + 2;
  robotWorldPos.y = robotMapPos.y * tileSize.y + 2;
  robotAngle = startingAngle - 125;

  while (u < 250) {
    struct Raycast r = robotRaycast(false, 1200);
    scan[i] = r;
    i++;
    robotAngle += 2;
    u += 2;
  }
  robotAngle = startingAngle;
}

void outputMatrixToFile(string name, double matrix[MAP_H][MAP_W]) {
  ofstream myfile (name);
  if (myfile.is_open())
  {
    for (int l = 0; l < MAP_H; ++l) {
      for (int c = 0; c < MAP_W; ++c) {
        myfile << matrix[l][c] << " " ;
      }
      myfile << std::endl;
    }
    myfile.close();
  }
  else cout << "Unable to open file";
}

void loadMatrixFromFile(string name, double (&matrix)[MAP_H][MAP_W]) {
  string line;
  ifstream myfile(name);
  if (myfile.is_open()) {
    int x = 0;
    int y = 0;
    while (getline(myfile, line, ' ')) {
      if (line == "\n") {
        y++;
        x= 0;
        continue;
      };
      matrix[y][x] = stod(line);
      x++;
    }
    myfile.close();
  }
}

// prob(dist, sigma_hit)
// dist = d
// sigma_hit = 0.35m = 35cm
double computeLikelihoodForDistance(double d) {
  return 1 / (sqrt(2 * PI) * 35) * exp(-(d * d) / (2 * 35 * 35));
}

int main() {

  string line;
  ifstream myfile("Assignment_04_Grid_Map.pbm");
  int k = 0;
  if (myfile.is_open()) {
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
  Mat likelihoodMap(MAP_H, MAP_W, CV_8UC3, Scalar(0, 0, 0));
  Mat likelihoodMapForRaycast(MAP_H, MAP_W, CV_8UC3, Scalar(0, 0, 0));

  char *source_window = const_cast<char *>("Raycasting");
  namedWindow(source_window, CV_WINDOW_AUTOSIZE);
  namedWindow("Raycast Likelihood", CV_WINDOW_AUTOSIZE);
  namedWindow("Likelihood Field", CV_WINDOW_AUTOSIZE);

  // Assignemt 4.1

  struct Raycast raycast[125];
  std::cout << "Getting raycast from initial robot position" << std::endl;
  getRobotRaycast(raycast);
  std::cout << "Raycast : z = (";
  for (struct Raycast j : raycast) {
    std::cout << j.d << ";";
  }
  std::cout << ")" << std::endl;

  Vector2i bak = robotMapPos;

  double likelihood[MAP_H][MAP_W];
  double raycastLikelihood[MAP_H][MAP_W];
  std::cout << "distance to closest obstacle " << getDistanceToClosestObstacle() << std::endl;
  std::cout << "likelihood" << computeLikelihoodForDistance(getDistanceToClosestObstacle()) << std::endl;

//  std::cout << "Computing likelihood field" << std::endl;
//
//  for (int l = 0; l < MAP_H; ++l) {
//    for (int c = 0; c < MAP_W; ++c) {
//      if (c == 0 && l % 5 == 0) std::cout << l << "/400" << std::endl;
//      if (map[l][c] == BLOCKADE) {
//        likelihood[l][c] = computeLikelihoodForDistance(0);
//        continue;
//      };
//      robotMapPos.x = c;
//      robotMapPos.y = l;
//      // prob(dist, sigma_hit)
//      likelihood[l][c] = computeLikelihoodForDistance(getDistanceToClosestObstacle());
//    }
//  }
//
//  outputMatrixToFile("likelihood.map", likelihood);
  std::cout << "Loading likelihood field from file" << std::endl;
  loadMatrixFromFile("likelihood.map", likelihood);

  std::cout << std::endl << "Display likelihood field" << std::endl;

  for (int l = 0; l < MAP_H; ++l) {
    for (int c = 0; c < MAP_W; ++c) {
      cv::Point2i P1 = {c * 1, l * 1};
      cv::Point2i P2 = {P1.x + 1, P1.y + 1};

      //we need to check by [y][x] to draw correctly because of array structure
//      if (map[l][c] == TileType::BLOCKADE) {
//        cv::rectangle(likelihoodMap, P1, P2, Scalar(255, 50, 50), CV_FILLED);
//      } else {
        cv::rectangle(likelihoodMap, P1, P2,
                      Scalar(likelihood[l][c] * 255 / 0.011398, likelihood[l][c] * 255 / 0.011398,
                             likelihood[l][c] * 255 / 0.011398), CV_FILLED);
//      }
    }
  }
  imshow("Likelihood Field", likelihoodMap);
  cv::waitKey(500);

  std::cout << std::endl << "Display likelihood field for raycast" << std::endl;

//  double pMax = 0;
//  for (int l = 0; l < MAP_H; ++l) {
//    for (int c = 0; c < MAP_W; ++c) {
//      if (c == 0 && l % 5 == 0) std::cout << l << "/400" << std::endl;
//      if (map[l][c] == BLOCKADE) {
//        raycastLikelihood[l][c] = 0;
//        continue;
//      };
//      robotMapPos = {c, l};
//      robotWorldPos.x = robotMapPos.x * tileSize.x + 2;
//      robotWorldPos.y = robotMapPos.y * tileSize.y + 2;
//      if (c == bak.x && l == bak.y) {
//        std::cout << "";
//      }
//
//      double p = getMaxLikelihoodForRobotPositionOverAllOrientations(raycast, likelihood);
//      if (p > pMax) pMax = p;
//      raycastLikelihood[l][c] = p;
//    }
//  }
//  outputMatrixToFile("raycastLikelihood.map", raycastLikelihood);
  loadMatrixFromFile("raycastLikelihood.map", raycastLikelihood);

  double pMax = 0.0113984;
  for (int l = 0; l < MAP_H; ++l) {
    for (int c = 0; c < MAP_W; ++c) {
      if (map[l][c] == BLOCKADE) {
        continue;
      };
      cv::Point2i P1 = {c * 1, l * 1};
      cv::Point2i P2 = {P1.x + 1, P1.y + 1};
      double p = raycastLikelihood[l][c] / pMax;
      cv::rectangle(likelihoodMapForRaycast, P1, P2, Scalar(p * 255, p * 255,
                                                            p * 255), CV_FILLED);
    }
  }


  imshow("Raycast Likelihood", likelihoodMapForRaycast);

  drawMap(src);
  imshow(source_window, src);
  cv::waitKey(1);

  robotMapPos = bak;
  robotWorldPos.x = robotMapPos.x * tileSize.x + 2;
  robotWorldPos.y = robotMapPos.y * tileSize.y + 2;

  k = 0;
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
      std::cout << robotMapPos.x << ":" << robotMapPos.y << std::endl;
      robotWorldPos.x = robotMapPos.x * tileSize.x + 2;
      robotWorldPos.y = robotMapPos.y * tileSize.y + 2;
      robotAngle = startingAngle - 125;
      u = 0;
      drawMap(src);
    }
    if (u < 250) {
      robotRaycast(true, 1200);
      i++;
      imshow(source_window, src);
      robotAngle += 2;
    } else {
      cv::circle(src,            // draw on original image
                 cv::Point(robotMapPos.x, robotMapPos.y),  // center point of circle
                 10,                // radius of circle in pixels
                 cv::Scalar(0, 255, 0),           // draw green
                 CV_FILLED);              // thickness

      robotWorldPos.x = robotMapPos.x * tileSize.x + 2;
      robotWorldPos.y = robotMapPos.y * tileSize.y + 2;

      //first ray hit position coordinates
      Vector2f rayWorldPos = {robotWorldPos.x + cos(deg2rad(startingAngle)) * 40, robotWorldPos.y + sin(deg2rad(startingAngle)) * 40};
      Vector2i rayMapPos = {int(rayWorldPos.x / tileSize.x),
                            int(rayWorldPos.y / tileSize.y)}; //just divide world coordinates by tile size

      cv::line(src, cv::Point(robotMapPos.x, robotMapPos.y), Point(rayMapPos.x, rayMapPos.y), Scalar(0, 0, 255), 1,
               CV_AA);
      imshow(source_window, src);
    }
    k = cv::waitKey(1);
    u += 2;
  }

  return 0;
}
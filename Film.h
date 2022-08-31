//
// Created by albus on 2022/08/19.
//

#ifndef RENDERING2022_FILM_H
#define RENDERING2022_FILM_H

#include <string>
#include <GL/freeglut.h>

#include "Camera.h"

class Film {
public:
    Film(const int _width, const int _height);
    void init();
    void reset();
    void update();
    void draw(const Camera& camera);
    void clearRayTracedResult();
    void savePpm(std::string filePath, const int numOfSamples);

private:
    int width;
    int height;
    GLuint textureIdx;
};


#endif //RENDERING2022_FILM_H

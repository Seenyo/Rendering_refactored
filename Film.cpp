//
// Created by albus on 2022/08/19.
//

#include <fstream>
#include <iostream>

#include "Film.h"

int *g_CountBuffer = nullptr;
float *g_FilmBuffer = nullptr;
float *g_AccumulationBuffer = nullptr;

Film::Film(const int _width, const int _height) : width(_width), height(_height), textureIdx(0){}

void Film::init() {
    g_FilmBuffer = (float *) malloc(sizeof(float) * width * height * 3);
    memset(g_FilmBuffer, 0, sizeof(float) * width * height * 3);

    g_AccumulationBuffer = (float *) malloc(sizeof(float) * width * height * 3);
    g_CountBuffer = (int *) malloc(sizeof(int) * width * height);

    glGenTextures(1, &textureIdx);
    glBindTexture(GL_TEXTURE_2D, textureIdx);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_FLOAT, g_FilmBuffer);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

void Film::reset() {
    memset(g_AccumulationBuffer, 0, sizeof(float) * width * height * 3);
    memset(g_CountBuffer, 0, sizeof(int) * width * height);
}


void Film::update() {

    for (int i = 0; i < width * height; i++) {
        if (g_CountBuffer[i] > 0) {
            g_FilmBuffer[i * 3] = g_AccumulationBuffer[i * 3] / g_CountBuffer[i];
            g_FilmBuffer[i * 3 + 1] = g_AccumulationBuffer[i * 3 + 1] / g_CountBuffer[i];
            g_FilmBuffer[i * 3 + 2] = g_AccumulationBuffer[i * 3 + 2] / g_CountBuffer[i];
        } else {
            g_FilmBuffer[i * 3] = 0.0;
            g_FilmBuffer[i * 3 + 1] = 0.0;
            g_FilmBuffer[i * 3 + 2] = 0.0;
        }
    }
    glBindTexture(GL_TEXTURE_2D, textureIdx);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGB, GL_FLOAT, g_FilmBuffer);
}

void Film::draw(const Camera& camera) {
    Eigen::Vector3d screen_center = camera.getEyePoint() - camera.getZVector() * camera.getFocalLength();
    Eigen::Vector3d p1 = screen_center - camera.getXVector() * camera.getScreenWidth() * 0.5 - camera.getYVector() * camera.getScreenHeight() * 0.5;
    Eigen::Vector3d p2 = screen_center + camera.getXVector() * camera.getScreenWidth() * 0.5 - camera.getYVector() * camera.getScreenHeight() * 0.5;
    Eigen::Vector3d p3 = screen_center + camera.getXVector() * camera.getScreenWidth() * 0.5 + camera.getYVector() * camera.getScreenHeight() * 0.5;
    Eigen::Vector3d p4 = screen_center - camera.getXVector() * camera.getScreenWidth() * 0.5 + camera.getYVector() * camera.getScreenHeight() * 0.5;

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textureIdx);

    glBegin(GL_TRIANGLES);
    glColor3f(1.0, 1.0, 1.0);

    glTexCoord2f(0.0, 1.0);
    glVertex3f(p1.x(), p1.y(), p1.z());
    glTexCoord2f(1.0, 1.0);
    glVertex3f(p2.x(), p2.y(), p2.z());
    glTexCoord2f(1.0, 0.0);
    glVertex3f(p3.x(), p3.y(), p3.z());

    glTexCoord2f(0.0, 1.0);
    glVertex3f(p1.x(), p1.y(), p1.z());
    glTexCoord2f(1.0, 0.0);
    glVertex3f(p3.x(), p3.y(), p3.z());
    glTexCoord2f(0.0, 0.0);
    glVertex3f(p4.x(), p4.y(), p4.z());

    glEnd();

    glDisable(GL_TEXTURE_2D);
}

void Film::clearRayTracedResult() {
    memset(g_FilmBuffer, 0, sizeof(float) * width * height * 3);
}

void Film::savePpm(std::string filePath, const int numOfSamples) {
    std::ofstream writing_file;

    writing_file.open(filePath, std::ios::out);
    std::string header = "P3\n" + std::to_string(width) + " " + std::to_string(height) + "\n" + "255\n";
    writing_file << header << std::endl;

    for (int i = 0; i < width * height; i++) {
        std::string pixel = std::to_string(int(g_FilmBuffer[i * 3] * 255)) + " " + std::to_string(int(g_FilmBuffer[i * 3 + 1] * 255)) + " " + std::to_string(int(g_FilmBuffer[i * 3 + 2] * 255));
        if (i % width == width - 1) {
            writing_file << pixel << std::endl;
        } else {
            writing_file << pixel << " ";
        }
    }
    writing_file.close();
    std::cout << "Number of sample = " << numOfSamples << "  ppm file saved !\n";
}
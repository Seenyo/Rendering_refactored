#include <GL/freeglut.h>
#include <cmath>
#include <iostream>
#include <ctime>
#include <string>
#include <vector>

#include "./Camera.h"
#include "./RayHitInfo.h"
#include "./TriangleObj.h"
#include "./Film.h"
#include "./utils.h"
#include "TriangleObj.h"
#include "./constants.h"

#define TRIANGLE_NUM 9
#define LIGHT_SOURCE_NUM 7


#define FILM_WIDTH 640
#define FILM_HEIGHT 480

// ↓一旦置いとくけどFilmクラスに隠ぺいしたい
// Film.cppに実体あるよ～
extern int *g_CountBuffer;
extern float *g_FilmBuffer;
extern float *g_AccumulationBuffer;

Film g_Film = Film(FILM_WIDTH, FILM_HEIGHT);

enum MouseMode {
    MM_CAMERA
};

enum SamplingMethod {
    Direction,
    FacePoint
};

const int g_SampleNum = 50;

const SamplingMethod g_SamplingMethod = FacePoint;

int g_MousePosX, g_MousePosY;
int g_WindowWidth = 640;
int g_WindowHeight = 480;
int g_CountRayhitIdx[TRIANGLE_NUM];
int g_NumOfSample = 0;
Eigen::Vector3d g_LightIntensitySum;

double g_EmissionIntensity = 5;
double g_FrameSize_WindowSize_Scale_x = 1.0;
double g_FrameSize_WindowSize_Scale_y = 1.0;
double g_TotalTrianglesArea = 0.0;
double g_Pdf[TRIANGLE_NUM];
double g_Cdf[LIGHT_SOURCE_NUM];

bool g_DrawFilm = true;

MouseMode g_MouseMode = MM_CAMERA;
Camera g_Camera;
auto g_Triangles = std::vector<TriangleObj>(TRIANGLE_NUM);
// TriangleObj g_Triangles[TRIANGLE_NUM];

void TotalTriangleArea(double &t_area) {
    for (int i = 0; i < TRIANGLE_NUM; i++) {
        if (g_Triangles[i].is_light) {
            t_area += g_Triangles[i].area;
        }
    }
}

// シーン内の三角形の面積に比例する確率質量
void SetSceneObjectsProbabilityDistribution(double *PDF, double &t_area) {
    for (int i = 0; i < TRIANGLE_NUM; i++) {
        PDF[i] = g_Triangles[i].area / t_area;
    }
}

void SetSceneObjectsCumulativeDistribution(double *CDF, double *PDF) {
    for (int i = 0; i < LIGHT_SOURCE_NUM; i++) {
        if (i == 0) {
            CDF[i] = PDF[i];
        } else {
            CDF[i] = CDF[i - 1] + PDF[i];
        }
    }
}

void CalcAdoptionRatio() {
    TotalTriangleArea(g_TotalTrianglesArea);
    SetSceneObjectsProbabilityDistribution(g_Pdf, g_TotalTrianglesArea);
    SetSceneObjectsCumulativeDistribution(g_Cdf, g_Pdf);
}

void ChooseTriangle(int &t) {
    double random = drand48();
    auto Iter = std::lower_bound(std::begin(g_Cdf), std::end(g_Cdf), random);
    t = Iter - std::begin(g_Cdf);
}


void rayTracing(const Ray &in_Ray, RayHitInfo &io_Hit) {
    double t_min = __FAR__;
    double alpha_I = 0.0, beta_I = 0.0;
    int mesh_idx = -1;

    for (int i = 0; i < TRIANGLE_NUM; i++) {
        RayHitInfo temp_hit{};
        bool intersected = g_Triangles[i].rayIntersect(in_Ray, temp_hit);
        if (intersected && temp_hit.t < t_min) {
            t_min = temp_hit.t;
            alpha_I = temp_hit.alpha;
            beta_I = temp_hit.beta;
            mesh_idx = i;
        }
    }

    io_Hit.t = t_min;
    io_Hit.alpha = alpha_I;
    io_Hit.beta = beta_I;
    io_Hit.mesh_idx = mesh_idx;
}

void directional_sampling(const Eigen::Vector3d &x, const RayHitInfo &ray_hit, Eigen::Vector3d &pixel_color) {

    for (int n = 0; n < g_SampleNum; n++) {
        const double theta = asin(sqrt(drand48()));
        const double phi = 2 * M_PI * drand48();
        const Eigen::Vector3d u = g_Triangles[ray_hit.mesh_idx].normal;
        const Eigen::Vector3d v = u.y() < 1 ? u.cross(Eigen::Vector3d{0.0, 1.0, 0.0}).normalized() : u.cross(Eigen::Vector3d{1.0, 0.0, 0.0}).normalized();
        const Eigen::Vector3d w = v.cross(u);
        const Eigen::Vector3d omega_yup = {sin(theta) * cos(phi), cos(theta), sin(theta) * sin(phi)};
        const Eigen::Vector3d omega = omega_yup.y() * u + omega_yup.x() * v + omega_yup.z() * w;

        Ray _ray;
        _ray.o = x;
        _ray.d = omega;

        RayHitInfo _ray_hit;

        rayTracing(_ray, _ray_hit);

        if (_ray_hit.mesh_idx != -1 && g_Triangles[_ray_hit.mesh_idx].is_light == true) {
            g_LightIntensitySum = g_LightIntensitySum + g_EmissionIntensity * g_Triangles[_ray_hit.mesh_idx].color;
        }

    }
    pixel_color = g_Triangles[ray_hit.mesh_idx].kd * g_Triangles[ray_hit.mesh_idx].color.cwiseProduct(g_LightIntensitySum);
}

void surface_sampling(const Eigen::Vector3d &x, const double &totalTriangleArea, const RayHitInfo &ray_hit, Eigen::Vector3d &pixel_color) {

    for (int i = 0; i < g_SampleNum; i++) {
        int tri_idx;
        int V = 0;
        ChooseTriangle(tri_idx);
        const float gamma = 1.0 - sqrt(1.0 - drand48());
        const float beta = drand48() * (1.0 - gamma);
        const Eigen::Vector3d xa = (1.0 - beta - gamma) * g_Triangles[tri_idx].v1 + beta * g_Triangles[tri_idx].v2 + gamma * g_Triangles[tri_idx].v3;
        const Eigen::Vector3d xa_x = xa - x;
        const Eigen::Vector3d w = xa_x.normalized();
        const Eigen::Vector3d ny = g_Triangles[tri_idx].normal;
        const Eigen::Vector3d nx = g_Triangles[ray_hit.mesh_idx].normal;
        const double cosx = w.dot(nx);
        const double cosy = (-w).dot(ny);


        Ray _ray_x_xa;
        _ray_x_xa.o = xa;
        _ray_x_xa.d = -w;

        RayHitInfo _ray_hit_x_xa;

        rayTracing(_ray_x_xa, _ray_hit_x_xa);

        Ray _ray_xa_x;
        _ray_xa_x.o = x;
        _ray_xa_x.d = w;

        RayHitInfo _ray_hit_xa_x;

        rayTracing(_ray_xa_x, _ray_hit_xa_x);

        if (_ray_hit_x_xa.mesh_idx == ray_hit.mesh_idx && _ray_hit_xa_x.mesh_idx == tri_idx) {
            V = 1;
        }

        g_LightIntensitySum += (g_EmissionIntensity * g_Triangles[ray_hit.mesh_idx].kd / M_PI * cosx * cosy / xa_x.squaredNorm() * V) * g_Triangles[tri_idx].color;
    }

    g_LightIntensitySum *= totalTriangleArea;
    pixel_color = g_Triangles[ray_hit.mesh_idx].color.cwiseProduct(g_LightIntensitySum);
}

void Learn() {

    memset(g_CountRayhitIdx, 0, sizeof g_CountRayhitIdx);
    double totalTriangleArea = 0.0;
    TotalTriangleArea(totalTriangleArea);
    for (int Y = 0; Y < FILM_HEIGHT; Y++) {
        for (int X = 0; X < FILM_WIDTH; X++) {
            const int pixel_idx = Y * FILM_WIDTH + X;
            const double p_x = (X + 0.5) / FILM_WIDTH;
            const double p_y = (Y + 0.5) / FILM_HEIGHT;

            Ray ray;
            g_Camera.screenView(p_x, p_y, ray);

            RayHitInfo ray_hit;
            rayTracing(ray, ray_hit);

            if (ray_hit.mesh_idx >= 0) {
                if (g_Triangles[ray_hit.mesh_idx].is_light == true) {
                    for (int i = 0; i < 3; i++) {
                        g_AccumulationBuffer[pixel_idx * 3 + i] += g_Triangles[ray_hit.mesh_idx].color[i];
                    }
                    g_CountBuffer[pixel_idx] += 1;

                } else {

                    const Eigen::Vector3d x = ray.o + ray_hit.t * ray.d;
                    Eigen::Vector3d pixel_color;
                    Eigen::Vector3d I_n;
                    g_LightIntensitySum = {0.0, 0.0, 0.0};


                    if (g_SamplingMethod == Direction)
                        directional_sampling(x, ray_hit, pixel_color);
                    else if(g_SamplingMethod == FacePoint)
                        surface_sampling(x, totalTriangleArea, ray_hit, pixel_color);

                    for (int i = 0; i < 3; i++) {
                        g_AccumulationBuffer[pixel_idx * 3 + i] += pixel_color[i];
                    }
                    g_CountBuffer[pixel_idx] += g_SampleNum;
                    g_NumOfSample = g_CountBuffer[pixel_idx];

                }
            } else {
                const Eigen::Vector3d pixel_color = rgbNormalize(Eigen::Vector3d{40, 40, 40});
                for (int i = 0; i < 3; i++) {
                    g_AccumulationBuffer[pixel_idx * 3 + i] += pixel_color[i];
                }
                g_CountBuffer[pixel_idx] += 1;
            }
        }
    }
    g_Film.update();
    glutPostRedisplay();

}

void drawTriangleObj(const TriangleObj &rect) {
    glBegin(GL_TRIANGLES);
    glColor3f(rect.color(0), rect.color(1), rect.color(2));

    glVertex3f(rect.v1.x(), rect.v1.y(), rect.v1.z());
    glVertex3f(rect.v2.x(), rect.v2.y(), rect.v2.z());
    glVertex3f(rect.v3.x(), rect.v3.y(), rect.v3.z());

    glEnd();
}

void mouseDrag(int x, int y) {
    int _dx = x - g_MousePosX, _dy = y - g_MousePosY;
    g_MousePosX = x;
    g_MousePosY = y;

    double dx = double(_dx) / double(g_WindowWidth);
    double dy = -double(_dy) / double(g_WindowHeight);

    if (g_MouseMode == MM_CAMERA) {
        double scale = 2.0;

        g_Camera.rotateCameraInLocalFrameFixLookAt(dx * scale);
        g_Film.reset();
        g_Film.update();
        glutPostRedisplay();
    }
}

void mouseDown(int x, int y) {
    g_MousePosX = x;
    g_MousePosY = y;
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
        mouseDown(x, y);
}

void key(unsigned char key, int x, int y) {
    switch (key) {
        case 'C':
        case 'c':
            g_MouseMode = MM_CAMERA;
            break;
        case 'f':
        case 'F':
            g_DrawFilm = !g_DrawFilm;
            glutPostRedisplay();
            break;
        case 'p':
        case 'P':
            std::string filePath;
            if (g_SamplingMethod == Direction)
                filePath = "./directionalSampling_" + std::to_string(g_NumOfSample) + ".ppm";
            else if(g_SamplingMethod == FacePoint)
                filePath = "./facePointSampling_" + std::to_string(g_NumOfSample) + ".ppm";
            g_Film.savePpm(filePath, g_NumOfSample);
            break;
    }
}

void projection_and_modelview(const Camera &in_Camera) {
    const double fovy_deg = (2.0 * 180.0 / M_PI) *
                            atan(0.024 * 0.5 / in_Camera.getFocalLength());

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fovy_deg, double(g_WindowWidth) / double(g_WindowHeight), 0.01 * in_Camera.getFocalLength(), 1000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    const Eigen::Vector3d lookAtPoint = in_Camera.getLookAtPoint();
    gluLookAt(in_Camera.getEyePoint().x(), in_Camera.getEyePoint().y(), in_Camera.getEyePoint().z(), lookAtPoint.x(), lookAtPoint.y(), lookAtPoint.z(), in_Camera.getYVector().x(), in_Camera.getYVector().y(), in_Camera.getYVector().z());
}

void display() {
    glViewport(0, 0, g_WindowWidth * g_FrameSize_WindowSize_Scale_x, g_WindowHeight * g_FrameSize_WindowSize_Scale_y);

    glClearColor(0.0, 0.0, 1.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    projection_and_modelview(g_Camera);

    glEnable(GL_DEPTH_TEST);

    Learn();

    if (g_DrawFilm)
        g_Film.draw(g_Camera);

    for (int i = 0; i < TRIANGLE_NUM; i++) {
        drawTriangleObj(g_Triangles[i]);
    }

    glDisable(GL_DEPTH_TEST);

    glutSwapBuffers();
}

void resize(int w, int h) {
    g_WindowWidth = w;
    g_WindowHeight = h;
}

int main(int argc, char *argv[]) {
    g_Camera.setEyePoint(Eigen::Vector3d{6.0, 1.5, 1.0});
    g_Camera.lookAt(Eigen::Vector3d{0.0, 0.5, -1.25}, Eigen::Vector3d{0.0, 1.0, 0.0});

    glutInit(&argc, argv);
    glutInitWindowSize(g_WindowWidth, g_WindowHeight);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);

    glutCreateWindow("Hello world!!");

    // With retina display, frame buffer size is twice the window size.
    // Viewport size should be set on the basis of the frame buffer size, rather than the window size.
    // g_FrameSize_WindowSize_Scale_x and g_FrameSize_WindowSize_Scale_y account for this factor.
    GLint dims[4] = {0};
    glGetIntegerv(GL_VIEWPORT, dims);
    g_FrameSize_WindowSize_Scale_x = double(dims[2]) / double(g_WindowWidth);
    g_FrameSize_WindowSize_Scale_y = double(dims[3]) / double(g_WindowHeight);

    g_Triangles[0] = TriangleObj(Eigen::Vector3d{-2.0, 2.25, -2.0},
                                 Eigen::Vector3d{-3.0, 0.0, -2.0},
                                 Eigen::Vector3d{0.0, 0.0, -2.0},
                                 Eigen::Vector3d{215.0, 14.0, 74.0},
                                 "Red",
                                 true,
                                 1.0);
    g_Triangles[1] = TriangleObj(Eigen::Vector3d{0.25, 0.0, -2.0},
                                 Eigen::Vector3d{0.5, 1.5, -2.0},
                                 Eigen::Vector3d{-1.5, 2.25, -2.0},
                                 Eigen::Vector3d{1.0, 195.0, 215.0},
                                 "Blue",
                                 true,
                                 1.0);
    g_Triangles[2] = TriangleObj(Eigen::Vector3d{1.5, 2.5, -2.0},
                                 Eigen::Vector3d{0.75, 1.0, -2.0},
                                 Eigen::Vector3d{1.75, 0.0, -2.0},
                                 Eigen::Vector3d{50.0, 205.0, 50.0},
                                 "Green",
                                 true,
                                 1.0);
    g_Triangles[3] = TriangleObj(Eigen::Vector3d{0.75, 0.75, -2.0},
                                 Eigen::Vector3d{0.5, 0.0, -2.0},
                                 Eigen::Vector3d{1.5, 0.0, -2.0},
                                 Eigen::Vector3d{255.0, 244.0, 1.0},
                                 "Yellow",
                                 true,
                                 1.0);
    g_Triangles[4] = TriangleObj(Eigen::Vector3d{2.0, 1.5, -2.0},
                                 Eigen::Vector3d{2.0, 0.0, -2.0},
                                 Eigen::Vector3d{3.0, 0.0, -2.0},
                                 Eigen::Vector3d{147.0, 112.0, 219.0},
                                 "Purple",
                                 true,
                                 1.0);
    g_Triangles[5] = TriangleObj(Eigen::Vector3d{0.5, 0.25, 0.5},
                                 Eigen::Vector3d{0.25, 0.0, 0.5},
                                 Eigen::Vector3d{0.75, 0.0, 0.25},
                                 Eigen::Vector3d{255.0, 255.0, 255.0},
                                 "White",
                                 true,
                                 1.0);
    g_Triangles[6] = TriangleObj(Eigen::Vector3d{-1.0, 1.5, 1.25},
                                 Eigen::Vector3d{-1.0, 0.0, 1.25},
                                 Eigen::Vector3d{-1.75, 0.0, 0.5},
                                 Eigen::Vector3d{210.0, 105.0, 30.0},
                                 "Orange",
                                 true,
                                 1.0);
    g_Triangles[7] = TriangleObj(Eigen::Vector3d{0.0, 0.0, 1.5},
                                 Eigen::Vector3d{3.0, 0.0, -1.5},
                                 Eigen::Vector3d{-3.0, 0.0, -1.5},
                                 Eigen::Vector3d{255.0, 255.0, 255.0},
                                 "Diffusion_Surface_1",
                                 false,
                                 1.0);
    g_Triangles[8] = TriangleObj(Eigen::Vector3d{-1.0, 0.0, -1.0},
                                 Eigen::Vector3d{-0.5, 0.5, -1.0},
                                 Eigen::Vector3d{-1.5, 0.5, -1.0},
                                 Eigen::Vector3d{255.0, 255.0, 255.0},
                                 "Diffusion_Surface_2",
                                 false,
                                 1.0);

    CalcAdoptionRatio();

    std::srand(time(NULL));

    glutDisplayFunc(display);
    glutReshapeFunc(resize);
    glutMouseFunc(mouse);
    glutMotionFunc(mouseDrag);
    glutKeyboardFunc(key);
    g_Film.init();
    g_Film.clearRayTracedResult();
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glutMainLoop();
    return 0;
}
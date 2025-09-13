// YuvToRgbConverter.h
#pragma once
#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <media/NdkImage.h>
#include <opencv2/core.hpp> // For cv::Mat as output

class YuvToRgbConverter {
public:
    YuvToRgbConverter();
    ~YuvToRgbConverter();

    bool initialize(int width, int height);
    void release();
    // The main conversion function
    bool convert(AImage* image, cv::Mat& outRgbMat);

private:
    bool initEGL();
    GLuint loadShader(GLenum type, const char* shaderSrc);
    GLuint createProgram(const char* vertexSrc, const char* fragmentSrc);
    void setupTextures();
    void setupFBO();

    int imageWidth_ = 0;
    int imageHeight_ = 0;

    // EGL
    EGLDisplay display_ = EGL_NO_DISPLAY;
    EGLContext context_ = EGL_NO_CONTEXT;
    EGLSurface surface_ = EGL_NO_SURFACE;

    // OpenGL programs
    GLuint programI420_ = 0;
    GLuint programNV12_ = 0; // Or NV21

    // Textures
    GLuint yTexture_ = 0;
    GLuint uTexture_ = 0;
    GLuint vTexture_ = 0;
    GLuint uvTexture_ = 0;

    // FBO for offscreen rendering
    GLuint fbo_ = 0;
    GLuint rgbTexture_ = 0;

    // Vertex data
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
};


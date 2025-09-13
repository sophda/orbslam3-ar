
// YuvToRgbConverter.cpp
#include "glyuv2rgb.hpp"
#include <android/log.h>
#define LOG_TAG "yuv2rgb"
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

// 顶点着色器
const char* VERTEX_SHADER = R"(#version 300 es
layout (location = 0) in vec4 aPosition;
layout (location = 1) in vec2 aTexCoord;
out vec2 vTexCoord;
void main() {
    gl_Position = aPosition;
    vTexCoord = aTexCoord;
}
)";

// I420 片段着色器
const char* FRAGMENT_SHADER_I420 = R"(#version 300 es
precision mediump float;
in vec2 vTexCoord;
out vec4 fragColor;
uniform sampler2D y_texture;
uniform sampler2D u_texture;
uniform sampler2D v_texture;
const mat3 yuv2rgb = mat3(
    1.164,  1.164, 1.164,
    0.0,   -0.392, 2.017,
    1.596, -0.813, 0.0
);
void main() {
    vec3 yuv;
    yuv.x = texture(y_texture, vTexCoord).r - (16.0/255.0);
    yuv.y = texture(u_texture, vTexCoord).r - 0.5;
    yuv.z = texture(v_texture, vTexCoord).r - 0.5;
    vec3 rgb = yuv2rgb * yuv;
    fragColor = vec4(rgb, 1.0);
}
)";

// NV12 片段着色器
const char* FRAGMENT_SHADER_NV12 = R"(#version 300 es
precision mediump float;
in vec2 vTexCoord;
out vec4 fragColor;
uniform sampler2D y_texture;
uniform sampler2D uv_texture;
const mat3 yuv2rgb = mat3(
    1.164,  1.164, 1.164,
    0.0,   -0.392, 2.017,
    1.596, -0.813, 0.0
);
void main() {
    vec3 yuv;
    yuv.x = texture(y_texture, vTexCoord).r - (16.0/255.0);
    vec2 uv = texture(uv_texture, vTexCoord).rg;
    yuv.y = uv.y - 0.5; // U
    yuv.z = uv.x - 0.5; // V
    vec3 rgb = yuv2rgb * yuv;
    fragColor = vec4(rgb, 1.0);
}
)";

YuvToRgbConverter::YuvToRgbConverter() {
    // Constructor
}

YuvToRgbConverter::~YuvToRgbConverter() {
    release();
}

bool YuvToRgbConverter::initialize(int width, int height) {
    imageWidth_ = width;
    imageHeight_ = height;

    if (!initEGL()) {
        LOGE("Failed to initialize EGL");
        return false;
    }

    // Load shaders and create programs
    programI420_ = createProgram(VERTEX_SHADER, FRAGMENT_SHADER_I420);
    if (!programI420_) return false;
    programNV12_ = createProgram(VERTEX_SHADER, FRAGMENT_SHADER_NV12);
    if (!programNV12_) return false;

    setupTextures();
    setupFBO();

    // Setup VAO and VBO for drawing a quad
    const float vertices[] = {
        // 顶点位置 (x, y, z)   // 纹理坐标 (u, v)
        -1.0f,  1.0f, 0.0f,    0.0f, 1.0f, // 左上顶点 -> 映射到纹理左上角(0,1)
        -1.0f, -1.0f, 0.0f,    0.0f, 0.0f, // 左下顶点 -> 映射到纹理左下角(0,0)
        1.0f, -1.0f, 0.0f,    1.0f, 0.0f, // 右下顶点 -> 映射到纹理右下角(1,0)
        1.0f,  1.0f, 0.0f,    1.0f, 1.0f  // 右上顶点 -> 映射到纹理右上角(1,1)
    };
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glBindVertexArray(0);

    LOGI("YuvToRgbConverter initialized successfully for %dx%d", width, height);
    return true;
}

void YuvToRgbConverter::release() {
    if (display_ != EGL_NO_DISPLAY) {
        eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (context_ != EGL_NO_CONTEXT) eglDestroyContext(display_, context_);
        if (surface_ != EGL_NO_SURFACE) eglDestroySurface(display_, surface_);
        eglTerminate(display_);
    }
    display_ = EGL_NO_DISPLAY;
    context_ = EGL_NO_CONTEXT;
    surface_ = EGL_NO_SURFACE;
    // ... release all GL resources (programs, textures, FBO, VBO, VAO)
}

bool YuvToRgbConverter::convert(AImage* image, cv::Mat& outRgbMat) {
    
    int32_t width, height;
    AImage_getWidth(image, &width);
    AImage_getHeight(image, &height);

    // 添加这个关键的日志和检查
    LOGI("AImage runtime dimensions: %d x %d. Converter was initialized for: %d x %d", 
         width, height, imageWidth_, imageHeight_);
    
    int32_t format;
    AImage_getFormat(image, &format);

    if (format != AIMAGE_FORMAT_YUV_420_888) {
        LOGE("Unsupported image format: %d", format);
        return false;
    }

    uint8_t *yBuffer, *uBuffer, *vBuffer;
    int32_t yLen, uLen, vLen;
    int32_t yRowStride, uRowStride, vRowStride;
    int32_t yPixelStride, uPixelStride, vPixelStride;

    AImage_getPlaneData(image, 0, &yBuffer, &yLen);
    AImage_getPlaneData(image, 1, &uBuffer, &uLen);
    AImage_getPlaneData(image, 2, &vBuffer, &vLen);

    AImage_getPlaneRowStride(image, 0, &yRowStride);
    AImage_getPlaneRowStride(image, 1, &uRowStride);
    AImage_getPlaneRowStride(image, 2, &vRowStride);

    AImage_getPlanePixelStride(image, 1, &uPixelStride);

    // Make EGL context current
    eglMakeCurrent(display_, surface_, surface_, context_);
    
    glViewport(0, 0, imageWidth_, imageHeight_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glClear(GL_COLOR_BUFFER_BIT);

    // Check if format is I420 (fully planar) or NV12/NV21 (semi-planar)
    bool isPlanar = (uPixelStride == 1);

    if (isPlanar) { // I420
        glUseProgram(programI420_);

        // Upload Y plane
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, yTexture_);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, yRowStride);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, imageWidth_, imageHeight_, GL_LUMINANCE, GL_UNSIGNED_BYTE, yBuffer);
        glUniform1i(glGetUniformLocation(programI420_, "y_texture"), 0);

        // Upload U plane
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, uTexture_);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, uRowStride);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, imageWidth_ / 2, imageHeight_ / 2, GL_LUMINANCE, GL_UNSIGNED_BYTE, uBuffer);
        glUniform1i(glGetUniformLocation(programI420_, "u_texture"), 1);

        // Upload V plane
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, vTexture_);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, vRowStride);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, imageWidth_ / 2, imageHeight_ / 2, GL_LUMINANCE, GL_UNSIGNED_BYTE, vBuffer);
        glUniform1i(glGetUniformLocation(programI420_, "v_texture"), 2);

    } else { // NV12 or NV21
        glUseProgram(programNV12_);

        // --- 上传 Y 平面 (这部分应该没问题) ---
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, yTexture_);
        AImage_getPlaneRowStride(image, 0, &yRowStride);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, yRowStride); 
        AImage_getPlaneData(image, 0, &yBuffer, &yLen);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, imageWidth_, imageHeight_, GL_RED, GL_UNSIGNED_BYTE, yBuffer);
        glUniform1i(glGetUniformLocation(programNV12_, "y_texture"), 0);


        // --- 上传 UV 平面 (关键修正) ---
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, uvTexture_);
        AImage_getPlaneRowStride(image, 1, &uRowStride); // 获取UV平面的字节跨距
        AImage_getPlaneData(image, 1, &uBuffer, &uLen);
        
        // 关键修正：GL_UNPACK_ROW_LENGTH 需要的是像素个数，而不是字节数。
        // 因为我们的格式是 GL_RG (每个像素2字节)，所以像素跨距是字节跨距的一半。
        glPixelStorei(GL_UNPACK_ROW_LENGTH, uRowStride / 2);
        
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, imageWidth_ / 2, imageHeight_ / 2, GL_RG, GL_UNSIGNED_BYTE, uBuffer);
        glUniform1i(glGetUniformLocation(programNV12_, "uv_texture"), 1);
    }
    
    // Reset pixel store alignment
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

    // Draw the quad
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glBindVertexArray(0);

    // Read the result back to CPU memory
    if (outRgbMat.empty() || outRgbMat.cols != imageWidth_ || outRgbMat.rows != imageHeight_) {
        outRgbMat.create(imageHeight_, imageWidth_, CV_8UC3);
    }
    glReadPixels(0, 0, imageWidth_, imageHeight_, GL_RGB, GL_UNSIGNED_BYTE, outRgbMat.data);
    
    // Unbind FBO
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    
    return true;
}

void YuvToRgbConverter::setupTextures() {
    LOGI("Setting up textures...");
    // --- 为 I420 (planar) 格式创建纹理 ---
    int uvWidth = imageWidth_ / 2;
    int uvHeight = imageHeight_ / 2;

    // 1. Y 纹理
    glGenTextures(1, &yTexture_);
    glBindTexture(GL_TEXTURE_2D, yTexture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    // 使用 GL_R8 作为单通道纹理的内部格式 (GLES 3.0+)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, imageWidth_, imageHeight_, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);

    // 2. U 纹理
    glGenTextures(1, &uTexture_);
    glBindTexture(GL_TEXTURE_2D, uTexture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, uvWidth, uvHeight, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);

    // 3. V 纹理
    glGenTextures(1, &vTexture_);
    glBindTexture(GL_TEXTURE_2D, vTexture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, uvWidth, uvHeight, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);

    // --- 为 NV12/NV21 (semi-planar) 格式创建纹理 ---
    // 4. UV 交错纹理
    glGenTextures(1, &uvTexture_);
    glBindTexture(GL_TEXTURE_2D, uvTexture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    // 使用 GL_RG8 作为双通道纹理的内部格式
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, uvWidth, uvHeight, 0, GL_RG, GL_UNSIGNED_BYTE, nullptr);

    // 解绑纹理
    glBindTexture(GL_TEXTURE_2D, 0);
    LOGI("Textures created successfully.");
}

void YuvToRgbConverter::setupFBO() {
    LOGI("Setting up Framebuffer Object (FBO)...");
    // 1. 创建 FBO
    glGenFramebuffers(1, &fbo_);

    // 2. 创建一个纹理，作为 FBO 的颜色附件（渲染目标）
    glGenTextures(1, &rgbTexture_);
    glBindTexture(GL_TEXTURE_2D, rgbTexture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    // 分配存储空间，格式为 RGB
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, imageWidth_, imageHeight_, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);

    // 3. 将纹理附加到 FBO
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, rgbTexture_, 0);

    // 4. 检查 FBO 状态，这是非常重要的调试步骤
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        LOGE("Failed to create complete FBO: 0x%x", status);
    } else {
        LOGI("FBO created successfully.");
    }

    // 5. 解绑 FBO，返回到默认的窗口系统帧缓冲
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}


GLuint YuvToRgbConverter::loadShader(GLenum type, const char* shaderSrc) {
    GLuint shader = glCreateShader(type);
    if (shader == 0) {
        LOGE("Could not create shader of type %d", type);
        return 0;
    }

    glShaderSource(shader, 1, &shaderSrc, nullptr);
    glCompileShader(shader);

    GLint compiled;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        GLint infoLen = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);
        if (infoLen > 1) {
            char* infoLog = (char*)malloc(sizeof(char) * infoLen);
            glGetShaderInfoLog(shader, infoLen, nullptr, infoLog);
            LOGE("Error compiling shader:\n%s", infoLog);
            free(infoLog);
        }
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

// 缺失的函数实现 1: createProgram
GLuint YuvToRgbConverter::createProgram(const char* vertexSrc, const char* fragmentSrc) {
    GLuint vertexShader = loadShader(GL_VERTEX_SHADER, vertexSrc);
    if (vertexShader == 0) {
        LOGE("Failed to load vertex shader");
        return 0;
    }

    GLuint fragmentShader = loadShader(GL_FRAGMENT_SHADER, fragmentSrc);
    if (fragmentShader == 0) {
        LOGE("Failed to load fragment shader");
        glDeleteShader(vertexShader);
        return 0;
    }

    GLuint program = glCreateProgram();
    if (program == 0) {
        LOGE("Could not create program");
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        return 0;
    }

    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    GLint linked;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (!linked) {
        GLint infoLen = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLen);
        if (infoLen > 1) {
            char* infoLog = (char*)malloc(sizeof(char) * infoLen);
            glGetProgramInfoLog(program, infoLen, nullptr, infoLog);
            LOGE("Error linking program:\n%s", infoLog);
            free(infoLog);
        }
        glDeleteProgram(program);
        return 0;
    }

    // 链接成功后，可以删除着色器对象
    glDetachShader(program, vertexShader);
    glDetachShader(program, fragmentShader);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return program;
}

// 缺失的函数实现 2: initEGL
bool YuvToRgbConverter::initEGL() {
    display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (display_ == EGL_NO_DISPLAY) {
        LOGE("eglGetDisplay failed: %d", eglGetError());
        return false;
    }

    EGLint major, minor;
    if (!eglInitialize(display_, &major, &minor)) {
        LOGE("eglInitialize failed: %d", eglGetError());
        return false;
    }
     LOGI("EGL initialized, version %d.%d", major, minor);

    // 配置属性
    const EGLint configAttribs[] = {
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, // 我们需要离屏渲染
            EGL_RED_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_BLUE_SIZE, 8,
            EGL_ALPHA_SIZE, 8,
            EGL_NONE
    };

    EGLConfig config;
    EGLint numConfigs;
    if (!eglChooseConfig(display_, configAttribs, &config, 1, &numConfigs)) {
        LOGE("eglChooseConfig failed: %d", eglGetError());
        return false;
    }

    // 创建一个 Pbuffer Surface
    const EGLint pbufferAttribs[] = {
            EGL_WIDTH, imageWidth_,
            EGL_HEIGHT, imageHeight_,
            EGL_NONE
    };
    surface_ = eglCreatePbufferSurface(display_, config, pbufferAttribs);
    if (surface_ == EGL_NO_SURFACE) {
        LOGE("eglCreatePbufferSurface failed: %d", eglGetError());
        return false;
    }

    // 创建 EGL 上下文
    const EGLint contextAttribs[] = {
            EGL_CONTEXT_CLIENT_VERSION, 3, // 请求 GLES 3.x
            EGL_NONE
    };
    context_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, contextAttribs);
    if (context_ == EGL_NO_CONTEXT) {
        LOGE("eglCreateContext failed: %d", eglGetError());
        return false;
    }

    if (!eglMakeCurrent(display_, surface_, surface_, context_)) {
        LOGE("eglMakeCurrent failed: %d", eglGetError());
        return false;
    }

    LOGI("EGL context created and made current successfully");
    return true;
}

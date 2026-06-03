#include <EGL/egl.h>
#include <stdio.h>
#include <stdlib.h>

int main() {
    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (display == EGL_NO_DISPLAY) {
        fprintf(stderr, "Failed to get EGL display\n");
        return 1;
    }

    EGLint major, minor;
    if (!eglInitialize(display, &major, &minor)) {
        fprintf(stderr, "Failed to initialize EGL\n");
        return 1;
    }
    printf("EGL initialized: %d.%d\n", major, minor);

    const char* vendor = eglQueryString(display, EGL_VENDOR);
    printf("EGL Vendor: %s\n", vendor);

    EGLConfig config;
    EGLint numConfigs;
    EGLint configAttribs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT | EGL_WINDOW_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_NONE
    };

    if (!eglChooseConfig(display, configAttribs, &config, 1, &numConfigs) || numConfigs == 0) {
        printf("Failed with full attributes, trying minimal...\n");
        EGLint minimalAttribs[] = { EGL_NONE };
        if (!eglChooseConfig(display, minimalAttribs, &config, 1, &numConfigs) || numConfigs == 0) {
            fprintf(stderr, "Failed to choose ANY EGL config\n");
            return 1;
        }
    }
    printf("Found %d compatible configs.\n", numConfigs);

    eglBindAPI(EGL_OPENGL_API);

    EGLContext context = eglCreateContext(display, config, EGL_NO_CONTEXT, NULL);
    if (context == EGL_NO_CONTEXT) {
        fprintf(stderr, "Failed to create EGL context\n");
        return 1;
    }

    if (!eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context)) {
        EGLint err = eglGetError();
        fprintf(stderr, "Failed to make EGL context current (Error: 0x%x)\n", err);
        return 1;
    }

    printf("SUCCESS: EGL context is current!\n");
    
    eglDestroyContext(display, context);
    eglTerminate(display);
    return 0;
}

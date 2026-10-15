#include "render/GlCanvas.h"

#if IBM
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#if APL
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <cstddef>

#include "XPLMGraphics.h"

namespace navsight::render {
namespace {

// GL 1.5 / 3.0 entry points are not exported by opengl32.dll; they are looked
// up at runtime. Only used to make sure no buffer object / vertex array object
// is bound while we use client-side vertex arrays, and to restore X-Plane's.
constexpr GLenum kArrayBuffer = 0x8892;           // GL_ARRAY_BUFFER
constexpr GLenum kArrayBufferBinding = 0x8894;    // GL_ARRAY_BUFFER_BINDING
constexpr GLenum kVertexArrayBinding = 0x85B5;    // GL_VERTEX_ARRAY_BINDING
#if IBM
using BindBufferFn = void(APIENTRY*)(GLenum, GLuint);
using BindVertexArrayFn = void(APIENTRY*)(GLuint);
#else
using BindBufferFn = void (*)(GLenum, GLuint);
using BindVertexArrayFn = void (*)(GLuint);
#endif

struct GlEntryPoints {
    bool loaded = false;
    BindBufferFn bindBuffer = nullptr;
    BindVertexArrayFn bindVertexArray = nullptr;
};

GlEntryPoints& gl() {
    static GlEntryPoints e;
    if (!e.loaded) {
        e.loaded = true;
#if IBM
        e.bindBuffer = reinterpret_cast<BindBufferFn>(wglGetProcAddress("glBindBuffer"));
        e.bindVertexArray = reinterpret_cast<BindVertexArrayFn>(wglGetProcAddress("glBindVertexArray"));
#endif
    }
    return e;
}

}  // namespace

bool GlCanvas::s_fastPath = true;

void GlCanvas::flush() {
    lastTriangles_ = batch_.size() / 3;
    if (batch_.empty()) return;

    // No fog, no textures, no lighting, no alpha test, alpha blend on, no depth.
    // XPLMSetGraphicsState is the SDK-sanctioned way to change this state; it
    // keeps X-Plane's state cache consistent.
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Canvas makes no winding-order promises; make sure culling can't eat
    // triangles, and put the state back the way X-Plane had it.
    const GLboolean cullWasOn = glIsEnabled(GL_CULL_FACE);
    if (cullWasOn) glDisable(GL_CULL_FACE);

    // Fast path: one glDrawArrays from client memory instead of two GL calls
    // per vertex. Requires no VBO/VAO bound; if X-Plane left one bound and we
    // can't unbind/restore it, fall back to immediate mode for this frame.
    GLint prevVbo = 0, prevVao = 0;
    bool arrays = s_fastPath;
    const GlEntryPoints& e = gl();
    if (arrays && e.bindBuffer) {
        glGetIntegerv(kArrayBufferBinding, &prevVbo);
        if (e.bindVertexArray) glGetIntegerv(kVertexArrayBinding, &prevVao);
    }
    if (arrays && prevVbo != 0) {
        if (e.bindBuffer) e.bindBuffer(kArrayBuffer, 0);
        else arrays = false;
    }
    if (arrays && prevVao != 0) {
        if (e.bindVertexArray) e.bindVertexArray(0);
        else arrays = false;
    }

    if (arrays) {
        glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        const GLsizei stride = static_cast<GLsizei>(sizeof(Vertex));
        glVertexPointer(2, GL_FLOAT, stride, &batch_[0].pos);
        glColorPointer(4, GL_FLOAT, stride, &batch_[0].color);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch_.size()));
        glPopClientAttrib();
        if (prevVao != 0) e.bindVertexArray(static_cast<GLuint>(prevVao));
        if (prevVbo != 0) e.bindBuffer(kArrayBuffer, static_cast<GLuint>(prevVbo));
    } else {
        glBegin(GL_TRIANGLES);
        for (const Vertex& v : batch_) {
            glColor4f(v.color.r, v.color.g, v.color.b, v.color.a);
            glVertex2f(v.pos.x, v.pos.y);
        }
        glEnd();
    }
    lastUsedFastPath_ = arrays;

    if (cullWasOn) glEnable(GL_CULL_FACE);

    batch_.clear();  // keeps capacity: no reallocation next frame
}

}  // namespace navsight::render

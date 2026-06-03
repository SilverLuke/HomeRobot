import sys

def patch_support():
    path = "../ogre-next/RenderSystems/GL3Plus/src/windowing/EGL/PBuffer/OgreEglPBufferSupport.cpp"
    with open(path, 'r') as f:
        lines = f.readlines()
    
    start = -1
    for i, line in enumerate(lines):
        if "void EglPBufferSupport::initDevice" in line:
            start = i
            break
    
    if start == -1: return
    
    end = -1
    for i in range(start, len(lines)):
        if lines[i].startswith("    }"):
            end = i
            break
            
    if end == -1: return
    
    new_init = [
        "    void EglPBufferSupport::initDevice( const size_t deviceIdx )\n",
        "    {\n",
        "        DeviceData &deviceData = mDeviceData[deviceIdx];\n",
        "        LogManager::getSingleton().logMessage( \"Trying to init device: \" + deviceData.name + \"...\" );\n",
        "        EGLAttrib attribs[] = { EGL_NONE };\n",
        "        deviceData.eglDisplay = eglGetPlatformDisplay( EGL_PLATFORM_DEVICE_EXT, mDevices[deviceIdx], attribs );\n",
        "        EGL_CHECK_ERROR\n",
        "        if( deviceData.eglDisplay == EGL_NO_DISPLAY ) {\n",
        "            OGRE_EXCEPT( Exception::ERR_RENDERINGAPI_ERROR, \"eglGetPlatformDisplay failed for device \" + deviceData.name, \"EGLSupport::getGLDisplay\" );\n",
        "        }\n",
        "        EGLint major = 0, minor = 0;\n",
        "        if( eglInitialize( deviceData.eglDisplay, &major, &minor ) == EGL_FALSE ) {\n",
        "            OGRE_EXCEPT( Exception::ERR_RENDERINGAPI_ERROR, \"eglInitialize failed for device \" + deviceData.name, \"EGLSupport::getGLDisplay\" );\n",
        "        }\n",
        "        EGL_CHECK_ERROR\n",
        "        const EGLint configAttribs[] = {\n",
        "            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_BLUE_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_RED_SIZE, 8,\n",
        "            EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE\n",
        "        };\n",
        "        EGLint numConfigs;\n",
        "        if( eglChooseConfig( deviceData.eglDisplay, configAttribs, &deviceData.eglCfg, 1, &numConfigs ) == EGL_FALSE ) {\n",
        "            OGRE_EXCEPT( Exception::ERR_RENDERINGAPI_ERROR, \"eglChooseConfig for device \" + deviceData.name, \"EGLSupport::getGLDisplay\" );\n",
        "        }\n",
        "    }\n"
    ]
    
    # Remove escaped backslashes for python internal use
    new_init = [l.replace("\\", "\") for l in new_init]
    
    lines[start:end+1] = [l.replace("\n", "\n").replace('\"', '"') for l in new_init]
    
    with open(path, 'w') as f:
        f.writelines(lines)

def patch_context():
    cpp_path = "../ogre-next/RenderSystems/GL3Plus/src/windowing/EGL/PBuffer/OgreEglPBufferContext.cpp"
    
    with open(cpp_path, 'w') as f:
        f.write('''
#include "PBuffer/OgreEglPBufferContext.h"
#include "OgreGL3PlusRenderSystem.h"
#include "OgreRoot.h"

namespace Ogre
{
    EglPBufferContext::EglPBufferContext( EglPBufferSupport *support ) :
        mGLSupport( support ),
        mDeviceData( support->getCurrentDevice() ),
        mContext( 0 ),
        mSurface( 0 )
    {
    }

    EglPBufferContext::~EglPBufferContext()
    {
        endCurrent();
        if( mContext ) eglDestroyContext( mDeviceData->eglDisplay, mContext );
        if( mSurface ) eglDestroySurface( mDeviceData->eglDisplay, mSurface );
        
        GL3PlusRenderSystem *rs = static_cast<GL3PlusRenderSystem *>( Root::getSingleton().getRenderSystem() );
        if (rs) rs->_unregisterContext( this );
    }

    void EglPBufferContext::setCurrent()
    {
        if( !mContext )
        {
            const EGLint pbufferAttribs[] = { EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE };
            mSurface = eglCreatePbufferSurface( mDeviceData->eglDisplay, mDeviceData->eglCfg, pbufferAttribs );

            eglBindAPI( EGL_OPENGL_API );
            EGLint contextAttrs[] = {
                EGL_CONTEXT_MAJOR_VERSION, 4,
                EGL_CONTEXT_MINOR_VERSION, 5,
                EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT_KHR,
                EGL_NONE
            };

            while( !mContext && ( contextAttrs[1] >= 3 ) )
            {
                mContext = eglCreateContext( mDeviceData->eglDisplay, mDeviceData->eglCfg, 0, contextAttrs );
                if( !mContext )
                {
                    if( contextAttrs[3] == 0 ) { contextAttrs[1] -= 1; contextAttrs[3] = 5; }
                    else { contextAttrs[3] -= 1; }
                }
            }
        }
        eglMakeCurrent( mDeviceData->eglDisplay, mSurface, mSurface, mContext );
    }

    void EglPBufferContext::endCurrent() 
    { 
        eglMakeCurrent( mDeviceData->eglDisplay, 0, 0, 0 ); 
    }

    GL3PlusContext *EglPBufferContext::clone() const
    {
        return new EglPBufferContext( mGLSupport );
    }
}
''')

patch_support()
patch_context()

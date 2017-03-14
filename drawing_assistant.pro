#-------------------------------------------------
#
# Project created by QtCreator 2015-09-16T19:06:57
#
#-------------------------------------------------

QT += core multimedia xml opengl widgets
CONFIG += c++11
CONFIG += console

macx {
   QMAKE_MAC_SDK = macosx10.12
}

#QMAKE_RPATHDIR +=

#QMAKE_LFLAGS_SONAME += /Applications/MATLAB_R2014b.app/bin/maci64/

#QMAKE_LFLAGS += -Wl,-rpath,

QMAKE_LFLAGS_SONAME     = -Wl,-install_name,@rpath/


INCLUDEPATH += Eigen \
               /usr/local/include/ \
               unsupported \
               igl \
               /Applications/MATLAB_R2014b.app/extern/include/ \
               /Library/gurobi651/mac64/include \

LIBS += -L/usr/local/lib/  \
        -lCGAL \
        -lgmp \
        -lmpfr \
        -L/Library/gurobi651/mac64/lib \
        -lgurobi65 \
        -lgurobi_c++ \
        -lgurobi_g++4.2 \
        -lgurobi_stdc++ \
        -L/Applications/MATLAB_R2014b.app/bin/maci64/ \
        -leng \
        -lmat \
        -lmex \
        -lmx \
        -lm \
        -lut \
        -lstdc++ \

QMAKE_CXXFLAGS += -frounding-math -O3

greaterThan(QT_MINOR_VERSION, 5.4): QT += widgets

TARGET = drawing_assistant
TEMPLATE = app


SOURCES += main.cpp \
##WIDGETS
        widgets/mainwindow.cpp \
        widgets/viewerwidget.cpp \
        widgets/controlswidget.cpp \
        widgets/suggestivecontourspanel.cpp \
##MESH
        mesh/meshdata.cpp \
        mesh/meshsegment.cpp \
        mesh/mesh.cpp \
        mesh/meshsegmentcluster.cpp \
        mesh/segmentsgraph.cpp \
##OPENGL
        opengl/trackball.cpp \
##UTILS
        utils/contourparams.cpp \
        utils/matlabengine.cpp \
#TRIMESH
        trimesh/conn_comps.cc \
        trimesh/diffuse.cc \
        trimesh/edgeflip.cc \
        trimesh/faceflip.cc \
        trimesh/filter.cc \
        trimesh/GLCamera.cc \
        trimesh/ICP.cc \
        trimesh/KDtree.cc \
        trimesh/lmsmooth.cc \
        trimesh/overlap.cc \
        trimesh/remove.cc \
        trimesh/reorder_verts.cc \
        trimesh/shared.cc \
        trimesh/subdiv.cc \
        trimesh/TriMesh_bounding.cc \
        trimesh/TriMesh_connectivity.cc \
        trimesh/TriMesh_curvature.cc \
        trimesh/TriMesh_grid.cc \
        trimesh/TriMesh_io.cc \
        trimesh/TriMesh_normals.cc \
        trimesh/TriMesh_pointareas.cc \
        trimesh/TriMesh_stats.cc \
        trimesh/TriMesh_tstrips.cc \
        widgets/primitivespanel.cpp \
        optimisation/primitiveoptimisation.cpp \
    optimisation/utility_functions.cpp

#WIDGETS
HEADERS  +=  widgets/mainwindow.h \
        widgets/viewerwidget.h \
        widgets/controlswidget.h \
        widgets/suggestivecontourspanel.h \
#MESH
        mesh/meshdata.h \
        mesh/meshsegment.h \
        mesh/segmentsgraph.h \
        mesh/mesh.h \
        mesh/meshsegmentcluster.h \
#OPENGL
        opengl/trackball.h \
#UTILS
        utils/contourparams.h \
        utils/matlabengine.h \
#TRIMESH
        trimesh/vec.h \
        trimesh/lineqn.h \
        trimesh/Box.h \
        trimesh/bsphere.h \
        trimesh/Color.h \
        trimesh/GLCamera.h \
        trimesh/ICP.h \
        trimesh/KDtree.h \
        trimesh/mempool.h \
        trimesh/noise3d.h \
        trimesh/strutil.h \
        trimesh/timestamp.h \
        trimesh/TriMesh.h \
        trimesh/TriMesh_algo.h \
        trimesh/XForm.h \
        widgets/primitivespanel.h \
        optimisation/primitiveoptimisation.h \
        optimisation/utility_functions.h
        #mesh/meshsegmentcluster_old.h \



RESOURCES += \
    shaders.qrc

#Add directory for build files
DESTDIR = /Users/JamesHennessey/Projects/multimodal_drawing_assistant/build
OBJECTS_DIR = /Users/JamesHennessey/Projects/multimodal_drawing_assistant/build
MOC_DIR = /Users/JamesHennessey/Projects/multimodal_drawing_assistant/build
RCC_DIR = /Users/JamesHennessey/Projects/multimodal_drawing_assistant/build
UI_DIR = /Users/JamesHennessey/Projects/multimodal_drawing_assistant/build


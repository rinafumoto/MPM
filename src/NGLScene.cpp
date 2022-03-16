#include <QMouseEvent>
#include <QGuiApplication>

#include "NGLScene.h"
#include <ngl/NGLInit.h>
#include <ngl/ShaderLib.h>
#include <ngl/Util.h>
#include <iostream>


const auto ColourShader = "ColourShader";
const auto SolidShader = "SolidShader";

NGLScene::NGLScene(QWidget *_parent) : QOpenGLWidget(_parent)
{

}

////////// Functions called on a button click //////////

// initialise the simulation with the setting values from GUI and start timer.
void NGLScene::initialise()
{
  m_mpm = std::make_unique<MPM>();
  m_mpm->initialise();
  startTimer(10);
  update();
}

void NGLScene::step()
{
  m_mpm->simulate();
  update();
}

void NGLScene::start()
{
  sim=true;
}

void NGLScene::stop()
{
  sim=false;
}

NGLScene::~NGLScene()
{
  std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
}

void NGLScene::resizeGL(int _w , int _h)
{
  m_win.width  = static_cast<int>( _w * devicePixelRatio() );
  m_win.height = static_cast<int>( _h * devicePixelRatio() );
}

void NGLScene::initializeGL()
{
  ngl::NGLInit::initialize();
  // Set the background colour to white.
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  // Enable depth testing for drawing.
  glEnable(GL_DEPTH_TEST);
  // Load shaders
  ngl::ShaderLib::loadShader(ColourShader, "shaders/ColourVertex.glsl", "shaders/ColourFragment.glsl");
  ngl::ShaderLib::loadShader(SolidShader,"shaders/SolidVertex.glsl","shaders/SolidFragment.glsl","shaders/SolidGeometry.glsl");
  // initialise the simulation.
  initialise();
}

void NGLScene::timerEvent ( QTimerEvent *_event)
{
  // Simulate one step on timer event when the simulation has been started.
  if(sim)
  {
    m_mpm->simulate();
    update();
  }
}

void NGLScene::paintGL()
{
  // Clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0,0,m_win.width,m_win.height);
  m_mpm->render();
}

//----------------------------------------------------------------------------------------------------------------------

void NGLScene::keyPressEvent(QKeyEvent *_event)
{
  // this method is called every time the main window recives a key event.
  // we then switch on the key value and set the camera in the GLWindow
  switch (_event->key())
  {
  // escape key to quite
  case Qt::Key_Escape : QGuiApplication::exit(EXIT_SUCCESS); break;
  case Qt::Key_Space :
      m_win.spinXFace=0;
      m_win.spinYFace=0;
      m_modelPos.set(ngl::Vec3::zero());

  break;
  default : break;
  }
  // finally update the GLWindow and re-draw

    update();
}

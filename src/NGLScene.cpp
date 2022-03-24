#include <QMouseEvent>
#include <QGuiApplication>

#include "NGLScene.h"
#include <ngl/NGLInit.h>
#include <ngl/ShaderLib.h>
#include <ngl/Util.h>
#include <iostream>


const auto ColourShader = "ColourShader";
const auto SolidShader = "SolidShader";
const auto GridViz = "GridViz";

NGLScene::NGLScene(QWidget *_parent) : QOpenGLWidget(_parent)
{

}

////////// Functions called on a button click //////////

// initialise the simulation with the setting values from GUI.
void NGLScene::initialise()
{
  m_mpm = std::make_unique<MPM>();
  m_mpm->initialise(m_shape, m_pos, m_size, m_vel, m_hardening, m_density, m_youngs, m_poisson, m_compression, m_stretch, m_blending, m_gridsize, m_timestep, m_force, m_resolutionX, m_resolutionY);
  update();
}

void NGLScene::step()
{
  m_mpm->simulate();
  update();
}

void NGLScene::start()
{
  if(m_timer < 0)
  {
    m_timer = startTimer(10);  
  }
}

void NGLScene::stop()
{
  if(m_timer >= 0)
  {
    killTimer(m_timer);
    m_timer = -1;
  }
}

////////// Functions called on a value change //////////

void NGLScene::setShape(int i)
{
  m_shape = i;
}

void NGLScene::setPositionX(double d)
{
  m_pos.m_x = d;
}

void NGLScene::setPositionY(double d)
{
  m_pos.m_y = d;
}

void NGLScene::setSizeX(double d)
{
  m_size.m_x = d;
}

void NGLScene::setSizeY(double d)
{
  m_size.m_y = d;
}

void NGLScene::setVelocityX(double d)
{
  m_vel.m_x = d;
}

void NGLScene::setVelocityY(double d)
{
  m_vel.m_y = d;
}

void NGLScene::setHardening(double d)
{
  m_hardening = d;
}

void NGLScene::setDensity(double d)
{
  m_density = d;
}

void NGLScene::setYoungs(double d)
{
  m_youngs = d;
}

void NGLScene::setPoisson(double d)
{
  m_poisson = d;
}

void NGLScene::setCompression(double d)
{
  m_compression = d;
}

void NGLScene::setStretch(double d)
{
  m_stretch = d;
}

void NGLScene::setBlending(double d)
{
  m_blending = d;
}

void NGLScene::setGridSize(double d)
{
  m_gridsize = d;
}

void NGLScene::setTimeStep(double d)
{
  m_timestep = d;
}

void NGLScene::setForceX(double d)
{
  m_force.m_x = d;
}

void NGLScene::setForceY(double d)
{
  m_force.m_y = d;
}

void NGLScene::setResolutionX(int i)
{
  m_resolutionX = i;
}

void NGLScene::setResolutionY(int i)
{
  m_resolutionY = i;
}

void NGLScene::setParticleVel(bool b)
{
  m_particleVel = b;
}

void NGLScene::setGridVel(bool b)
{
  m_gridVel = b;
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
  ngl::ShaderLib::loadShader(GridViz,"shaders/VectorVizVertex.glsl","shaders/VectorVizFragment.glsl","shaders/VectorVizGeometry.glsl");
  // initialise the simulation.
  initialise();
}

void NGLScene::timerEvent ( QTimerEvent *_event)
{
  // Simulate one step on timer event when the simulation has been started.
    m_mpm->simulate();
    update();
}

void NGLScene::paintGL()
{
  // Clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0,0,m_win.width,m_win.height);
  m_mpm->render(m_win.width,m_win.height, m_particleVel, m_gridVel);
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

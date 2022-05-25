#include <QMouseEvent>
#include <QGuiApplication>
#include <QMessageBox>
#include <QFileDialog>

#include "NGLScene.h"
#include <ngl/NGLInit.h>
#include <ngl/ShaderLib.h>
#include <ngl/Util.h>
#include <iostream>
#include <fstream>

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
  if(m_playtimer >= 0)
  {
    killTimer(m_playtimer);
    m_playtimer = -1;        
  }
  m_mpm = std::make_unique<MPM>();
  m_mpm->initialise(m_shape, m_pos, m_size, m_vel, m_hardening, m_density, m_youngs, m_poisson, m_compression, m_stretch, m_blending, m_gridsize, m_timestep, m_force, m_resolutionX, m_resolutionY);
  update();
}

void NGLScene::step()
{
  if(m_playtimer >= 0)
  {
    killTimer(m_playtimer);
    m_playtimer = -1;
    initialise();
  }  
  m_mpm->simulate();
  update();
}

void NGLScene::start()
{
  if(m_playtimer >= 0)
  {
    killTimer(m_playtimer);
    m_playtimer = -1;
    initialise();    
  }
  if(m_timer < 0)
  {
    m_timer = startTimer(1);  
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

// Functions for save tab actions

void NGLScene::setFilename(QString s)
{
  m_filename = s.toStdString();
}

void NGLScene::setFrame(int i)
{
  m_frame = i;
}

void NGLScene::setFPS(int i)
{
  m_fps = i;
}

void NGLScene::save()
{
  if(m_filename.length() <= 0)
  {
    QMessageBox msgBox;
    msgBox.setText("Please set the file name.");
    msgBox.exec();
  }
  else
  {
    std::ifstream ifile;
    ifile.open(fmt::format("../render/{}.txt", m_filename));
    if(ifile) {
      QMessageBox msgBox;
      msgBox.setText("The file name is already used.");
      msgBox.exec();
    }
    else 
    {
      std::ofstream file;
      file.open(fmt::format("../render/{}.txt", m_filename));
      std::stringstream ss;
      ss<<"frame: "<<m_frame<<'\n';
      ss<<"fps: "<<m_fps<<'\n';
      ss<<"gridsize: "<<m_gridsize<<'\n';
      ss<<"resolutionX: "<<m_resolutionX<<'\n';
      ss<<"resolutionY: "<<m_resolutionY<<'\n';
      ss<<"shape: "<<m_shape<<'\n';
      ss<<"pos: "<<m_pos.m_x<<' '<<m_pos.m_y<<'\n';
      ss<<"size: "<<m_size.m_x<<' '<<m_size.m_y<<'\n';
      ss<<"Velocity: "<<m_vel.m_x<<' '<<m_vel.m_y<<'\n';
      ss<<"hardening: "<<m_hardening<<'\n';
      ss<<"density: "<<m_density<<'\n';
      ss<<"youngs: "<<m_youngs<<'\n';
      ss<<"poisson: "<<m_poisson<<'\n';
      ss<<"compression: "<<m_compression<<'\n';
      ss<<"stretch: "<<m_stretch<<'\n';
      ss<<"blending: "<<m_blending<<'\n';
      ss<<"timestep: "<<m_timestep<<'\n';
      ss<<"force: "<<m_force.m_x<<' '<<m_force.m_y<<'\n';
      file<<ss.rdbuf();
      file.close();

      m_mpm = std::make_unique<MPM>();
      m_mpm->initialise(m_shape, m_pos, m_size, m_vel, m_hardening, m_density, m_youngs, m_poisson, m_compression, m_stretch, m_blending, m_gridsize, m_timestep, m_force, m_resolutionX, m_resolutionY);
      m_mpm->saveFrame(0, m_filename);
      for(int i=1; i<=m_frame; ++i)
      {
        for(int j=0; j<static_cast<int>(1.0f/m_fps/m_timestep); ++j)
        {
          std::cout<<"Frame: "<<i<<" Sim: "<<j<<'\n';
          m_mpm->simulate();
        }
        m_mpm->saveFrame(i, m_filename);
      }
      QMessageBox msgBox;
      msgBox.setText("Completed");
      msgBox.exec();    
    }
  }
}

void NGLScene::setTextfile(QString s)
{
  m_textfile = s.toStdString();
}

void NGLScene::lookup()
{ 
  std::string filename = QFileDialog::getOpenFileName(this, tr("Select file"), "../render/", tr("Text Files (*.txt)")).toStdString();
  std::cout<<filename<<'\n';
}

void NGLScene::load()
{
  if(m_textfile.length() <= 0)
  {
    QMessageBox msgBox;
    msgBox.setText("Please set the file name.");
    msgBox.exec();
  }
  else
  {
    int numParticles = 0;
    std::string line, token;

    std::ifstream file(fmt::format("../render/{}.txt", m_textfile));
    if(file.is_open())
    {
      getline(file, line);
      m_totalFrame = std::stoi(line.substr(7, line.length()));
      getline(file, line);
      m_playfps = std::stof(line.substr(5, line.length()));
      getline(file, line);
      float gridsize = std::stof(line.substr(10, line.length()));
      getline(file, line);
      int resolutionX = std::stoi(line.substr(13, line.length()));
      getline(file, line);
      int resolutionY = std::stoi(line.substr(13, line.length()));
      file.close();

      file.open(fmt::format("../render/{}_0000.geo", m_textfile));

      if(file.is_open())
      {
        stop();
        m_currentFrame = 0;
        m_particleVel = 0;
        m_gridVel = 0;
        getline(file, line);
        getline(file, line);
        std::istringstream stream(line);
        getline(stream, token, ' ');
        getline(stream, token, ' ');
        int numParticles = std::stoi(token);

        m_mpm = std::make_unique<MPM>();
        m_mpm->prep(numParticles, gridsize, resolutionX, resolutionY);
        m_loaded = true;
        playstep();
      }
      else
      {
        QMessageBox msgBox;
        msgBox.setText(QString::fromStdString(fmt::format("Unable to open the file: ../render/{}_0000.geo", m_textfile)));
        msgBox.exec();
      }    
    }
    else
    {
      QMessageBox msgBox;
      msgBox.setText(QString::fromStdString(fmt::format("Unable to open the file: ../render/{}.txt", m_textfile)));
      msgBox.exec();
    }
  }
}

void NGLScene::playstep()
{
  if(m_loaded && m_currentFrame <= m_totalFrame)
  {
    std::cout<<"Frame: "<<m_currentFrame<<'\n';
    m_mpm->play(m_currentFrame, m_textfile);
    ++m_currentFrame;
    if(m_currentFrame == m_totalFrame && m_playtimer >= 0)
    {
      killTimer(m_playtimer);
      m_playtimer = -1;        
    }
    update();
  }
}

void NGLScene::play()
{
  if(m_loaded)
  {
    m_playtimer = startTimer(static_cast<int>(1.0f/m_playfps*1000.0f));
  }
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
  if(m_timer >= 0)
  {
    m_mpm->simulate();
  }
  if(m_playtimer >= 0)
  {
    if(m_currentFrame <= m_totalFrame)
    {
      std::cout<<"Frame: "<<m_currentFrame<<'\n';
      m_mpm->play(m_currentFrame, m_textfile);
      ++m_currentFrame;
      if(m_currentFrame == m_totalFrame)
      {
        killTimer(m_playtimer);
        m_playtimer = -1;        
      }
    }
  }
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

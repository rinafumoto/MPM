#ifndef NGLSCENE_H_
#define NGLSCENE_H_
#include <ngl/Vec3.h>
#include <memory>
#include "MPM.h"
#include "WindowParams.h"
// this must be included after NGL includes else we get a clash with gl libs
#include <QOpenGLWidget>
//----------------------------------------------------------------------------------------------------------------------
/// @file NGLScene.h
/// @brief this class inherits from the Qt OpenGLWindow and allows us to use NGL to draw OpenGL
/// @author Jonathan Macey
/// @version 1.0
/// @date 10/9/13
/// Revision History :
/// This is an initial version used for the new NGL6 / Qt 5 demos
/// @class NGLScene
/// @brief our main glwindow widget for NGL applications all drawing elements are
/// put in this file
//----------------------------------------------------------------------------------------------------------------------

class NGLScene : public QOpenGLWidget
{
    Q_OBJECT
  public:
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief ctor for our NGL drawing class
    /// @param [in] parent the parent window to the class
    //----------------------------------------------------------------------------------------------------------------------
    NGLScene(QWidget *_parent);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief dtor must close down ngl and release OpenGL resources
    //----------------------------------------------------------------------------------------------------------------------
    ~NGLScene() override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the initialize class is called once when the window is created and we have a valid GL context
    /// use this to setup any default GL stuff
    //----------------------------------------------------------------------------------------------------------------------
    void initializeGL() override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this is called everytime we want to draw the scene
    //----------------------------------------------------------------------------------------------------------------------
    void paintGL() override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this is called everytime we resize the window
    //----------------------------------------------------------------------------------------------------------------------
    void resizeGL(int _w, int _h) override;

public slots :

    //----------------------------------------------------------------------------------------------------------------------
	/// @brief a slot to initialise the simulation with the setting values from GUI
    //----------------------------------------------------------------------------------------------------------------------
    void initialise();

    //----------------------------------------------------------------------------------------------------------------------
	/// @brief a slot to simulate one step
    //----------------------------------------------------------------------------------------------------------------------
    void step();

    //----------------------------------------------------------------------------------------------------------------------
	/// @brief a slot to start the simulation
    //----------------------------------------------------------------------------------------------------------------------
    void start();

    //----------------------------------------------------------------------------------------------------------------------
	/// @brief a slot to stop the simulation
    //----------------------------------------------------------------------------------------------------------------------
    void stop();

    //----------------------------------------------------------------------------------------------------------------------
	/// @brief a slot to set the values from GUI
	/// @param i the integer value to set
	/// @param d the double value to set
	/// @param b the boolean value to set
    //----------------------------------------------------------------------------------------------------------------------
    void setShape(int i);
    void setPositionX(double d);
    void setPositionY(double d);
    void setSizeX(double d);
    void setSizeY(double d);
    void setVelocityX(double d);
    void setVelocityY(double d);
    void setHardening(double d);
    void setDensity(double d);
    void setYoungs(double d);
    void setPoisson(double d);
    void setCompression(double d);
    void setStretch(double d);
    void setBlending(double d);
    void setGridSize(double d);
    void setTimeStep(double d);
    void setForceX(double d);
    void setForceY(double d);
    void setResolutionX(int i);
    void setResolutionY(int i);
    void setParticleVel(bool b);
    void setGridVel(bool b);

    void setFilename(QString s);
    void setFrame(int i);
    void setFPS(int i);
    void save();
    void setTextfile(QString s);
    void lookup();
    void load();
    void playstep();
    void play();
    
private:

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief Qt Event called when a key is pressed
    /// @param [in] _event the Qt event to query for size etc
    //----------------------------------------------------------------------------------------------------------------------
    void keyPressEvent(QKeyEvent *_event) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called every time a mouse is moved
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void mouseMoveEvent (QMouseEvent * _event ) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the mouse button is pressed
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void mousePressEvent ( QMouseEvent *_event) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the mouse button is released
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void mouseReleaseEvent ( QMouseEvent *_event ) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the timer event occurs
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void timerEvent ( QTimerEvent *_event) override;

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the mouse wheel is moved
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void wheelEvent( QWheelEvent *_event) override;


    /// @brief windows parameters for mouse control etc.
    WinParams m_win;
    /// position for our model
    ngl::Vec3 m_modelPos;


    std::unique_ptr<MPM> m_mpm;

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief A variable to store timer id.
    //----------------------------------------------------------------------------------------------------------------------
    int m_timer = -1;
    int m_playtimer = -1;
    int m_totalFrame = -1;
    int m_currentFrame = 0;
    bool m_loaded = 0;
    int m_playfps = 0;

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief variables to store the simulation settings from GUI
    //----------------------------------------------------------------------------------------------------------------------
    int m_shape = 0;
    ngl::Vec3 m_pos = {10.0f, 10.0f, 0.0f};
    ngl::Vec3 m_size = {10.0f, 10.0f, 0.0f};
    ngl::Vec3 m_vel = 0.0f;
    double m_hardening = 10.0f;
    double m_density = 400.0f;
    double m_youngs = 140000.0f;
    double m_poisson = 0.2f;
    double m_compression = 0.025f;
    double m_stretch = 0.0075f;
    double m_blending = 0.95f;
    double m_gridsize = 0.001f;
    double m_timestep = 0.000005f;
    ngl::Vec3 m_force = 0.0f;
    int m_resolutionX = 20;
    int m_resolutionY = 20;
    bool m_particleVel = 0;
    bool m_gridVel = 0;

    std::string m_filename;
    int m_frame = 50;
    std::string m_textfile;
    int m_fps = 25;
    

};



#endif

#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent)
  , ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  m_gl = new NGLScene(this);
  ui->m_mainWindowGridLayout->addWidget(m_gl);
  ui->m_lookup->setIcon(QIcon("../data/lookup.png"));

  // connect the GUI dropdown value to a slot
  connect(ui->m_shape,SIGNAL(currentIndexChanged(int)),m_gl,SLOT(setShape(int)));
  // connect the other GUI value to slots
  connect(ui->m_posX,SIGNAL(valueChanged(double)),m_gl,SLOT(setPositionX(double)));
  connect(ui->m_posY,SIGNAL(valueChanged(double)),m_gl,SLOT(setPositionY(double)));
  connect(ui->m_sizeX,SIGNAL(valueChanged(double)),m_gl,SLOT(setSizeX(double)));
  connect(ui->m_sizeY,SIGNAL(valueChanged(double)),m_gl,SLOT(setSizeY(double)));
  connect(ui->m_velX,SIGNAL(valueChanged(double)),m_gl,SLOT(setVelocityX(double)));
  connect(ui->m_velY,SIGNAL(valueChanged(double)),m_gl,SLOT(setVelocityY(double)));
  connect(ui->m_hardening,SIGNAL(valueChanged(double)),m_gl,SLOT(setHardening(double)));
  connect(ui->m_density,SIGNAL(valueChanged(double)),m_gl,SLOT(setDensity(double)));
  connect(ui->m_youngs,SIGNAL(valueChanged(double)),m_gl,SLOT(setYoungs(double)));
  connect(ui->m_poisson,SIGNAL(valueChanged(double)),m_gl,SLOT(setPoisson(double)));
  connect(ui->m_compression,SIGNAL(valueChanged(double)),m_gl,SLOT(setCompression(double)));
  connect(ui->m_stretch,SIGNAL(valueChanged(double)),m_gl,SLOT(setStretch(double)));
  connect(ui->m_blending,SIGNAL(valueChanged(double)),m_gl,SLOT(setBlending(double)));
  connect(ui->m_resX,SIGNAL(valueChanged(int)),m_gl,SLOT(setResolutionX(int)));
  connect(ui->m_resY,SIGNAL(valueChanged(int)),m_gl,SLOT(setResolutionY(int)));
  connect(ui->m_forceX,SIGNAL(valueChanged(double)),m_gl,SLOT(setForceX(double)));
  connect(ui->m_forceY,SIGNAL(valueChanged(double)),m_gl,SLOT(setForceY(double)));
  connect(ui->m_gridsize,SIGNAL(valueChanged(double)),m_gl,SLOT(setGridSize(double)));
  connect(ui->m_timestep,SIGNAL(valueChanged(double)),m_gl,SLOT(setTimeStep(double)));
  // connect the GUI checkboxs value to slots
  connect(ui->m_particleVel,&QCheckBox::stateChanged,m_gl,&NGLScene::setParticleVel);
  connect(ui->m_gridVel,&QCheckBox::stateChanged,m_gl,&NGLScene::setGridVel);
  // connect the GUI buttons to slots
  connect(ui->m_initialise,&QPushButton::clicked,m_gl,&NGLScene::initialise);
  connect(ui->m_start,&QPushButton::clicked,m_gl,&NGLScene::start);
  connect(ui->m_stop,&QPushButton::clicked,m_gl,&NGLScene::stop);
  connect(ui->m_step,&QPushButton::clicked,m_gl,&NGLScene::step);
  // connect the GUI save tab contents to slots
  connect(ui->m_filename,SIGNAL(textChanged(QString)),m_gl,SLOT(setFilename(QString)));
  connect(ui->m_frame,SIGNAL(valueChanged(int)),m_gl,SLOT(setFrame(int)));
  connect(ui->m_fps,SIGNAL(valueChanged(int)),m_gl,SLOT(setFPS(int)));
  connect(ui->m_save,&QPushButton::clicked,m_gl,&NGLScene::save);
  connect(ui->m_load,&QPushButton::clicked,m_gl,&NGLScene::load);
  connect(ui->m_playstep,&QPushButton::clicked,m_gl,&NGLScene::playstep);
  connect(ui->m_play,&QPushButton::clicked,m_gl,&NGLScene::play);
  connect(ui->m_pause,&QPushButton::clicked,m_gl,&NGLScene::pause);
  connect(ui->m_textfile,SIGNAL(textChanged(QString)),m_gl,SLOT(setTextfile(QString)));
  connect(ui->m_lookup,&QToolButton::clicked,m_gl,&NGLScene::lookup);

  connect(m_gl,SIGNAL(fileSelected(QString)),ui->m_textfile,SLOT(setText(QString)));

}

MainWindow::~MainWindow()
{
  delete ui;
  delete m_gl;
}
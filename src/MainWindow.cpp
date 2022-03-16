#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent)
  , ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  m_gl = new NGLScene(this);
  ui->m_mainWindowGridLayout->addWidget(m_gl);

  // connect the GUI buttons to slots
  connect(ui->m_initialise,&QPushButton::clicked,m_gl,&NGLScene::initialise);
  connect(ui->m_start,&QPushButton::clicked,m_gl,&NGLScene::start);
  connect(ui->m_stop,&QPushButton::clicked,m_gl,&NGLScene::stop);
  connect(ui->m_step,&QPushButton::clicked,m_gl,&NGLScene::step);
}

MainWindow::~MainWindow()
{
  delete ui;
  delete m_gl;
}


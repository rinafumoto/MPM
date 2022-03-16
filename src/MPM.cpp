#include "MPM.h"
#include <iostream>
#include <iomanip>
#include <ngl/ShaderLib.h>
#include <ngl/VAOFactory.h>
#include <ngl/Util.h>

std::mt19937 MPM::m_generator;
auto randomPositivezDist=std::uniform_real_distribution<float>(0.0f,1.0f);

////////// Initialisation //////////

void MPM::initialise()
{
    m_gridsize = 1;
    m_resolutionX = 10+2;
    m_resolutionY = 10+2;
    m_timestep = 0.1;
    m_force = ngl::Vec3(0,-9.8,0);


    // Initialise vectors.
    m_boundary.resize(m_resolutionX*m_resolutionY, 0);
    // m_velocityX.resize((m_resolutionX+1)*m_resolutionY, 0.0f);
    // m_velocityY.resize(m_resolutionX*(m_resolutionY+1), 0.0f);

    // // Initialise VAO.
    m_vao = ngl::vaoFactoryCast<ngl::MultiBufferVAO>(ngl::VAOFactory::createVAO(ngl::multiBufferVAO,GL_POINTS));
    m_vao->bind();
        m_vao->setData(ngl::MultiBufferVAO::VertexData(0,0));
    m_vao->unbind();

    ngl::Vec3 initPos = ngl::Vec3(5,5,0);
    int initHalfWidth = 4/2;
    int initHalfHeight = 4/2;

    for(int i = initPos.m_x+1-initHalfWidth; i<=initPos.m_x+1+initHalfWidth; ++i)
    {
        for(int j = initPos.m_y+1-initHalfHeight; j<=initPos.m_y+1+initHalfHeight; ++j)
        {
            m_position.push_back({static_cast<float>(i),static_cast<float>(j),0.0f});
            m_boundary[i*m_resolutionX+i] = 1;
            ++m_numParticles;
        }
    }
    
    // Set solid cells on the edge.
    for(int i=0; i<m_resolutionY; ++i)
    {
        for(int j=0; j<1/m_gridsize; ++j)
        {
            m_boundary[i*m_resolutionX+j] = -1;
            m_boundary[i*m_resolutionX+m_resolutionX-1-j] = -1;
        }
    }
    for(int i=0; i<m_resolutionX; ++i)
    {
        for(int j=0; j<1/m_gridsize; ++j)
        {
            m_boundary[j*m_resolutionX+i] = -1;
            m_boundary[m_resolutionX*m_resolutionY-1-i-j*m_resolutionX] = -1;
        }
    }

    // Add the positions of the solid cells to the vector for visualisation.
    m_solid.reserve(m_resolutionX*2 + m_resolutionX*2 - 4/(m_gridsize*m_gridsize));
    for(int j=0; j<m_resolutionY; ++j)
    {
        for(int i=0; i<m_resolutionX; ++i)
        {
            if(m_boundary[j*m_resolutionX+i] == -1)
            {
                m_solid.push_back({(static_cast<float>(i)+0.5f)*m_gridsize,(static_cast<float>(j)+0.5f)*m_gridsize, 0.0f});
            }
        }
    }
}

void MPM::simulate()
{

}

////////// Render //////////

void MPM::render()
{
    std::cout<<"RENDERING\n";
    const auto ColourShader = "ColourShader";
    const auto SolidShader = "SolidShader";

    float width = static_cast<float>(m_resolutionX*m_gridsize);
    float height = static_cast<float>(m_resolutionY*m_gridsize);

    // Use orthographic projection for 2D simulation.
    auto view = ngl::lookAt({width/2.0f,height/2.0f,10}, {width/2.0f,height/2.0f,0}, {0,1,0});
    auto project = ngl::ortho(-width/2.0f, width/2.0f, -height/2.0f, height/2.0f, 0.1f, 50.0f);

    // Visualise solid cells.
    ngl::ShaderLib::use(SolidShader);
    ngl::ShaderLib::setUniform("MVP",project*view);
    ngl::ShaderLib::setUniform("size",ngl::Vec2(m_resolutionX,m_resolutionY));

    m_vao->bind();
        m_vao->setData(0,ngl::MultiBufferVAO::VertexData(m_solid.size()*sizeof(ngl::Vec3),m_solid[0].m_x));
        m_vao->setVertexAttributePointer(0,3,GL_FLOAT,0,0);
        m_vao->setNumIndices(m_solid.size());
        m_vao->draw();
    m_vao->unbind();   
    
    ngl::ShaderLib::use(ColourShader);
    ngl::ShaderLib::setUniform("MVP",project*view);
    glPointSize(10);

    m_vao->bind();
        m_vao->setData(0,ngl::MultiBufferVAO::VertexData(m_numParticles*sizeof(ngl::Vec3),m_position[0].m_x));
        m_vao->setVertexAttributePointer(0,3,GL_FLOAT,0,0);          
        m_vao->setNumIndices(m_numParticles);
        m_vao->draw();
    m_vao->unbind();
    
}
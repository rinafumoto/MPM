#include "MPM.h"
#include <iostream>
#include <iomanip>
#include <math.h>
#include <ngl/ShaderLib.h>
#include <ngl/VAOFactory.h>
#include <ngl/Util.h>

std::mt19937 MPM::m_generator;
auto randomPositivezDist=std::uniform_real_distribution<float>(0.0f,1.0f);

////////// Initialisation //////////

void MPM::initialise()
{
    // std::cout << std::setprecision(2) << std::fixed;

    m_gridsize = 1.0f;
    m_resolutionX = 10+2;
    m_resolutionY = 10+2;
    m_timestep = 0.1f;
    m_force = ngl::Vec3(0,-9.8f,0);


    // Initialise vectors.
    m_boundary.resize(m_resolutionX*m_resolutionY, 0);
    m_gridMass.resize((m_resolutionX+1)*(m_resolutionY+1), 0.0f);

    // // Initialise VAO.
    m_vao = ngl::vaoFactoryCast<ngl::MultiBufferVAO>(ngl::VAOFactory::createVAO(ngl::multiBufferVAO,GL_POINTS));
    m_vao->bind();
        m_vao->setData(ngl::MultiBufferVAO::VertexData(0,0));
    m_vao->unbind();

    int bottom = 3;
    int left = 3;
    int top = 7;
    int right = 7;

    for(int i = left+1; i<=right+1; ++i)
    {
        for(int j = bottom+1; j<=top+1; ++j)
        {
            m_position.push_back({static_cast<float>(i),static_cast<float>(j),0.0f});
            m_velocity.push_back(0.0f);
            m_mass.push_back(5.0f);
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

float MPM::interpolate(float i, float j, ngl::Vec3 x)
{
    return bSpline((x.m_x-i*m_gridsize)/m_gridsize)*bSpline((x.m_y-j*m_gridsize)/m_gridsize);
}

float MPM::bSpline(float x)
{
    float absoluteX = std::abs(x);
    if(absoluteX >= 0.0f && absoluteX < 1.0f)
    {
        return pow(absoluteX,3.0f)/2.0f-pow(x,2.0f)+2.0f/3.0f;
    }
    if(absoluteX >= 1.0f && absoluteX < 2.0f)
    {
        return -pow(absoluteX,3.0f)/6.0f+pow(x,2.0f)-2.0f*absoluteX+4.0f/3.0f;
    }
    return 0.0f;
}

void MPM::simulate()
{
    particleToGrid();
    // computeDensity();
    // computeVolume();
    // updateGridVelocity();
    // collision();
    // updateDeformationGradients();
    // gridToParticle();
}

void MPM::particleToGrid()
{
    std::fill(m_gridMass.begin(), m_gridMass.end(), 0.0f);

    for(int k=0; k<m_numParticles; ++k)
    {
        int x_index = static_cast<int>(m_position[k].m_x)-1;
        int y_index = static_cast<int>(m_position[k].m_y)-1;
        for (int i=x_index; i<x_index+4; ++i)
        {
            for (int j=y_index; j<y_index+4; ++j)
            {
                if(i>=0 && i<=m_resolutionX && j>=0 && j<=m_resolutionY)
                    m_gridMass[j*(m_resolutionX+1)+i] += m_mass[k]*interpolate(i,j,m_position[k]);
            }
        }
    }
}

void MPM::computeDensity()
{

}

void MPM::computeVolume()
{

}

void MPM::updateGridVelocity()
{

}

void MPM::collision()
{

}

void MPM::updateDeformationGradients()
{

}

void MPM::gridToParticle()
{

}


////////// Render //////////

void MPM::render()
{
    // std::cout<<"RENDERING\n";
    std::cout<<"\n*******************************\nMass Field\n";
    for(int j=m_resolutionY; j>=0; --j)
    {
        for(int i=0; i<m_resolutionX+1; ++i)
        {
            std::cout<<m_gridMass[j*(m_resolutionX+1)+i]<<' ';
        }
        std::cout<<'\n';
    }

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
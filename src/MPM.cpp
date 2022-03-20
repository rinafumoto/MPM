#include "MPM.h"
#include <iostream>
#include <iomanip>
#include <math.h>
#include <ngl/ShaderLib.h>
#include <ngl/VAOFactory.h>
#include <ngl/Util.h>
#include <Eigen/LU>

std::mt19937 MPM::m_generator;
auto randomPositivezDist=std::uniform_real_distribution<float>(0.0f,1.0f);

////////// Initialisation //////////

void MPM::initialise()
{
    //Disney
    float youngs = 1.4*pow(10.0f, 5.0f);
    float poisson = 0.2f;
    m_lambda = (youngs*poisson)/((1.0f+poisson)*(1.0f-2.0f*poisson));
    m_mu = youngs/(2.0f*(1.0f+poisson));
    float initialDensity = 400.0f;

    m_hardening = 10.0f;
    m_compression = 0.025f;
    m_stretch = 0.0075f;
    m_blending = 0.95f;

    m_first = true;
    m_gridsize = 0.001f;
    m_resolutionX = 20+2;
    m_resolutionY = 20+2;
    m_timestep = 0.000005f;
    m_force = ngl::Vec3(0.0f);
    m_gravity = -9.81f;

    // Initialise vectors.
    m_gridMass.resize((m_resolutionX+1)*(m_resolutionY+1), 0.0f);
    m_gridVelocity.resize((m_resolutionX+1)*(m_resolutionY+1), 0.0f);

    // // Initialise VAO.
    m_vao = ngl::vaoFactoryCast<ngl::MultiBufferVAO>(ngl::VAOFactory::createVAO(ngl::multiBufferVAO,GL_POINTS));
    m_vao->bind();
        m_vao->setData(ngl::MultiBufferVAO::VertexData(0,0));
        m_vao->setData(ngl::MultiBufferVAO::VertexData(0,0));
    m_vao->unbind();

    int bottom = 5;
    int left = 5;
    int top = 15;
    int right = 15;

    // Initialise particles at cell corners
    float initialMass = initialDensity*m_gridsize*m_gridsize;
    m_numParticles = (top-bottom+1)*(right-left+1);
    m_position.reserve(m_numParticles);
    m_velocity.reserve(m_numParticles);
    m_mass.reserve(m_numParticles);

    for(int i = left; i<=right; ++i)
    {
        for(int j = bottom; j<=top; ++j)
        {
            m_position.push_back({(i+0.5f)*m_gridsize,(j+0.5f)*m_gridsize,0.0f});
            // m_position.push_back({static_cast<float>(i)*m_gridsize,static_cast<float>(j)*m_gridsize,0.0f});
            m_velocity.push_back(0.0f);
            // m_velocity.push_back({20.0f*initialMass/m_timestep,0.0f,0.0f});
            m_mass.push_back(initialMass);
        }
    }

    // // Initialise particles at random positions
    // size_t numParticlesInCell = 4;
    // m_numParticles = (top-bottom)*(right-left)*numParticlesInCell;
    // m_position.reserve(m_numParticles);
    // m_velocity.reserve(m_numParticles);
    // m_mass.reserve(m_numParticles);
    // float initialMass = initialDensity*m_gridsize*m_gridsize/numParticlesInCell;
    // for(int j=bottom+1; j<top+1; ++j)
    // {
    //     for(int i=left+1; i<right+1; ++i)
    //     {
    //         for(int k=0; k<numParticlesInCell; ++k)
    //         {
    //             float x = (randomPositivezDist(m_generator)+i)*m_gridsize;
    //             float y = (randomPositivezDist(m_generator)+j)*m_gridsize;
    //             m_position.push_back({x,y,0.0f});
    //             m_velocity.push_back(0.0f);
    //             m_mass.push_back(initialMass);
    //         }
    //     }
    // }

    m_elastic.resize(m_numParticles, Eigen::Matrix3f::Identity());
    m_plastic.resize(m_numParticles, Eigen::Matrix3f::Identity());

    // Set solid cells on the edge.
    m_normal.resize((m_resolutionX+1)*(m_resolutionY+1), 0.0f);
    m_solid.reserve(m_resolutionX*2 + m_resolutionX*2 - 4);
    for(int i=0; i<m_resolutionY+1; ++i)
    {
        m_normal[i*(m_resolutionX+1)].m_x = 1.0f;
        m_normal[i*(m_resolutionX+1)+1].m_x = 1.0f;
        m_normal[i*(m_resolutionX+1)+m_resolutionX].m_x = -1.0f;
        m_normal[i*(m_resolutionX+1)+m_resolutionX-1].m_x = -1.0f;
        m_solid.push_back({0.5f*m_gridsize,(static_cast<float>(i)+0.5f)*m_gridsize, 0.0f});
        m_solid.push_back({(m_resolutionX-0.5f)*m_gridsize,(static_cast<float>(i)+0.5f)*m_gridsize, 0.0f});
    }
    for(int i=0; i<m_resolutionX+1; ++i)
    {
        m_normal[i].m_y = 1.0f;
        m_normal[m_resolutionX+1+i].m_y = 1.0f;
        m_normal[(m_resolutionX+1)*(m_resolutionY+1)-1-i].m_y = -1.0f;
        m_normal[(m_resolutionX+1)*m_resolutionY-1-i].m_y = -1.0f;
        m_normal[i].normalize();
        m_normal[m_resolutionX+1+i].normalize();
        m_normal[(m_resolutionX+1)*(m_resolutionY+1)-1-i].normalize();
        m_normal[(m_resolutionX+1)*m_resolutionY-1-i].normalize();        
        m_solid.push_back({(static_cast<float>(i)+0.5f)*m_gridsize,0.5f*m_gridsize, 0.0f});
        m_solid.push_back({(static_cast<float>(i)+0.5f)*m_gridsize,(m_resolutionY-0.5f)*m_gridsize, 0.0f});        
    }

    m_indices.resize((m_resolutionX+1)*(m_resolutionY+1));
    for(int i=0; i<m_resolutionX+1; ++i)
    {
        for(int j=0; j<m_resolutionY+1; ++j)
        {
            m_indices[j*(m_resolutionX+1)+i] = ngl::Vec3(static_cast<float>(i)*m_gridsize,static_cast<float>(j)*m_gridsize, 0.0f);
        }
    }
}

float MPM::interpolate(float _i, float _j, ngl::Vec3 _x)
{
    return bSpline((_x.m_x-_i*m_gridsize)/m_gridsize)*bSpline((_x.m_y-_j*m_gridsize)/m_gridsize);
}

float MPM::bSpline(float _x)
{
    float absX = std::abs(_x);
    if(absX >= 0.0f && absX < 1.0f)
    {
        return pow(absX,3.0f)/2.0f-pow(_x,2.0f)+2.0f/3.0f;
    }
    if(absX >= 1.0f && absX < 2.0f)
    {
        return -pow(absX,3.0f)/6.0f+pow(_x,2.0f)-2.0f*absX+4.0f/3.0f;
    }
    return 0.0f;
}

ngl::Vec3 MPM::dInterpolate(float _i, float _j, ngl::Vec3 _x)
{
    return {dBSpline((_x.m_x-_i*m_gridsize)/m_gridsize)*bSpline((_x.m_y-_j*m_gridsize)/m_gridsize),
            bSpline((_x.m_x-_i*m_gridsize)/m_gridsize)*dBSpline((_x.m_y-_j*m_gridsize)/m_gridsize),
            0.0f};
}

float MPM::dBSpline(float _x)
{
    float absX = std::abs(_x);
    float result;
    if(absX >= 0.0f && absX < 1.0f)
    {
        result = 3.0f/2.0f*pow(absX,2.0f)-2.0f*absX;
    }
    else if(absX >= 1.0f && absX < 2.0f)
    {
        result = -pow(absX,2.0f)/2.0f+2.0f*absX-2.0f;
    }
    else
    {
        result = 0.0f;
    }
    if(_x < 0.0f)
    {
        result *= -1.0f;
    }
    return result;
}

Eigen::Vector3f MPM::eigenVec3(ngl::Vec3 _v)
{
    return Eigen::Vector3f(_v.m_x, _v.m_y, _v.m_z);
}

void MPM::simulate()
{
    particleToGrid();
    if(m_first)
    {
        computeDensityAndVolume();
        m_first = false;
    }
    updateGridVelocity();
    gridCollision();
    updateDeformationGradients();
    gridToParticle();
}

void MPM::particleToGrid()
{
    std::fill(m_gridMass.begin(), m_gridMass.end(), 0.0f);
    std::fill(m_gridVelocity_old.begin(), m_gridVelocity_old.end(), 0.0f);
    std::fill(m_gridVelocity.begin(), m_gridVelocity.end(), 0.0f);
    
    std::vector<ngl::Vec3> velSum;
    velSum.resize((m_resolutionX+1)*(m_resolutionY+1), 0.0f);

    for(int k=0; k<m_numParticles; ++k)
    {
        int x_index = static_cast<int>(m_position[k].m_x/m_gridsize)-1;
        int y_index = static_cast<int>(m_position[k].m_y/m_gridsize)-1;
        for (int i=x_index; i<x_index+4; ++i)
        {
            for (int j=y_index; j<y_index+4; ++j)
            {
                if(i>=0 && i<=m_resolutionX && j>=0 && j<=m_resolutionY)
                {
                    float weightedMass = m_mass[k]*interpolate(i,j,m_position[k]);
                    m_gridMass[j*(m_resolutionX+1)+i] += weightedMass;
                    velSum[j*(m_resolutionX+1)+i] += weightedMass*m_velocity[k];
                }
            }
        }
    }

    for (int i=0; i<m_gridVelocity.size(); ++i)
    {
        if(velSum[i].m_x != 0)
             m_gridVelocity[i].m_x = velSum[i].m_x/m_gridMass[i];
        if(velSum[i].m_y != 0)
             m_gridVelocity[i].m_y = velSum[i].m_y/m_gridMass[i];
    }

    m_gridVelocity_old = m_gridVelocity;
}

void MPM::computeDensityAndVolume()
{
    m_density.resize(m_numParticles, 0.0f);
    m_volume.resize(m_numParticles, 0.0f);
    for(int k=0; k<m_numParticles; ++k)
    {
        int x_index = static_cast<int>(m_position[k].m_x/m_gridsize)-1;
        int y_index = static_cast<int>(m_position[k].m_y/m_gridsize)-1;
        for (int i=x_index; i<x_index+4; ++i)
        {
            for (int j=y_index; j<y_index+4; ++j)
            {
                if(i>=0 && i<=m_resolutionX && j>=0 && j<=m_resolutionY)
                {
                    m_density[k] += m_gridMass[j*(m_resolutionX+1)+i]*interpolate(i,j,m_position[k])/pow(m_gridsize,2.0f);
                }
            }
        }
        m_volume[k] = m_mass[k]/m_density[k];
    }
}

void MPM::updateGridVelocity()
{
    std::vector<ngl::Vec3> forces;
    forces.resize((m_resolutionX+1)*(m_resolutionY+1), 0.0f);

    for(int k=0; k<m_numParticles; ++k)
    {
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(m_elastic[k], Eigen::ComputeFullU | Eigen::ComputeFullV);
        float mu = m_mu*exp(m_hardening*(1.0f-m_plastic[k].determinant()));
        float lambda = m_lambda*exp(m_hardening*(1.0f-m_plastic[k].determinant()));
        Eigen::Matrix3f R = svd.matrixU()*svd.matrixV().transpose();
        Eigen::Matrix3f dLeft = 2.0f*mu*(m_elastic[k]-R);
        Eigen::Matrix3f dRight = lambda*(m_elastic[k].determinant()-1.0f)*m_elastic[k].determinant()*m_elastic[k].transpose().inverse();
        Eigen::Matrix3f dGD = dLeft+dRight;

        int x_index = static_cast<int>(m_position[k].m_x/m_gridsize)-1;
        int y_index = static_cast<int>(m_position[k].m_y/m_gridsize)-1;
        for (int i=x_index; i<x_index+4; ++i)
        {
            for (int j=y_index; j<y_index+4; ++j)
            {
                if(i>=0 && i<=m_resolutionX && j>=0 && j<=m_resolutionY)
                {
                    Eigen::Vector3f f = -m_volume[k]*dGD*m_elastic[k].transpose()*eigenVec3(dInterpolate(i,j,m_position[k]));
                    forces[j*(m_resolutionX+1)+i] += {f[0], f[1], f[2]};
                }
            }
        }
    }

    // Add external forces and update velocities
    for(int i=0; i<forces.size(); ++i)
    {
        forces[i] += m_force + ngl::Vec3(0.0f, m_gravity*m_gridMass[i], 0.0f);
        if(m_gridMass[i] != 0.0f)
            m_gridVelocity[i] += m_timestep / m_gridMass[i] * forces[i];
    }
}

void MPM::gridCollision()
{
    float vn;
    for(int i=0; i<m_normal.size(); ++i)
    {
        vn = m_gridVelocity[i].dot(m_normal[i]);
        if(vn<0)
        {
            m_gridVelocity[i] -= m_normal[i]*vn;
        }
    }
}

void MPM::updateDeformationGradients()
{
    for(int k=0; k<m_numParticles; ++k)
    {
        int x_index = static_cast<int>(m_position[k].m_x/m_gridsize)-1;
        int y_index = static_cast<int>(m_position[k].m_y/m_gridsize)-1;
        Eigen::Matrix3f gVel = Eigen::Matrix3f::Zero();
        Eigen::Matrix3f weightSum = Eigen::Matrix3f::Zero();
        for (int i=x_index; i<x_index+4; ++i)
        {
            for (int j=y_index; j<y_index+4; ++j)
            {
                if(i>=0 && i<=m_resolutionX && j>=0 && j<=m_resolutionY)
                {
                    // Eigen::Matrix3f mat;
                    for(int l=0; l<3; ++l)
                    {
                        for(int m=0; m<3; ++m)
                        {
                            gVel(l,m) += m_gridVelocity[j*(m_resolutionX+1)+i][l]*dInterpolate(i,j,m_position[k])[m];
                            weightSum(l,m) += dInterpolate(i,j,m_position[k])[m];
                        }
                    }
                }
            }
        }

        m_elastic[k] = (Eigen::Matrix3f::Identity()+gVel)*m_elastic[k];

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(m_elastic[k], Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f sigma = Eigen::Matrix3f::Zero();
        for(int i=0; i<3; ++i)
        {
            sigma(i,i) = std::clamp(svd.singularValues()(i),1.0f-m_compression,1.0f+m_stretch);
        }

        m_plastic[k] = svd.matrixV()*sigma.inverse()*svd.matrixU().transpose()*m_elastic[k]*m_plastic[k];
        m_elastic[k] = svd.matrixU()*sigma*svd.matrixV().transpose();
    }
}

void MPM::gridToParticle()
{
    for(int k=0; k<m_numParticles; ++k)
    {
        // Update particle velocity
        int x_index = static_cast<int>(m_position[k].m_x/m_gridsize)-1;
        int y_index = static_cast<int>(m_position[k].m_y/m_gridsize)-1;
        ngl::Vec3 pic = 0.0f;
        ngl::Vec3 flip = 0.0f;
        for (int i=x_index; i<x_index+4; ++i)
        {
            for (int j=y_index; j<y_index+4; ++j)
            {
                if(i>=0 && i<=m_resolutionX && j>=0 && j<=m_resolutionY)
                {
                    pic += m_gridVelocity[j*(m_resolutionX+1)+i]*interpolate(i,j,m_position[k]);
                    flip += m_gridVelocity_old[j*(m_resolutionX+1)+i]*interpolate(i,j,m_position[k]);
                }
            }
        }
        m_velocity[k] = pic + m_blending*(m_velocity[k]-flip);

        // Particle-based collision handling.
        ngl::Vec3 normal = 0.0f;
        for (int i=x_index; i<x_index+4; ++i)
        {
            for (int j=y_index; j<y_index+4; ++j)
            {
                if(i>=0 && i<=m_resolutionX && j>=0 && j<=m_resolutionY)
                {
                    normal += m_normal[j*(m_resolutionX+1)+i]*interpolate(i,j,m_position[k]);
                }
            }
        }

        if(normal.length()>0.0f)
            normal.normalize();

        float vn = m_velocity[k].dot(normal);
        if(vn<0)
        {
            m_velocity[k] -= normal*vn;
        }

        // Update Particle position
        m_position[k] += m_timestep*m_velocity[k];
    }
}

////////// Render //////////

void MPM::render(size_t _w, size_t _h)
{
    const auto ColourShader = "ColourShader";
    const auto SolidShader = "SolidShader";
    const auto GridViz = "GridViz";

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
    glPointSize(5);

    m_vao->bind();
        m_vao->setData(0,ngl::MultiBufferVAO::VertexData(m_numParticles*sizeof(ngl::Vec3),m_position[0].m_x));
        m_vao->setVertexAttributePointer(0,3,GL_FLOAT,0,0);          
        m_vao->setNumIndices(m_numParticles);
        m_vao->draw();
    m_vao->unbind();

    ngl::ShaderLib::use(GridViz);
    ngl::ShaderLib::setUniform("thickness",2.0f);
    ngl::ShaderLib::setUniform("thickness2",1.0f);
    ngl::ShaderLib::setUniform("viewportSize",ngl::Vec2(_w,_h));
    ngl::ShaderLib::setUniform("MVP",project*view);

    // m_vao->bind();
    //     m_vao->setData(0,ngl::MultiBufferVAO::VertexData(m_numParticles*sizeof(ngl::Vec3),m_position[0].m_x));
    //     m_vao->setVertexAttributePointer(0,3,GL_FLOAT,0,0);
    //     m_vao->setData(1,ngl::MultiBufferVAO::VertexData(m_numParticles*sizeof(ngl::Vec3),m_velocity[0].m_x));
    //     m_vao->setVertexAttributePointer(1,3,GL_FLOAT,0,0);              
    //     m_vao->setNumIndices(m_numParticles);
    //     m_vao->draw();
    // m_vao->unbind();

    // m_vao->bind();
    //     m_vao->setData(0,ngl::MultiBufferVAO::VertexData(m_indices.size()*sizeof(ngl::Vec3),m_indices[0].m_x));
    //     m_vao->setVertexAttributePointer(0,3,GL_FLOAT,0,0);
    //     m_vao->setData(1,ngl::MultiBufferVAO::VertexData(m_gridVelocity.size()*sizeof(ngl::Vec3),m_gridVelocity[0].m_x));
    //     m_vao->setVertexAttributePointer(1,3,GL_FLOAT,0,0);              
    //     m_vao->setNumIndices(m_indices.size());
    //     m_vao->draw();
    // m_vao->unbind();  
}
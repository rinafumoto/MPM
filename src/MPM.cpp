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

    float youngs = 1.4*pow(10.0f, 5.0f);
    float poisson = 0.2f;
    m_lambda = (youngs*poisson)/((1.0f+poisson)*(1.0f-2.0f*poisson));
    m_mu = youngs/(2.0f*(1.0f+poisson));
    m_hardening = 10.0f;

    // std::cout << std::setprecision(2) << std::fixed;
    m_first = true;
    m_gridsize = 1.0f;
    m_resolutionX = 10+2;
    m_resolutionY = 10+2;
    m_timestep = 0.1f;
    m_force = ngl::Vec3(0,-9.8f,0);

    // Initialise vectors.
    m_boundary.resize(m_resolutionX*m_resolutionY, 0);
    m_gridMass.resize((m_resolutionX+1)*(m_resolutionY+1), 0.0f);
    m_gridVelocity.resize((m_resolutionX+1)*(m_resolutionY+1), 0.0f);

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
            m_velocity.push_back({3.0f,2.0f,0.0f});
            m_mass.push_back(5.0f);
            m_boundary[i*m_resolutionX+i] = 1;
            ++m_numParticles;
        }
    }
    
    m_elastic.resize(m_numParticles);
    m_plastic.resize(m_numParticles);

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

float MPM::interpolate(float _i, float _j, ngl::Vec3 _x)
{
    return bSpline(std::abs(_x.m_x-_i*m_gridsize)/m_gridsize)*bSpline(std::abs(_x.m_y-_j*m_gridsize)/m_gridsize);
}

float MPM::bSpline(float _x)
{
    if(_x >= 0.0f && _x < 1.0f)
    {
        return pow(_x,3.0f)/2.0f-pow(_x,2.0f)+2.0f/3.0f;
    }
    if(_x >= 1.0f && _x < 2.0f)
    {
        return -pow(_x,3.0f)/6.0f+pow(_x,2.0f)-2.0f*_x+4.0f/3.0f;
    }
    return 0.0f;
}

Eigen::Vector3f MPM::dInterpolate(float _i, float _j, ngl::Vec3 _x)
{
    return {dBSpline(std::abs(_x.m_x-_i*m_gridsize)/m_gridsize)*bSpline(std::abs(_x.m_y-_j*m_gridsize)/m_gridsize)/m_gridsize,
            bSpline(std::abs(_x.m_x-_i*m_gridsize)/m_gridsize)*dBSpline(std::abs(_x.m_y-_j*m_gridsize)/m_gridsize)/m_gridsize,
            0.0f};
}

float MPM::dBSpline(float _x)
{
    if(_x >= 0.0f && _x < 1.0f)
    {
        return 3.0f/2.0f*pow(_x,2.0f)-2.0f*_x;
    }
    if(_x >= 1.0f && _x < 2.0f)
    {
        return -pow(_x,2.0f)/2.0f+2.0f*_x-2.0f;
    }
    return 0.0f;
}


Eigen::Matrix3f MPM::eigenMat3(ngl::Mat3 _m)
{
    Eigen::Matrix3f m;
    for(int i=0; i<3; ++i)
    {
        for(int j=0; j<3; ++j)
        {
            m(i,j) = _m.m_m[i][j];
        }
    }
    return m;
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
    // collision();
    // updateDeformationGradients();
    // gridToParticle();
}

void MPM::particleToGrid()
{
    std::fill(m_gridMass.begin(), m_gridMass.end(), 0.0f);
    std::fill(m_gridVelocity.begin(), m_gridVelocity.end(), 0.0f);
    
    std::vector<ngl::Vec3> velSum;
    velSum.resize((m_resolutionX+1)*(m_resolutionY+1), 0.0f);

    for(int k=0; k<m_numParticles; ++k)
    {
        int x_index = static_cast<int>(m_position[k].m_x)-1;
        int y_index = static_cast<int>(m_position[k].m_y)-1;
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

    // std::cout<<"*******************************\nMass Field\n";
    // for(int j=m_resolutionY; j>=0; --j)
    // {
    //     for(int i=0; i<m_resolutionX+1; ++i)
    //     {
    //         std::cout<<m_gridMass[j*(m_resolutionX+1)+i]<<' ';
    //     }
    //     std::cout<<'\n';
    // }
    // std::cout<<"*******************************\nVelocity Field X\n";
    // for(int j=m_resolutionY; j>=0; --j)
    // {
    //     for(int i=0; i<m_resolutionX+1; ++i)
    //     {
    //         std::cout<<m_gridVelocity[j*(m_resolutionX+1)+i].m_x<<' ';
    //     }
    //     std::cout<<'\n';
    // }
    // std::cout<<"*******************************\nVelocity Field Y\n";
    // for(int j=m_resolutionY; j>=0; --j)
    // {
    //     for(int i=0; i<m_resolutionX+1; ++i)
    //     {
    //         std::cout<<m_gridVelocity[j*(m_resolutionX+1)+i].m_y<<' ';
    //     }
    //     std::cout<<'\n';
    // }
}

void MPM::computeDensityAndVolume()
{
    m_density.resize(m_numParticles, 0.0f);
    m_volume.resize(m_numParticles, 0.0f);
    for(int k=0; k<m_numParticles; ++k)
    {
        int x_index = static_cast<int>(m_position[k].m_x)-1;
        int y_index = static_cast<int>(m_position[k].m_y)-1;
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
    std::vector<Eigen::Vector3f> force;
    force.resize((m_resolutionX+1)*(m_resolutionY+1), {0.0f, 0.0f, 0.0f});

    for(int k=0; k<m_numParticles; ++k)
    {
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(eigenMat3(m_elastic[k]), Eigen::ComputeFullU | Eigen::ComputeFullV);
        auto mu = m_mu*exp(m_hardening*(1.0f-m_plastic[k].determinant()));
        auto lambda = m_lambda*exp(m_hardening*(1.0f-m_plastic[k].determinant()));
        auto R = svd.matrixU()*svd.matrixV().transpose();
        auto dLeft = 2.0f*mu*(eigenMat3(m_elastic[k])-R)*eigenMat3(m_elastic[k]).transpose();
        auto dRight = (lambda*(m_elastic[k].determinant()-1.0f)*m_elastic[k].determinant())*eigenMat3(ngl::Mat3());
        auto temp = -m_volume[k]*(dLeft+dRight);

        // std::cout<<"mu: "<<mu<<'\n';
        // std::cout<<"lambda: "<<lambda<<'\n';
        // std::cout<<"R: "<<R<<'\n';
        // std::cout<<"dLeft: "<<dLeft<<'\n';
        // std::cout<<"dRight: "<<dRight<<'\n';
        // std::cout<<"temp: "<<temp<<'\n';

        int x_index = static_cast<int>(m_position[k].m_x)-1;
        int y_index = static_cast<int>(m_position[k].m_y)-1;
        for (int i=x_index; i<x_index+4; ++i)
        {
            for (int j=y_index; j<y_index+4; ++j)
            {
                if(i>=0 && i<=m_resolutionX && j>=0 && j<=m_resolutionY)
                {
                    force[j*(m_resolutionX+1)+i] += temp*dInterpolate(i,j,m_position[k]);
                }
            }
        }
    }

    // std::cout<<"*******************************\nForce Field X\n";
    // for(int j=m_resolutionY; j>=0; --j)
    // {
    //     for(int i=0; i<m_resolutionX+1; ++i)
    //     {
    //         std::cout<<force[j*(m_resolutionX+1)+i][0]<<' ';
    //     }
    //     std::cout<<'\n';
    // }
    // std::cout<<"*******************************\nForce Field Y\n";
    // for(int j=m_resolutionY; j>=0; --j)
    // {
    //     for(int i=0; i<m_resolutionX+1; ++i)
    //     {
    //         std::cout<<force[j*(m_resolutionX+1)+i][1]<<' ';
    //     }
    //     std::cout<<'\n';
    // }
    // std::cout<<"*******************************\nForce Field Z\n";
    // for(int j=m_resolutionY; j>=0; --j)
    // {
    //     for(int i=0; i<m_resolutionX+1; ++i)
    //     {
    //         std::cout<<force[j*(m_resolutionX+1)+i][2]<<' ';
    //     }
    //     std::cout<<'\n';
    // }


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
    // Eigen::MatrixXf m = Eigen::MatrixXf::Random(3,3);
    // Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // std::cout << "Here is the matrix m:" << '\n' << m << '\n';
    // std::cout << "Its singular values are:" <<'\n'<< svd.singularValues() <<'\n';
    // std::cout << "Its left singular vectors are the columns of the Full U matrix:" <<'\n'<< svd.matrixU() <<'\n';
    // std::cout << "Its right singular vectors are the columns of the Full V matrix:" <<'\n'<< svd.matrixV() <<'\n';
    // Eigen::MatrixXf m_singularMatt = Eigen::Matrix3f::Zero();
    // for(int i=0; i<3; ++i)
    // {
    //     m_singularMatt(i,i)=svd.singularValues()(i);
    // }
    // std::cout << "Its singular values matrix:" <<'\n'<< m_singularMatt <<'\n';
    // std::cout << "Reconstructed matrix is:" << '\n' << svd.matrixU()*m_singularMatt*svd.matrixV().transpose() << '\n';

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
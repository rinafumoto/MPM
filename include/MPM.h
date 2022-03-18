#ifndef MPM_H_
#define MPM_H_
#include <ngl/Vec3.h>
#include <vector>
#include <random>
#include <ngl/MultiBufferVAO.h>
#include <memory>
#include <Eigen/SVD>

class MPM
{
    public:
        //----------------------------------------------------------------------------------------------------------------------
        /// @brief default constructor
        //----------------------------------------------------------------------------------------------------------------------
        MPM()=default;

        //----------------------------------------------------------------------------------------------------------------------
        /// @brief default copy constructor
        //----------------------------------------------------------------------------------------------------------------------
        MPM(const MPM &)=default;

        void initialise();
        void simulate();
        void render(size_t _w, size_t _h);

    private:
        // Particle properties
        std::vector<ngl::Vec3> m_position;
        std::vector<ngl::Vec3> m_velocity;
        std::vector<float> m_mass;
        std::vector<float> m_volume;
        std::vector<float> m_density;
        std::vector<Eigen::Matrix3f> m_elastic;
        std::vector<Eigen::Matrix3f> m_plastic;

        // Grid properties
        std::vector<float> m_gridMass;
        std::vector<ngl::Vec3> m_gridVelocity_old;
        std::vector<ngl::Vec3> m_gridVelocity;
        std::vector<ngl::Vec3> m_deformationGradientE;
        std::vector<ngl::Vec3> m_deformationGradientP;

        //----------------------------------------------------------------------------------------------------------------------
        /// @brief vector to store the position of solid cells to visualise solid cells.
        //----------------------------------------------------------------------------------------------------------------------
        std::vector<ngl::Vec3> m_solid;
        std::vector<ngl::Vec3> m_normal;
        std::vector<ngl::Vec3> m_indices;


        // Simulation settings
        bool m_first;
        float m_lambda;
        float m_mu;
        float m_hardening;
        float m_gravity;
        float m_compression;
        float m_stretch;
        float m_blending;

        float m_gridsize;
        //----------------------------------------------------------------------------------------------------------------------
        /// @brief the resolution of the simulation
        //----------------------------------------------------------------------------------------------------------------------
        size_t m_resolutionX;
        size_t m_resolutionY;
        //----------------------------------------------------------------------------------------------------------------------
        /// @brief the timestep
        //----------------------------------------------------------------------------------------------------------------------
        float m_timestep;
        //----------------------------------------------------------------------------------------------------------------------
        /// @brief the external force
        //----------------------------------------------------------------------------------------------------------------------
        ngl::Vec3 m_force;
        //----------------------------------------------------------------------------------------------------------------------
        /// @brief the number of particles
        //----------------------------------------------------------------------------------------------------------------------
        size_t m_numParticles;

        //----------------------------------------------------------------------------------------------------------------------
        /// @brief Mersenne Twister pseudo-random generator of 32-bit numbers with a state size of 19937 bits
        //----------------------------------------------------------------------------------------------------------------------
        static std::mt19937 m_generator;

        //----------------------------------------------------------------------------------------------------------------------
        /// @brief multi buffer VAO to store data for the simulation
        //----------------------------------------------------------------------------------------------------------------------
        std::unique_ptr<ngl::MultiBufferVAO> m_vao;

        // Functions
        float interpolate(float _i, float _j, ngl::Vec3 _x);
        float bSpline(float _x);
        ngl::Vec3 dInterpolate(float _i, float _j, ngl::Vec3 _x);
        float dBSpline(float _x);
        Eigen::Vector3f eigenVec3(ngl::Vec3 _v);

        void particleToGrid();
        void computeDensityAndVolume();
        void updateGridVelocity();
        void gridCollision();
        void updateDeformationGradients();
        void gridToParticle();

};

#endif

#ifndef MPM_H_
#define MPM_H_
#include <ngl/Vec3.h>
#include <vector>
#include <random>
#include <ngl/MultiBufferVAO.h>
#include <memory>

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
        void render();

    private:
        // Particle properties
        std::vector<ngl::Vec3> m_position;
        std::vector<ngl::Vec3> m_velocity;
        std::vector<float> m_mass;
        std::vector<float> m_volume;
        std::vector<float> m_density;

        // Grid properties
        std::vector<float> m_gridMass;
        std::vector<ngl::Vec3> m_gridVelocity;
        std::vector<ngl::Vec3> m_deformationGradientE;
        std::vector<ngl::Vec3> m_deformationGradientP;
        //----------------------------------------------------------------------------------------------------------------------
        /// @brief vector for boundary field. 0 for air, 1 for fluid, -1 for solid.
        //----------------------------------------------------------------------------------------------------------------------
        std::vector<int> m_boundary;
        //----------------------------------------------------------------------------------------------------------------------
        /// @brief vector to store the position of solid cells to visualise solid cells.
        //----------------------------------------------------------------------------------------------------------------------
        std::vector<ngl::Vec3> m_solid;


        // Simulation settings
        bool m_first;
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
        float interpolate(float i, float j, ngl::Vec3 x);
        float bSpline(float x);

        void particleToGrid();
        void computeDensityAndVolume();
        void updateGridVelocity();
        void collision();
        void updateDeformationGradients();
        void gridToParticle();

};

#endif
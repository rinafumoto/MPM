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
        void render();

    private:
        // Particle properties
        std::vector<ngl::Vec3> m_position;
        std::vector<ngl::Vec3> m_velocity;
        std::vector<float> m_mass;
        std::vector<float> m_volume;
        std::vector<float> m_density;
        std::vector<ngl::Mat3> m_elastic;
        std::vector<ngl::Mat3> m_plastic;

        // Grid properties
        std::vector<float> m_gridMass;
        std::vector<ngl::Vec3> m_gridVelocity;
        std::vector<ngl::Vec3> m_deformationGradientE;
        std::vector<ngl::Vec3> m_deformationGradientP;
        //----------------------------------------------------------------------------------------------------------------------
        /// @brief vector for boundary field. -1 for solid.
        //----------------------------------------------------------------------------------------------------------------------
        std::vector<int> m_boundary;
        //----------------------------------------------------------------------------------------------------------------------
        /// @brief vector to store the position of solid cells to visualise solid cells.
        //----------------------------------------------------------------------------------------------------------------------
        std::vector<ngl::Vec3> m_solid;
        std::vector<ngl::Vec3> m_normal;


        // Simulation settings
        bool m_first;
        float m_lambda;
        float m_mu;
        float m_hardening;
        float m_gravity;

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
        Eigen::Vector3f dInterpolate(float _i, float _j, ngl::Vec3 _x);
        float dBSpline(float _x);
        Eigen::Matrix3f eigenMat3(ngl::Mat3 _m);

        void particleToGrid();
        void computeDensityAndVolume();
        void updateGridVelocity();
        void collision();
        void updateDeformationGradients();
        void gridToParticle();

};

#endif

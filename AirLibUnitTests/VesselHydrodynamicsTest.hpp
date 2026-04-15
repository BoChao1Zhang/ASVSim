#ifndef msr_AirLibUnitTests_VesselHydrodynamicsTest_hpp
#define msr_AirLibUnitTests_VesselHydrodynamicsTest_hpp

#include <cmath>
#include <limits>
#include <string>

#include "TestBase.hpp"
#include "vehicles/vessel/AbstractTau.hpp"
#include "vehicles/vessel/HydrodynamicsWrench.hpp"
#include "vehicles/vessel/hydrodynamics/FossenCurrent.hpp"
#include "vehicles/vessel/parameters/MilliAmpereParams.hpp"

namespace msr
{
namespace airlib
{

class VesselHydrodynamicsTest : public TestBase
{
public:
    void run() override
    {
        testHydrodynamicsWrenchUsesCurrentSample();
        testHydrodynamicsWrenchClampsInvalidForce();
        testFossenCurrentNominalMilliAmpereResponse();
        testFossenCurrentRejectsAbnormalVelocity();
    }

private:
    static bool isNear(real_T lhs, real_T rhs, real_T tolerance)
    {
        return std::abs(lhs - rhs) <= tolerance;
    }

    void testVectorNear(const Vector3r& actual, const Vector3r& expected, real_T tolerance, const std::string& label)
    {
        testAssert(isNear(actual.x(), expected.x(), tolerance), label + " x mismatch");
        testAssert(isNear(actual.y(), expected.y(), tolerance), label + " y mismatch");
        testAssert(isNear(actual.z(), expected.z(), tolerance), label + " z mismatch");
    }

    void testHydrodynamicsWrenchUsesCurrentSample()
    {
        HydroDynamicsWrench wrench(Vector3r::Zero(), Vector3r(-1, 0, 0));
        wrench.setComputationMethod([]() {
            return Vector3r(1.0f, 2.0f, 3.0f);
        });

        wrench.reset();
        wrench.update(0.01f);

        const auto applied = wrench.getWrench();
        testVectorNear(applied.force, Vector3r(1.0f, 2.0f, 0.0f), 1.0e-6f, "HydrodynamicsWrench force");
        testVectorNear(applied.torque, Vector3r(0.0f, 0.0f, 3.0f), 1.0e-6f, "HydrodynamicsWrench torque");
    }

    void testHydrodynamicsWrenchClampsInvalidForce()
    {
        HydroDynamicsWrench wrench(Vector3r::Zero(), Vector3r(-1, 0, 0));
        wrench.setComputationMethod([]() {
            return Vector3r(std::numeric_limits<real_T>::infinity(), 0.0f, 0.0f);
        });

        wrench.reset();
        wrench.update(0.01f);

        const auto applied = wrench.getWrench();
        testVectorNear(applied.force, Vector3r::Zero(), 1.0e-6f, "HydrodynamicsWrench invalid force");
        testVectorNear(applied.torque, Vector3r::Zero(), 1.0e-6f, "HydrodynamicsWrench invalid torque");
    }

    void testFossenCurrentNominalMilliAmpereResponse()
    {
        MilliAmpereParams params;
        FossenCurrent hydrodynamics;
        AbstractHydrodynamics& hydrodynamics_api = hydrodynamics;
        hydrodynamics.setVesselParameters(&params);

        Kinematics::State state = Kinematics::State::zero();
        state.twist.linear = Vector3r(2.0f, 0.5f, 0.0f);
        state.twist.angular = Vector3r(0.0f, 0.0f, 0.1f);

        hydrodynamics.updateState(state, Vector2r::Zero());
        hydrodynamics_api.computeDamping();
        hydrodynamics_api.computeCoriolis();

        testVectorNear(hydrodynamics.getDampingForce(), Vector3r(607.24f, 105.728f, 8.733f), 1.0e-3f, "FossenCurrent damping");
        testVectorNear(hydrodynamics.getCoriolisForce(), Vector3r(-120.5f, 480.0f, 10.0f), 1.0e-3f, "FossenCurrent coriolis");
    }

    void testFossenCurrentRejectsAbnormalVelocity()
    {
        MilliAmpereParams params;
        FossenCurrent hydrodynamics;
        AbstractHydrodynamics& hydrodynamics_api = hydrodynamics;
        hydrodynamics.setVesselParameters(&params);

        Kinematics::State state = Kinematics::State::zero();
        state.twist.linear = Vector3r(20000.0f, 0.0f, 0.0f);

        hydrodynamics.updateState(state, Vector2r::Zero());
        hydrodynamics_api.computeDamping();
        hydrodynamics_api.computeCoriolis();

        testVectorNear(hydrodynamics.getDampingForce(), Vector3r::Zero(), 1.0e-6f, "FossenCurrent abnormal damping");
        testVectorNear(hydrodynamics.getCoriolisForce(), Vector3r::Zero(), 1.0e-6f, "FossenCurrent abnormal coriolis");
    }
};

}
}

#endif

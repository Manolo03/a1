#pragma once

#include <locomotion/LocomotionController.h>
#include <locomotion/LocomotionTrajectoryPlanner.h>
#include <robot/RB.h>
#include <robot/RBJoint.h>
#include <gui/model.h>
#include <gui/renderer.h>

namespace crl {

/**
 * A controller that kinematically "tracks" the objectives output by a
 * locomotion trajectory generator
 */
class KinematicTrackingController : public LocomotionController {
public:
    LeggedRobot *robot;
    IK_Solver *ikSolver = nullptr;
    std::vector<dVector> trajectory;

public:
    /**
     * constructor
     */
    KinematicTrackingController(LocomotionTrajectoryPlanner *planner)
        : LocomotionController(planner) {
        this->robot = planner->robot;
        ikSolver = new IK_Solver(robot);
        trajectory.clear();
    }

    /**
     * destructor
     */
    virtual ~KinematicTrackingController(void) { delete ikSolver; }

    void generateMotionTrajectories(double dt = 1.0 / 30) override {
        planner->planGenerationTime = planner->simTime;
        planner->generateTrajectoriesFromCurrentState(dt);
    }

    void computeAndApplyControlSignals(double dt, bool recordTrajectory) override {
        crl::gui::MyModel test1;
        P3D hitpoint;
        // set base pose. in this assignment, we just assume the base perfectly
        // follow target base trajectory.
        P3D targetPos =
            planner->getTargetTrunkPositionAtTime(planner->getSimTime() + dt);
        Quaternion targetOrientation = planner->getTargetTrunkOrientationAtTime(
            planner->getSimTime() + dt);
        
        if(test1.mesh.hitByRay(P3D(targetPos[0], -5, targetPos[2]), V3D(0,1,0), hitpoint)){
            robot->setRootState(targetPos + P3D(0,hitpoint.y, 0), targetOrientation);
        }
        else{
            robot->setRootState(targetPos, targetOrientation);
        }
        

        // now we solve inverse kinematics for each limbs
        for (uint i = 0; i < robot->limbs.size(); i++) {
            P3D target = planner->getTargetLimbEEPositionAtTime(
                robot->limbs[i], planner->getSimTime() + dt);

            // crl::gui::MySphere test1;
            // test1.modify(P3D(0, -2.5, 2.5), 3);

            
            if(test1.mesh.hitByRay(P3D(target[0], -5, target[2]), V3D(0,1,0), hitpoint)){
                ikSolver->addEndEffectorTarget(
                    robot->limbs[i]->eeRB, robot->limbs[i]->ee->endEffectorOffset,
                    target + P3D(0, hitpoint.y, 0));
            }
            else{
                ikSolver->addEndEffectorTarget(
                    robot->limbs[i]->eeRB, robot->limbs[i]->ee->endEffectorOffset,
                    target);
            }
            // ikSolver->addEndEffectorTarget(
            //     robot->limbs[i]->eeRB, robot->limbs[i]->ee->endEffectorOffset,
            //     target);

            
                //target + offset); 
        }

        dVector q;
        ikSolver->solve(q);
        if (recordTrajectory) {
            trajectory.push_back(q);
        }
    }

    void advanceInTime(double dt) override { planner->advanceInTime(dt); }

    void resetRecordedTrajectory() { trajectory.clear(); }

    void drawDebugInfo(gui::Shader *shader) override {
        planner->drawTrajectories(shader);
    }

    void plotDebugInfo() override {
        // add plot if you need...
    }

};

}  // namespace crl
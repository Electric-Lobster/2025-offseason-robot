// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SubSystems.DriveTrain;

/** Container for all auto programs */
public class AutoContainer {

    DriveTrain driveTrain;

    /**
     * https://choreo.autos/choreolib/auto-factory/
     */
    AutoFactory autoFactory = new AutoFactory(
        driveTrain::getPose,
        driveTrain::resetPose,
        driveTrain::followTrajectory,
        true,
        driveTrain
    );

    public AutoContainer(DriveTrain m_driveTrain) {
        this.driveTrain = m_driveTrain;
    }


    /**
     * test trajectory 1 
     * drives straight
     */
    public AutoRoutine test1() {
        //Creates the routine
        AutoRoutine routine = autoFactory.newRoutine("Test 1");

        //Creates the trajectories
        AutoTrajectory test1 = routine.trajectory("Test 1");

        //starts trajectory
        routine.active().onTrue(
            Commands.sequence(
                test1.resetOdometry(),
                test1.cmd()
            )
        );

        return routine;
    }


    /**
     * test trajectory 2
     * turns left 90 degrees
     */
    public AutoRoutine test2() {
        //Creates the routine
        AutoRoutine routine = autoFactory.newRoutine("Test 2");

        //Creates the trajectories
        AutoTrajectory test2 = routine.trajectory("Test 2");

        //starts trajectory
        routine.active().onTrue(
            Commands.sequence(
                test2.resetOdometry(),
                test2.cmd()
            )
        );

        return routine;
    }

}

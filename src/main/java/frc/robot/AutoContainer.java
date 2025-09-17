// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import frc.robot.SubSystems.DriveTrain;

/** Container for all auto programs */
public class AutoContainer {

    DriveTrain driveTrain;

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
}

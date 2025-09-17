// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.SubSystems.DriveTrain;

/** Container for all auto programs */
public class AutoContainer {

    DriveTrain driveTrain;

    public AutoContainer(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }
}

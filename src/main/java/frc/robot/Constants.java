// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public final double ksVolts = 0.11787;
    public final double kvVolts = 0.05655;
    public final double kaVolts = 0.0030613;
    public final double kpDriveVel = 0.000022962;

    public final double _trackWidthMeters = Units.inchesToMeters(24);
    public final DifferentialDriveKinematics _diffDriveKinematics = new DifferentialDriveKinematics(_trackWidthMeters);

    public final double ramseteB = 2;
    public final double ramseteZeta = 0.7;
}

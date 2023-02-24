// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import frc.TrajectoryHelper;

/** Add your docs here. */
public class Constants {
    public final static double ksVolts = 0.11787;
    public final static double kvVolts = 0.05655;
    public final static double kaVolts = 0.0030613;
    public final static double kpDriveVel = 0.000022962;

    public final static double _trackWidthMeters = Units.inchesToMeters(24);
    public final static DifferentialDriveKinematics _diffDriveKinematics = new DifferentialDriveKinematics(_trackWidthMeters);

    public final static double ramseteB = 2;
    public final static double ramseteZeta = 0.7;


    public static Trajectory CurveThenBalance = TrajectoryHelper.generateFromPathPlanner("New New Path");
    public static Trajectory goStraight = TrajectoryHelper.generateFromPathPlanner("New Path");
}

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
    public final static double ksVolts = 0.080799;
    public final static double kvVolts = 0.070717;
    public final static double kaVolts = 0.0080209;
    public final static double kpDriveVel = 0.000022962;

    public final static double _trackWidthMeters = Units.inchesToMeters(25.129);
    public final static DifferentialDriveKinematics _diffDriveKinematics = new DifferentialDriveKinematics(_trackWidthMeters);

    public final static double ramseteB = 2;
    public final static double ramseteZeta = 0.7;


    public static Trajectory CurveThenBalance = TrajectoryHelper.generateFromPathPlanner("Curvethenbalance");
    public static Trajectory Straight5meters = TrajectoryHelper.generateFromPathPlanner("Straight5meters");
    public static Trajectory Straight6meters = TrajectoryHelper.generateFromPathPlanner("Straight6meters");
    public static Trajectory Straight4meters = TrajectoryHelper.generateFromPathPlanner("Straight4meters");
}

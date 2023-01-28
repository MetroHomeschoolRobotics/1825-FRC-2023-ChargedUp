// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    //FeedForward Constansts
    public static final double ksVolts = 0.12027;
    public static final double kvVoltSecondsPerMeter = 2.2248;
    public static final double kaVoltSecondsSquaredPerMeter = 0.12052;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 3.5;

    //differentialdrivekinematics
    //https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/entering-constants.html#differentialdrivekinematics
    public static final double kTrackwidthMeters = 0.7651;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    //Max Trajectory Velocity/Acceleration
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    //if tuning these values see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html#constructing-the-ramsete-controller-object
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}

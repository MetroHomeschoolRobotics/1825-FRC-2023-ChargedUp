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
    public final static double ksVolts = 0.16988; //0.1667; //0.013659;
    public final static double kvVolts = 2.2586*1.25; //1.0577; //1.0438;
    public final static double kaVolts = 0.49195; //0.27778; //0.18004;
    public final static double kpDriveVel2 = 0.22962;

    public final static double kpDriveVel = 0.51425; //0.18391; //1.7218E-7;//0.000022962
    public final static double rampTimeSec = 0.25;

    public final static double _trackWidthMeters = Units.inchesToMeters(22.0);
    public final static DifferentialDriveKinematics _diffDriveKinematics = new DifferentialDriveKinematics(_trackWidthMeters);

    public final static double ramseteB = 2; //2;  //10
    public final static double ramseteZeta = 0.7;  //0.7;  //1


    public static Trajectory CurveThenBalance = TrajectoryHelper.generateFromPathPlanner("Curvethenbalance");
    public static Trajectory Straight5meters = TrajectoryHelper.generateFromPathPlanner("Straight5meters");
    public static Trajectory Straight6meters = TrajectoryHelper.generateFromPathPlanner("Straight6meters");
    public static Trajectory Straight4meters = TrajectoryHelper.generateFromPathPlanner("Straight4meters");
    public static Trajectory ForwardthenBack = TrajectoryHelper.generateFromPathPlanner("ForwardthenBack");
    public static Trajectory Turn = TrajectoryHelper.generateFromPathPlanner("Turn");
    public static double armExtendRetractSeconds = 3.1;

    public static final int BeamBreakSensor = 1;
    public static final int ToFSensor = 0;

    public static final double nodeAprilHeightIn = 14.25;
    public static final double substationAprilHeightIn = 23.375;


    public static final double GrabConeHeight = (Units.inchesToMeters(RobotMap.shoulderHeight)*100)-(24);
    
}

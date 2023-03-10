package frc;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AddTrajectoryToField;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryHelper {
  /**
   * Generates a trajectory from the given waypoints and settings.
   * 
   * @param start         The starting pose.
   * @param waypoints     The interior waypoints.
   * @param end           The ending pose.
   * @param reversed      If the trajectory should be run backwards.
   * @param maxVelocity   The max speed the robot should go.
   * @param maxAccel      The max acceleration you want the robot to go.
   * @param startVelocity The speed the robot starts the trajectory at.
   * @param endVelocity   The speed the robot finishes the trajectory at.
   * @param maxVoltage    The maximum voltage the motors can use; will affect
   *                      acceleration.
   * @return The generated trajectory.
   */
  //private static Drivetrain r_drivetrain= new Drivetrain();

  public final static Trajectory generateTrajectory(Pose2d start, List<Translation2d> waypoints, Pose2d end,
      boolean reversed, double maxVelocity, double maxAccel, double maxCentripAccel, double startVelocity,
      double endVelocity, double maxVoltage) {
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVolts, Constants.kaVolts),
        Constants._diffDriveKinematics, maxVoltage);
    CentripetalAccelerationConstraint autoTurningConstraint = new CentripetalAccelerationConstraint(maxCentripAccel);
    TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(Constants._diffDriveKinematics);
    config.addConstraint(autoVoltageConstraint);
    config.addConstraint(autoTurningConstraint);
    config.setStartVelocity(startVelocity);
    config.setEndVelocity(endVelocity);
    config.setReversed(reversed);
    return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
  }

public final static Trajectory generateFromPathPlanner(String trajectoryJSON){
  // trajectoryJSON = "pathplanner/generatedJSON/"+trajectoryJSON+".wpilib.json";
  // Trajectory trajectory;
  // try {
  // Path trajectoryPath =
  // Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  // trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  // } catch (IOException ex) {
  // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
  // ex.getStackTrace());
  // TrajectoryConfig config = new TrajectoryConfig(1, 1);
  // trajectory= TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(),
  // new Pose2d(), config);
  // }
  // return trajectory;
  return TrajectoryGenerator.generateTrajectory(
  // Start at the origin facing the +X direction
  new Pose2d(0,0,new Rotation2d(0)),
  // Pass through these two interior waypoints, making an 's' curve path
  List.of(new Translation2d(1,0),new Translation2d(2,0)),
  // End 3 meters straight ahead of where we started, facing forward
  new Pose2d(3,0,new Rotation2d(180)),
  // Pass config
  new TrajectoryConfig(1,3)
  // Add kinematics to ensure max speed is actually obeyed
  .setKinematics(Constants._diffDriveKinematics)
  // Apply the voltage constraint
  );
  //var Traj2 = TrajectoryGenerator.generateTrajectory(
    
  //new Pose2d(3,0,new Rotation2d(180)),

  //List.of(new Translation2d(2,0),new Translation2d(1,0)),
    // End 3 meters straight ahead of where we started, facing forward
  //new Pose2d(0,0,new Rotation2d(180)),
    // Pass config
  //new TrajectoryConfig(1,3)
    // Add kinematics to ensure max speed is actually obeyed
  //.setKinematics(Constants._diffDriveKinematics)
  //);
  //var concatTraj = Traj1.concatenate(Traj2);
  //return concatTraj;

  }
public final static void putTrajectoryOnField(Trajectory trajectory, Field2d field) {
    field.getObject("traj").setTrajectory(trajectory);
  }

/**
 * Creates a command that runs the given trajectory. 
 * @param trajectoryToFollow The trajectory the robot should follow. 
 * @return A RamseteCommand that follows the specified trajectory.
 */
  public final static Command createTrajectoryCommand(Trajectory _trajectoryToFollow) {
      var leftController = new PIDController(Constants.kpDriveVel, 0, 0);
      var rightController = new PIDController(Constants.kpDriveVel, 0, 0);
      //RobotContainer.s_drivetrain.getField2d().getObject("traj").setTrajectory(_trajectoryToFollow);
      RamseteController ramseteController = new RamseteController(Constants.ramseteB, Constants.ramseteZeta);
      //ramseteController.setEnabled(false);
      RamseteCommand ramseteCommand = new RamseteCommand(
        _trajectoryToFollow,
        RobotContainer.r_drivetrain::getPosition,
        ramseteController,
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVolts,Constants.kaVolts),
        Constants._diffDriveKinematics,
        RobotContainer.r_drivetrain::getWheelSpeed,
        leftController,
        rightController,
        (leftVolts, rightVolts) -> {
          RobotContainer.r_drivetrain.tankDriveVolts(leftVolts, rightVolts);
        },
        RobotContainer.r_drivetrain);
        return new AddTrajectoryToField(RobotContainer.r_drivetrain.getField2d(), _trajectoryToFollow).andThen(ramseteCommand);
      //r_drivetrain.resetOdometry(generateFromPathPlanner("hi"));
    }
  
}

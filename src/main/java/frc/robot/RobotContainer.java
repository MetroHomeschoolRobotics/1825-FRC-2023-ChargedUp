// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.autoBalance;
import frc.robot.commands.moveArmPos;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.TimeofFlight;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.DriveToApril;
import frc.robot.commands.ExtendToPoint;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.RetractArm;
import frc.TrajectoryHelper;
import frc.robot.commands.Grabber;
import frc.robot.commands.ReflectivePipeline;
import frc.robot.commands.ToggleCompressor;
import frc.robot.commands.AprilTagPipeline;
import frc.robot.commands.ArmMovement;
import frc.robot.commands.ArmStability;
import frc.robot.commands.AutoGrabber;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_manipulatorController = new CommandXboxController(1);

  public static final Pneumatics pneumatics = new Pneumatics();
  private static final Arm arm = new Arm();
  public static final Drivetrain r_drivetrain = new Drivetrain();
  private final Limelight limelight = new Limelight(r_drivetrain);
  private final DriveTeleop r_teleop = new DriveTeleop(r_drivetrain, m_driverController);
  private final ArmMovement armRotation = new ArmMovement(m_manipulatorController, arm);
  private final TimeofFlight grabbersensor = new TimeofFlight();


  SendableChooser<Command> _autoChooser = new SendableChooser<>();

  private void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(r_drivetrain, r_teleop);
    CommandScheduler.getInstance().setDefaultCommand(arm, armRotation);
  }

  private void init() {
    r_drivetrain.resetHeading();
    pneumatics.setGrabberClose();
    setDefaultCommands();
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // get the auto chooser options
    getAutoChooserOptions();
    init();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  // Possible path planner things:
  public Command loadPathPlannerTrajectoryToRamseteCommand(String filename, boolean resetOdometry) {
    filename = "pathplanner/generatedJSON/"+filename+".wpilib.json";
    Trajectory trajectory;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) {
      DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
      System.out.println("Unable to read from file " + filename);
      return new InstantCommand();
    }

    RamseteController ramseteController = new RamseteController(Constants.ramseteB, Constants.ramseteZeta);

    ramseteController.setTolerance(new Pose2d(0.00001,0.00001, new Rotation2d(1)));

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, r_drivetrain::getPosition,
        ramseteController,
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVolts, Constants.kaVolts),
        Constants._diffDriveKinematics,r_drivetrain::getWheelSpeed, new PIDController(Constants.kpDriveVel, 0, 0),
        new PIDController(Constants.kpDriveVel, 0, 0), (leftVolts, rightVolts)-> r_drivetrain.tankDriveVolts(leftVolts,rightVolts), r_drivetrain);
    
    if(resetOdometry){
      return new SequentialCommandGroup(new InstantCommand(()->r_drivetrain.resetOdometry(trajectory.getInitialPose())),ramseteCommand);
    }else{
      return ramseteCommand;
    }
  }

  //////////////////////////////////////////////////////////////////////////
  /*  autoChooserOptions                                                  */
  //////////////////////////////////////////////////////////////////////////
  private void getAutoChooserOptions() {
    _autoChooser.setDefaultOption("No Autonomous", new WaitCommand(15));

    _autoChooser.addOption("Go forward, turn, and dock (L)", loadPathPlannerTrajectoryToRamseteCommand("TurnPath", true).andThen(new autoBalance(r_drivetrain))); 

    _autoChooser.addOption("Go forward, turn, and dock (R)", loadPathPlannerTrajectoryToRamseteCommand("CurveThenBalanceR", true).andThen(new autoBalance(r_drivetrain)));

    _autoChooser.addOption("Score, exit community, and dock (L)",((new moveArmPos(arm, 55, 141).raceWith(new WaitCommand(2.1))).andThen(new Grabber(pneumatics))).andThen(new WaitCommand(0.05)).andThen(new moveArmPos(arm, 0, 10).raceWith(new WaitCommand(2.1))).andThen(loadPathPlannerTrajectoryToRamseteCommand("TurnPath", true)).andThen(new autoBalance(r_drivetrain)));
    
    _autoChooser.addOption("Score, exit community, and dock (R)",((new moveArmPos(arm, 55, 141).raceWith(new WaitCommand(2.1))).andThen(new Grabber(pneumatics))).andThen(new WaitCommand(0.05)).andThen(new moveArmPos(arm, 0, 10).raceWith(new WaitCommand(2.1))).andThen(loadPathPlannerTrajectoryToRamseteCommand("CurveThenBalanceR", true)).andThen(new autoBalance(r_drivetrain)));
    
    _autoChooser.addOption("Score, exit community", ((new moveArmPos(arm, 55,141).raceWith(new WaitCommand(2.1))).andThen(new Grabber(pneumatics))).andThen(new WaitCommand(0.05)).andThen(new moveArmPos(arm, 0, 10).raceWith(new WaitCommand(2.1))).andThen(loadPathPlannerTrajectoryToRamseteCommand("Straight4meters", true)));
    
    _autoChooser.addOption("Score", ((new moveArmPos(arm, 55,141).raceWith(new WaitCommand(2.1))).andThen(new Grabber(pneumatics))).andThen(new WaitCommand(0.05)).andThen(new moveArmPos(arm, 0, 10).raceWith(new WaitCommand(2.1))));
    //_autoChooser.addOption("Score, exit community, dock in middle", ((new moveArmPos(arm, 55,141).raceWith(new WaitCommand(2.1))).andThen(new Grabber(pneumatics))).andThen(new WaitCommand(0.05)).andThen(new moveArmPos(arm, 0, 10).raceWith(new WaitCommand(2.1))).andThen(loadPathPlannerTrajectoryToRamseteCommand("Straight4meters", true)).andThen(loadPathPlannerTrajectoryToRamseteCommand("ReturnToDock", true)).andThen(new autoBalance(r_drivetrain)));
    
    SmartDashboard.putData(_autoChooser);
  } 

  private void configureBindings() {
    ///////////////// Driver controller ///////////////////
    m_driverController.a().whileTrue(new autoBalance(r_drivetrain))
        .whileFalse(new DriveTeleop(r_drivetrain, m_driverController));
    m_driverController.x().whileTrue(new AprilTagPipeline(limelight));
    m_driverController.b().whileTrue(new ReflectivePipeline(limelight));
    m_driverController.leftBumper().whileTrue(new DriveToApril(r_drivetrain, limelight));


    ///////////////// Manipulator controller //////////////////
    Trigger BeamBreakDetector = new Trigger(() -> !arm.getBeamBreakSensor());
    m_manipulatorController.back().whileTrue(new ToggleCompressor(pneumatics));
    // TODO changed this to onTrue and took out the trigger/put it in the command and commented it
    //m_manipulatorController.povDown()/*.and(BeamBreakDetector)*/.whileTrue(new RetractArm(m_driverController, arm, -0.5));
    m_manipulatorController.leftBumper().whileTrue(new AutoGrabber(pneumatics, grabbersensor));
    m_manipulatorController.rightBumper().whileTrue(new Grabber(pneumatics));
    // TODO changed this to run moveArmPos
    m_manipulatorController.y().whileTrue(new moveArmPos(arm, 55, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // this runs the selected autonomous command☺
    return _autoChooser.getSelected();
  }
}

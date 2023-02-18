// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutonomousExperiment;
import frc.robot.commands.autoBalance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.ResetOdometry;
import frc.TrajectoryHelper;
import frc.robot.commands.AutoTurnExperiment;

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

  public static final Drivetrain r_drivetrain = new Drivetrain();
  private final DriveTeleop r_teleop = new DriveTeleop(r_drivetrain, m_driverController);

  SendableChooser<Command> _autoChooser = new SendableChooser<>();

  private void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(r_drivetrain, r_teleop);
  }

  private void init() {
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

  private void getAutoChooserOptions() {
    _autoChooser.setDefaultOption("No Autonomous", new WaitCommand(15));

    _autoChooser.addOption("Autonomous Test", new AutoTurnExperiment(r_drivetrain, 90));
    
    _autoChooser.addOption("Fowards Auto", new AutonomousExperiment(r_drivetrain, 5, 0));

    _autoChooser.addOption("Backwards Auto", new AutonomousExperiment(r_drivetrain, -5, 0));

    _autoChooser.addOption("TrajectoryTest", new  ResetOdometry(Constants.CurveThenBalance.sample(0).poseMeters, r_drivetrain).andThen(TrajectoryHelper.createTrajectoryCommand(Constants.CurveThenBalance)).andThen(new autoBalance(r_drivetrain)));

    SmartDashboard.putData(_autoChooser);
  }

  private void configureBindings() {
    // make 'B' turn on autoBalance when held
    m_driverController.b().whileTrue(new autoBalance(r_drivetrain))
        .whileFalse(new DriveTeleop(r_drivetrain, m_driverController));

    m_driverController.a().onTrue(new  ResetOdometry(Constants.goStraight.sample(0).poseMeters, r_drivetrain).andThen(TrajectoryHelper.createTrajectoryCommand(Constants.goStraight)).andThen(new autoBalance(r_drivetrain)));
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

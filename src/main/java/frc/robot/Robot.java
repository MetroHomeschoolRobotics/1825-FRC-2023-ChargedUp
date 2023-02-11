// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
      // PID constants should be tuned per robot

      final double LINEAR_P = 0.0000000017;

      final double LINEAR_D = 0.0025;
  
      PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
  
  
      final double ANGULAR_P = 0.1;//working on finding
  
      final double ANGULAR_D = 0.0;//working on finding
  XboxController xboxController = new XboxController(0);
  Vision v = new Vision();
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  Drivetrain drive = new Drivetrain();

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override

  public void teleopPeriodic() {

      double forwardSpeed;

      double rotationSpeed;


      forwardSpeed = -xboxController.getRightY();


      if (xboxController.getAButton()) {

          // Vision-alignment mode

          // Query the latest result from PhotonVision

          var result = v.aprilTagCam.getLatestResult();


          if (result.hasTargets()) {

              // Calculate angular turn power

              // -1.0 required to ensure positive PID controller effort _increases_ yaw

              rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);

          } else {

              // If we have no targets, stay still.

              rotationSpeed = 0;

          }

      } else {

          // Manual Driver Mode

          rotationSpeed = xboxController.getLeftX();

      }


      // Use our forward/turn speeds to control the drivetrain

      drive.autoDrive(forwardSpeed, rotationSpeed);
    }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

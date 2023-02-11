// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class autoBalance extends CommandBase {
  // the first working value was (0.005,0,0);second was (0.008,0,0); critical gain = 0.0095
  private double critGain = 0.01;
  private double period = .96;
  private double kp = 0.6*critGain;
  private double ki = (2*kp/period)*0; // this is the formula multiplyed by zero (to keep the formula in tact)
  private double kd = 0.125*kp*period;



  private PIDController _PIDController = new PIDController(kp, ki, kd);
  private Drivetrain drivetrain;

  /** Creates a new autoBalance. */
  public autoBalance(Drivetrain _drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = _drivetrain;
    addRequirements(_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = drivetrain.getPitchAngle();
    System.out.println(angle);




    drivetrain.autoDrive(MathUtil.clamp(-(_PIDController.calculate(drivetrain.getPitchAngle())),-0.2,0.2), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveMovement(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

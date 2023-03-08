// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.DriverAction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class DriveToApril extends CommandBase {

  private Drivetrain drivetrain;
  private Limelight limelight;

  private static double kp = 0.6;
  private static double ki = 0;
  private static double kd = 0;
  private PIDController pidR = new PIDController(Constants.kpDriveVel, 0, 0);
  private PIDController pidL = new PIDController(Constants.kpDriveVel, 0, 0);
  private PIDController turnPID = new PIDController(0.005, ki, kd);

  private static double distance;
  private static double turnSpeed;
  private static double speedR;
  private static double speedL;
  private static double TY;
  /** Creates a new AprilTag. */
  public DriveToApril(Drivetrain _Drivetrain, Limelight _limelight) {
    drivetrain = _Drivetrain;
    limelight = _limelight;
    addRequirements(_limelight);
    addRequirements(_Drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = limelight.getNodeAprilDistance()-1;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(-limelight.getTargetYaw());

    if(limelight.hasTargests()){
      turnSpeed = turnPID.calculate(limelight.getTargetYaw(), 2.7);
      System.out.println(limelight.getTargetYaw());
    }else{
      turnSpeed = 0;
      drivetrain.getSignal();
    }
    

    drivetrain.autoTurnDrive(turnSpeed);
    // speedR = -MathUtil.clamp(pidR.calculate(drivetrain.getAverageEncoderDistance(), distance), -0.1, 0.1);
    // speedL = -MathUtil.clamp(pidL.calculate(drivetrain.getAverageEncoderDistance(), distance), -0.1, 0.1);
    // drivetrain.tankDriveVolts(speedL, speedR);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDriveVolts(0, 0);  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidR.atSetpoint();
  }
}

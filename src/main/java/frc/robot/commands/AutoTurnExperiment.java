// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;

public class AutoTurnExperiment extends CommandBase {

  private double critGain = 0.005;  // multiply this for faster speeds if needed. (it will ocillate more when tipped)
  private double period = .62;
  private double kp = 0.6*critGain;
  private double ki = (2*kp/period)*0; // this is the formula multiplied by zero (to keep the formula intact)
  private double kd = 0.125*kp*period;

  private Drivetrain _drivetrain;           //outputs a number of distance          outputs how fast you're moving away
//                                            Probably must be below 1
  private PIDController _PIDController = new PIDController(kp, ki*0, kd);


  Double turnAngle;

  /** Creates a new AutoTurnExperiment. */
  public AutoTurnExperiment(Drivetrain drivetrain, double _turnAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    turnAngle = _turnAngle;
    _drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _drivetrain.resetHeading();
    _drivetrain.resetEncoders();
    _PIDController.enableContinuousInput(-180, 180);
    _PIDController.setTolerance(2,5);
    _PIDController.setIntegratorRange(-.5, .5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double turning = _PIDController.calculate(_drivetrain.getHeading(),turnAngle);
    Double forward = MathUtil.clamp(_PIDController.calculate(.5*(_drivetrain.getDistanceL()+_drivetrain.getDistanceR()), turnAngle),-.5,.5);
    
    SmartDashboard.putNumber("Turning Speed", turning);
    System.out.println(_drivetrain.getHeading());

    Double SDistance = _drivetrain.getDistanceL();



    _drivetrain.autoTurnDrive(MathUtil.clamp(_PIDController.calculate(_drivetrain.getHeading(),turnAngle), -0.3, 0.3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.autoTurnDrive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _PIDController.atSetpoint();
  }
}

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

  // Ziegler-Nichols PID controller tuning
  // https://engineering.purdue.edu/~zak/ECE_382-Fall_2018/IntroPID_16.pdf
  private double critGain = 0.037037;  // Max speed at 45 degree difference
  private double period = 1.0;
  private double kp = 0.6*critGain;
  private double ki = (2*kp/period); // this is the formula multiplied by zero (to keep the formula intact)
  private double kd = 0.125*kp*period;

  private Drivetrain _drivetrain;           //outputs a number of distance          outputs how fast you're moving away
//                                            Probably must be below 1
  private PIDController _PIDController = new PIDController(kp, ki, kd);


  private Double _turnAngle;

  /** Creates a new AutoTurnExperiment. */
  public AutoTurnExperiment(Drivetrain drivetrain, double turnAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    _turnAngle = turnAngle;
    _drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _drivetrain.resetHeading();
    _drivetrain.resetEncoders();
    _PIDController.enableContinuousInput(-360, 360);
    _PIDController.setTolerance(2,5);
    _PIDController.setIntegratorRange(-.5, .5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Double heading = -_drivetrain.getHeading();
    final Double turning = _PIDController.calculate(heading,_turnAngle);
    //Double forward = MathUtil.clamp(_PIDController.calculate(.5*(_drivetrain.getDistanceL()+_drivetrain.getDistanceR()), turnAngle),-.5,.5);
    
    SmartDashboard.putNumber("Turning Speed", turning);
   // System.out.println("target " + String.valueOf(_turnAngle) + " heading " + String.valueOf(heading) 
   // + " output " + String.valueOf(turning));

    Double SDistance = _drivetrain.getDistanceL();



    //_drivetrain.autoTurnDrive(MathUtil.clamp(_PIDController.calculate(_drivetrain.getHeading(),turnAngle), -0.3, 0.3));
    final double maxRate = 1.0;
    _drivetrain.autoTurnDrive(MathUtil.clamp(turning, -maxRate, maxRate));
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

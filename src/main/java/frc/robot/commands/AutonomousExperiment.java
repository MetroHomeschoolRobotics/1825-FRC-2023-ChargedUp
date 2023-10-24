// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutonomousExperiment extends CommandBase {

  // TODO delete this class


  private double critGain = 0.2;  // multiply this for faster speeds if needed. (it will ocillate more when tipped)
  private double period = 0.96;
  private double kp = 0.6*critGain;
  private double ki = (2*kp/period)*0; // this is the formula multiplied by zero (to keep the formula intact)
  private double kd = 0.125*kp*period*0;
  private Drivetrain _drivetrain;           //outputs a number of distance          outputs how fast you're moving away
//                                            Probably must be below 1.  We didn't seem to need ki.  If kd is too high, it oscillates
  private PIDController _PIDController = new PIDController(0.01, 0, 0);//.5, 0, 0
  // motor output = kp x error    motor output = ki x errorSum
  double distance;
  double turnAngle;



  /** Creates a new AutonomousExperiment. */
  public AutonomousExperiment(Drivetrain drivetrain, double _distance, double _turnAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    distance = _distance;
    turnAngle = _turnAngle;
    _drivetrain = drivetrain;
    addRequirements(drivetrain);
    _PIDController.setSetpoint(distance);

  }




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _drivetrain.resetHeading();
    _drivetrain.resetEncoders();
    _PIDController.setTolerance(2,5);
    _PIDController.setIntegratorRange(-.5, .5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double turning = _PIDController.calculate(.5*(_drivetrain.getDistanceL()+_drivetrain.getDistanceR()),distance);
    Double forward = MathUtil.clamp(_PIDController.calculate(.5*(_drivetrain.getDistanceL()+_drivetrain.getDistanceR()), distance),-.3,.3);

    Double SDistance = _drivetrain.getDistanceL();
    
    // Clamp is used to lower the max speed
    _drivetrain.autoDrive(MathUtil.clamp(_PIDController.calculate(.5*(_drivetrain.getDistanceL()+_drivetrain.getDistanceR()), distance),-.5,.5),
    _PIDController.calculate(_drivetrain.getHeading(), turnAngle));

    _drivetrain.getSignal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.autoDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _PIDController.atSetpoint();
  }
}

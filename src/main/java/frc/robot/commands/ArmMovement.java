// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class ArmMovement extends CommandBase {

  private Arm arm;
  private CommandXboxController controller;

  /** Creates a new ArmMovement. */
  public ArmMovement(CommandXboxController _controller, Arm _arm) {
    arm = _arm;
    controller = _controller;
    addRequirements(_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.resetAngleEncoders();
    arm.resetTeleEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ///////////// rotate the arm /////////////
    if(controller.getRightY()>=0.001){
    arm.moveAngleMotor((-controller.getRightY()/3)+(arm.setArmStability(arm.getAbsoluteAngle(), arm.getTeleEncoderDistance())));
    }else if(controller.getRightY()<=-0.001){
      arm.moveAngleMotor((-controller.getRightY()/3)+(arm.setArmStability(arm.getAbsoluteAngle(), arm.getTeleEncoderDistance())));
    } else {
      arm.moveAngleMotor(arm.setArmStability(arm.getAbsoluteAngle(), arm.getTeleDistance()));
    }
    // Added the setArmStability command to the move angle, and set the division to 5 Joseph B.

    ///////////// telescoping the arm /////////////
    if(controller.getRightTriggerAxis()>0.01){
      arm.moveTeleMotor(controller.getRightTriggerAxis());
    }else if(controller.getLeftTriggerAxis()>0.01){
      arm.moveTeleMotor(-(controller.getLeftTriggerAxis()));
    }else{
      arm.moveTeleMotor(0);
    }

    if(arm.getBeamBreakSensor() == true){
      arm.resetShaftEncoders();   // reset the encoder when the beam break sensor is true (when it is fully retracted)
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

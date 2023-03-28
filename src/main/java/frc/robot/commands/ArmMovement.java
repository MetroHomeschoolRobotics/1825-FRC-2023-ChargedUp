// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;

public class ArmMovement extends CommandBase {

  private Arm arm;
  private CommandXboxController controller;
  private double value;
  public PIDController armPIDController;

  /** Creates a new ArmMovement. */
  public ArmMovement(CommandXboxController _controller, Arm _arm, double encoderValue) {
    arm = _arm;
    value = encoderValue;
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

    Trigger BeamBreakDetector = new Trigger(() -> !arm.getBeamBreakSensor());

    // rotate the arm
    arm.moveAngleMotor(controller.getRightY()+(arm.setArmStability(arm.getAbsoluteAngle(), arm.getTeleEncoderDistance()))*3);
    SmartDashboard.putNumber("Arm joystick position", controller.getRightY());
    SmartDashboard.putNumber("Total value sent to arm", (controller.getRightY()+(arm.setArmStability(arm.getAbsoluteAngle(), arm.getTeleEncoderDistance()))*3)/3);
    // Added the setArmStability command to the move angle, and set the division to 5 Joseph B.

    // telescoping the arm 
    //Great job on the comments :) A+ 
    if(controller.getRightTriggerAxis()>0.01){
      arm.moveTeleMotor(controller.getRightTriggerAxis());
      //TODO I need to move this to RobotContainer so the BeamBreak can sense this
    }else if(controller.getLeftTriggerAxis()>0.01){
      arm.moveTeleMotor(-(controller.getLeftTriggerAxis()));
    }else{
      arm.moveTeleMotor(0);
    }

    // System.out.println(arm.setArmStability(arm.getAbsoluteAngle(), arm.getTeleEncoderDistance()));
    // Commented out the print statement, Joseph B

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

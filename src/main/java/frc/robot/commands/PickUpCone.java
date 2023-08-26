// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Arm;

public class PickUpCone extends CommandBase {

  private final Pneumatics grabberPneumatic;
  private final Arm arm;

  public enum State {
    Initial,
    Angle,
    Done;
  }
  private State state;

  /** Creates a new Grabber. */
  public PickUpCone(Arm armParam, Pneumatics pneumatic) {
    // Use addRequirements() here to declare subsystem dependencies.
    grabberPneumatic = pneumatic;
    arm = armParam;
    addRequirements(pneumatic);
    addRequirements(armParam);
    state = State.Initial;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //grabberPneumatic.setGrabber();
   grabberPneumatic.setGrabberOpen();
   grabberPneumatic.changeGrabberState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
        case Initial:
        state = State.Angle;
        break;
        case Angle:
        if(arm.moveToTarget(115, 0)) {
            state = State.Done;
        }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (state == State.Done);
  }
}

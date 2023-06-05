// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.TimeofFlight;



public class AutoGrabber extends CommandBase {
  /** Creates a new AutoGrabber. */
  private final Pneumatics grabberPneumatic;
  private final TimeofFlight grabberSensor;

  public AutoGrabber(Pneumatics pneumatic, TimeofFlight _grabberSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    grabberPneumatic = pneumatic;
    grabberSensor = _grabberSensor;
    addRequirements(pneumatic);
    addRequirements(_grabberSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(grabberPneumatic.getGrabberState() == DoubleSolenoid.Value.kForward && grabberSensor.getSensorDistanceMM() <= 255.9 && grabberSensor.getSensorDistanceMM() >= 165.1 && grabberSensor.isMeasurementSuccessful()){
      grabberPneumatic.setGrabberClose();
    } else{
      // grabberPneumatic.setGrabberOpen(); TODO might need to delete this
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

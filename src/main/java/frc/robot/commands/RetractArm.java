package frc.robot.commands;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.DigitalInput;

public class RetractArm extends CommandBase{
    private Arm arm;
    private CommandXboxController controller;
    public PIDController armPIDController;
    private double speed;
  /**
   *
   * @param subsystem The subsystem used by this command.
   */
    public RetractArm(CommandXboxController _controller, Arm _arm, double _speed) {
        arm = _arm;
        speed = _speed;
        controller = _controller;
        addRequirements(_arm);    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.resetTeleEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.moveTeleMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


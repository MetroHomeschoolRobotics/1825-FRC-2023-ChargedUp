package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.*;
public class Vision extends SubsystemBase{
    public Vision() {}
    public PhotonCamera aprilTagCam = new PhotonCamera("limelight");

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("aprilTagHasTargets", aprilTagCam.getLatestResult().hasTargets());
        
    }
}
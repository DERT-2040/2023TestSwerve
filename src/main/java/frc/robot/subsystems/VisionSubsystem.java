package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;
    public VisionSubsystem() {
        camera = new PhotonCamera("Camera1");
    }

    public Pose2d getPose() {
        var result = camera.getLatestResult();
        List<PhotonTrackedTarget> target = result.getTargets();
        if(target == null || target.size() == 0) {
            return new Pose2d(0, 0, new Rotation2d(0));
        }
     //   Pose2d robotPose = ??????;   FIX
      //  return robotPose;             FIX
      return new Pose2d(0, 0, new Rotation2d(0)); // temp to remove errors
    }
}

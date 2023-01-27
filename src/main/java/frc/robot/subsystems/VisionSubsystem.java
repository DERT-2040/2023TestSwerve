package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;
    AprilTagFieldLayout atfl;
    PhotonPoseEstimator photonPoseEstimator;

    public VisionSubsystem() {
        final AprilTag tag2 = new AprilTag(2, new Pose3d(new Pose2d(10, 5, new Rotation2d(0))));

        ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
        atList.add(tag2);
        atfl = new AprilTagFieldLayout(atList, 54, 27);

        camera = new PhotonCamera("OV5647");
        photonPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, new Transform3d(new Pose3d(new Pose2d(0, 0, new Rotation2d(0))), new Pose3d(new Pose2d(0, 0, new Rotation2d(0)))));

        

    }

    //public Pose2d getEstimatedGlobalPose(Pose2d prevEstimateddRobotPose) {
        //photonPoseEstimator.setReferencePose(prevEstimateddRobotPose);
        //return photonPoseEstimator.update();

    //}


   

    public Pose2d getPose() {
        var result = camera.getLatestResult();
        List<PhotonTrackedTarget> target = result.getTargets();
        if(target == null || target.size() == 0) {
            return new Pose2d(0, 0, new Rotation2d(0));
        }
        SmartDashboard.putNumber("Target", target.get(0).getPitch());
    
        

 //Alternate way to get data
        /*if (result.hasTargets()) {
            var imageCaptureTime = result.getTimestampSeconds();
            var camToTargetTrans = result.getBestTarget().getBestCameraToTarget();
            var camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
            photonPoseEstimator.addVisionMeasurement(
                    camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);
        }  */

        //April tag locationo
        Pose2d targetPose = new Pose2d(3.67, 0.55, new Rotation2d(Units.degreesToRadians(180)));


        //Camera to target
        Transform3d trans = result.getBestTarget().getBestCameraToTarget();
        SmartDashboard.putString("ResultTransform", trans.toString());
        //SmartDashboard.putNumber("x", trans.getRotation().getX());
        //SmartDashboard.putNumber("y", trans.getRotation().getY());
        //SmartDashboard.putNumber("z", trans.getRotation().getZ());
        

        Transform2d camToTarget2d = new Transform2d(new Translation2d(trans.getX(), trans.getY()), new Rotation2d(trans.getRotation().getZ()));
        SmartDashboard.putString("2d Transform", camToTarget2d.toString());


        Pose2d finalPose = targetPose.plus(camToTarget2d);//PhotonUtils.estimateFieldToCamera(camToTarget2d, targetPose);
        
    
        /*Pose3d pose = photonPoseEstimator.update().get().estimatedPose;
    
        SmartDashboard.putString("Vision Pose", pose.toString());

        double myPoseX = pose.getX();
        double myPoseY = pose.getY();
        double myPoseZ = pose.getZ();
        SmartDashboard.putNumber("myPose X", myPoseX);
        SmartDashboard.putNumber("myPose Y", myPoseY);
        SmartDashboard.putNumber("myPose Z", myPoseZ);

     //   Pose2d robotPose = ??????;   FIX
      //  return robotPose;             FIX
      return pose; // temp to remove errors*/
      return finalPose;
    }
}

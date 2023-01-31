package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
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
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;
    ArrayList<AprilTag> atList;
    AprilTagFieldLayout atfl;
    PhotonPoseEstimator photonPoseEstimator;

    public VisionSubsystem() {
        final AprilTag tag1 = new AprilTag(1, new Pose3d(15.513558, 1.071626, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 1))));
        final AprilTag tag2 = new AprilTag(2, new Pose3d(15.513558, 2.748026, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 1))));
        final AprilTag tag3 = new AprilTag(3, new Pose3d(15.513558, 4.424426, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 1))));
        final AprilTag tag4 = new AprilTag(4, new Pose3d(16.513558, 6.749796, 0.695452, new Rotation3d(new Quaternion(0, 0, 0, 1))));
        final AprilTag tag5 = new AprilTag(5, new Pose3d(0.36195, 6.749796, 0.695452, new Rotation3d(new Quaternion(1, 0, 0, 0))));
        final AprilTag tag6 = new AprilTag(6, new Pose3d(1.02743, 4.424426, 0.462788, new Rotation3d(new Quaternion(1, 0, 0, 0))));
        final AprilTag tag7 = new AprilTag(7, new Pose3d(1.02743, 2.748026, 0.462788, new Rotation3d(new Quaternion(1, 0, 0, 0))));
        final AprilTag tag8 = new AprilTag(8, new Pose3d(1.02743, 1.071626, 0.462788, new Rotation3d(new Quaternion(1, 0, 0, 0))));


        atList = new ArrayList<AprilTag>();
        atList.add(tag1);
        atList.add(tag2);
        atList.add(tag3);
        atList.add(tag4);
        atList.add(tag5);
        atList.add(tag6);
        atList.add(tag7);
        atList.add(tag8);

        /*try {
            atfl = AprilTagFieldLayout.loadFromResource("k2023ChargedUp.json");
        } catch(Exception e) {

        }*/

        


        atfl = new AprilTagFieldLayout(atList, 16.54175, 8.0137);
        
        SmartDashboard.putString("field layout", atfl.toString());

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


        int id = target.get(0).getFiducialId();
        SmartDashboard.putNumber("Target ID", id);



        //April tag locationo
        //Pose2d targetPose = new Pose2d(3.67, 0.55, new Rotation2d(Units.degreesToRadians(180)));
        Pose3d targetPose3d = atList.get(id-1).pose;
        Pose2d targetPose = new Pose2d(new Translation2d(targetPose3d.getX(), targetPose3d.getY()), new Rotation2d(targetPose3d.getRotation().getZ()));

        //Camera to target
        Transform3d trans = result.getBestTarget().getBestCameraToTarget();
        SmartDashboard.putString("ResultTransform", trans.toString());
        //SmartDashboard.putNumber("x", trans.getRotation().getX());
        //SmartDashboard.putNumber("y", trans.getRotation().getY());
        //SmartDashboard.putNumber("z", trans.getRotation().getZ());
        

        Transform2d camToTarget2d = new Transform2d(new Translation2d(trans.getX(), trans.getY()), new Rotation2d(trans.getRotation().getZ()));
        SmartDashboard.putString("2d Transform", camToTarget2d.toString());
        


        Pose2d finalPose = targetPose.plus(camToTarget2d);//PhotonUtils.estimateFieldToCamera(camToTarget2d, targetPose);
        
    
        //Pose3d pose = photonPoseEstimator.update().get().estimatedPose;
        //SmartDashboard.putString("Estimated Pose", pose.toString());
    
        /*SmartDashboard.putString("Vision Pose", pose.toString());

        double myPoseX = pose.getX();
        double myPoseY = pose.getY();
        double myPoseZ = pose.getZ();
        SmartDashboard.putNumber("myPose X", myPoseX);
        SmartDashboard.putNumber("myPose Y", myPoseY);
        SmartDashboard.putNumber("myPose Z", myPoseZ);

     //   Pose2d robotPose = ??????;   FIX
      //  return robotPose;             FIX
      return pose; // temp to remove errors*/
      SmartDashboard.putString("Field Position", finalPose.toString());
      return finalPose; //new Pose2d(new Translation2d(trans.getX(), trans.getY()), new Rotation2d(trans.getRotation().getZ()));
    }
}

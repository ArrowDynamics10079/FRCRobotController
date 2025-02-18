package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class LimelightSubsystem extends SubsystemBase {
    // Limelight Data table
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // Basic targeting data

    // The ID of the targetted AprilTag
    double tid = LimelightHelpers.getFiducialID("limelight");
    // Horizontal offset from crosshair to target in degrees
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    // Vertical offset from crosshair to target in degrees
    NetworkTableEntry ty = limelightTable.getEntry("ty");
    // Target area (0% to 100% of image)
    NetworkTableEntry ta = limelightTable.getEntry("ta");

    // A list of the robot's position relative to the april tag [tx, ty, tz, Pitch, Yaw, Roll]
    NetworkTableEntry botPose = limelightTable.getEntry("botpose_targetspace");
    // A  list of the robot's position relative to the field from the Blue alliance
    // [X, Y, Z, Roll, Pitch, Yaw, Latency, Tag Count, Tag Span, Avg Tag Distance, and Avg Tag Area]
    NetworkTableEntry botPoseFieldBlue = limelightTable.getEntry("botpose_wpiblue");
    // The current pipeline index
    NetworkTableEntry activePipeline = limelightTable.getEntry("getpipe");
    // Allows pipeline to be set
    NetworkTableEntry pipelineToSet = limelightTable.getEntry("pipeline");

    // Do you have a valid target?
    boolean hasTarget = LimelightHelpers.getTV("limelight");
    // Horizontal offset from principal pixel to target in degrees
    NetworkTableEntry txnc = limelightTable.getEntry("txnc");
    // Vertical offset from principal pixel to target in degrees
    NetworkTableEntry tync = limelightTable.getEntry("tync");

    // Indicates if limelight is being used
    private final boolean kUseLimelight = false;
    // RobotContainer
    private final RobotContainer m_robotContainer;

    // CONSTRUCTOR
    public LimelightSubsystem(RobotContainer m_robotContainer) {
        // Switch to pipeline 0
        LimelightHelpers.setPipelineIndex("limelight", 0);
        // Sets LED settings
        LimelightHelpers.setLEDMode_PipelineControl("limelight");
        LimelightHelpers.setLEDMode_ForceOn("limelight");
        this.m_robotContainer = m_robotContainer;
    }

    @Override
    public void periodic() {
        // If Limelight is in use
        if (kUseLimelight) {
            // Returns the robot's current state(position, orientation, and velocity)
            var driveState = m_robotContainer.drivetrain.getState();
            // Gets the heading/rotation of the robot in degrees
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            // Converts the robot's angular velocity from radians to rotations per second
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
    
            LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
                // Adds the limelight pose estimate to the drivetrain's odometry calculation
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
            }

            // The parameter inside getDouble or getDoubleArray is what is returned if nothing is found
            SmartDashboard.putNumber("limelight tx", tx.getDouble(0));
            SmartDashboard.putNumber("limelight yaw", getYaw());
            SmartDashboard.putNumber("limelight ta", ta.getDouble(0));
        }
    }
    
    // Returns the bot pose list
    public double[] getBotPose(){
        return botPose.getDoubleArray(new double[6]);
    }

    // Returns the Yaw of the robot from the april tag
    public double getYaw() {
        return getBotPose()[4];
    }

    public double getTx() {
        return tx.getDouble(0);
    }

    public double getTa() {
        return ta.getDouble(0);
    }

    public int getTid() {
        return (int) tid.getDouble(0);
    }

    public double getTagCount() {
        return poseArray[7];
    }

    public Pose2d getPose() {
        Rotation2d rot = new Rotation2d(poseArray[5] * Math.PI / 180);
        Pose2d out = new Pose2d(poseArray[0], poseArray[1], rot);
        return out;
    }

    public int getActivePipeline() {
        return (int)activePipeline.getDouble(2);
    }

    
    
}
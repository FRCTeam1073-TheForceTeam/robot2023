package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Localizer extends SubsystemBase
{
    private DriveSubsystem driveTrain;
    private SwerveDrivePoseEstimator estimator;
    private MapInterface map;
    private AprilTagFinder finder;
    private double lastUpdateTime;

    public Localizer(DriveSubsystem driveTrain, MapInterface map, AprilTagFinder finder)
    {
        this.driveTrain = driveTrain;
        this.map = map;
        this.finder = finder;
        estimator = new SwerveDrivePoseEstimator(
            driveTrain.getKinematics(), driveTrain.getGyroAngle(), driveTrain.getModulePositions(), new Pose2d()
        );
        lastUpdateTime = Timer.getFPGATimestamp();
    }
    
    @Override
    public void periodic()
    {
        double now = Timer.getFPGATimestamp();
        estimator.updateWithTime(now, driveTrain.getGyroAngle(), driveTrain.getModulePositions());
        /*
         * if (have a new sensor measurement && it is valid)
         * {
         *  transform sensor measurement to a measurement of where the robot is on the map
         *  apply update to estimator
         * }
         */
        // only run sensor update if we've moved enough and a few seconds have passed
        if (now - lastUpdateTime > 1.0)
        {
            ArrayList<AprilTag> tags = finder.getTags();
            for (int i = 0; i < tags.size(); i++)
            {
                /*
                * if (tag is "good")
                * {
                *      get the pose3d of the tag relative to the robot
                *      find the landmark in the map
                *      if (we get the landmark)
                *      {
                *          apply this as an inverse transform to find the robot's position
                *      }
                * }
                */
                Pose3d landMarkPose = map.getLandmark(tags.get(i).ID);
                if (landMarkPose != null)
                {
                    Transform3d transform = new Transform3d(new Pose3d(), tags.get(i).pose);
                    Pose3d measurement = landMarkPose.transformBy(null);
                    Pose2d measurement2d = new Pose2d(
                        new Translation2d(measurement.getX(), measurement.getY()), 
                        new Rotation2d(measurement.getRotation().getAngle())
                    );
                    estimator.addVisionMeasurement(measurement2d, now);
                }
            }
            lastUpdateTime = now;
        }
        
    }

    public Pose2d getOdometry()
    {
        // TODO: actually implement this
        return new Pose2d();
    }

}

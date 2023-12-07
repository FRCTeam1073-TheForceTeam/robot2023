package frc.robot.subsystems;

import java.util.HashMap;
import edu.wpi.first.math.geometry.Pose3d;

public class FieldMap implements MapInterface
{
    private HashMap<Integer, Pose3d> apriltagMap;
    private HashMap<Integer, Pose3d> landmarkMap;

    public FieldMap()
    {
        apriltagMap = new HashMap<>();
        // populate apriltag map
        apriltagMap.put(1, new Pose3d());
        apriltagMap.put(2, new Pose3d());

        landmarkMap = new HashMap<>();
        //populate landmark map
        landmarkMap.put(1, new Pose3d());
        landmarkMap.put(2, new Pose3d());
    }
    
    @Override
    public Pose3d getLandmark(int id)
    {
        if (landmarkMap.containsKey(id))
        {
            return landmarkMap.get(id); 
        }
        return null;
    }

    public Pose3d getApriltagLandmark(int id){
        if (apriltagMap.containsKey(id))
        {
            return apriltagMap.get(id); 
        }
        return null;
    }
}

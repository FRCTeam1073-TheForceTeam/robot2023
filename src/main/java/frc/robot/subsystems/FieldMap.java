package frc.robot.subsystems;

import java.util.HashMap;
import edu.wpi.first.math.geometry.Pose3d;

public class FieldMap implements MapInterface
{
    private HashMap<Integer, Pose3d> map;

    public FieldMap()
    {
        map = new HashMap<>();
        // populate map
        map.put(1, new Pose3d());
        map.put(2, new Pose3d());
    }
    
    @Override
    public Pose3d getLandmark(int id)
    {
        if (map.containsKey(id))
        {
            return map.get(id); 
        }
        return null;
    }
}

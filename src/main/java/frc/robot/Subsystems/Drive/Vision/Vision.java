package frc.robot.Subsystems.Drive.Vision;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import frc.robot.Subsystems.Drive.Vision.CameraPhoton.VisionReadings;

public class Vision {
    private final ArrayList<CameraPhoton> kCameras = new ArrayList<>();

    public Vision(CameraPhoton... cameras) {
        for(var camera : cameras) {
            kCameras.add(camera);
        }
    }

    public void updateVisionPoses() {
        for(var camera : kCameras) {
            camera.updateVisionReadings();
            camera.updateInputs(camera.kInputs);
            Logger.processInputs(camera.kName, camera.kInputs);
        }
    }

    public VisionReadings getNewPoses() {
        VisionReadings readings = new VisionReadings();
        for(var camera : kCameras) {
            readings.combineReadings(camera.getVisionReadings());
        }
        return readings;
    }
}

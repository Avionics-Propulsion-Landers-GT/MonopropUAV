import java.io.*;
import java.util.Random;

/**
 * Emulates an IMU, returning data for the test benches.
 *
 * @author Anyi Lin
 * @version 1.0
 */
public class IMU {
    // Scales the inputs from each of the axes of the sensors
    private static double gyroXScaling = 1;
    private static double gyroYScaling = 1;
    private static double gyroZScaling = 1;
    private static double accelXScaling = 1;
    private static double accelYScaling = 1;
    private static double accelZScaling = 1;
    private static double magXScaling = 1;
    private static double magYScaling = 1;
    private static double magZScaling = 1;

    // Standard deviations of normally distributed error on each axis of each of the sensors
    private static double gyroXErrorSD = 1;
    private static double gyroYErrorSD = 1;
    private static double gyroZErrorSD = 1;
    private static double accelXErrorSD = 1;
    private static double accelYErrorSD = 1;
    private static double accelZErrorSD = 1;
    private static double magXErrorSD = 1;
    private static double magYErrorSD = 1;
    private static double magZErrorSD = 1;

    // Adds an offset to the sensors (will result in gyro drifting due to integration)
    private static double gyroXOffset = 1;
    private static double gyroYOffset = 1;
    private static double gyroZOffset = 1;
    private static double accelXOffset = 1;
    private static double accelYOffset = 1;
    private static double accelZOffset = 1;
    private static double magXOffset = 1;
    private static double magYOffset = 1;
    private static double magZOffset = 1;

    public static void main(String[] args) throws IOException {
        String line = "";
        String delimiter = ",";
        BufferedReader bufferedReader = new BufferedReader(new FileReader("fileName.csv"));
        File outputFile = new File("IMUData.csv");
        PrintWriter out = new PrintWriter(outputFile);
        Random random = new Random();

        while ((line = bufferedReader.readLine()) != null) {
            // Gets the simulated data from the CSV file
            String[] inputData = line.split(delimiter);
            IMUData imuData = new IMUData(Double.valueOf(inputData[0]), Double.valueOf(inputData[1]),
                    Double.valueOf(inputData[2]), Double.valueOf(inputData[3]), Double.valueOf(inputData[4]),
                    Double.valueOf(inputData[5]), Double.valueOf(inputData[6]), Double.valueOf(inputData[7]),
                    Double.valueOf(inputData[8]), Double.valueOf(inputData[9]));

            // Scales inputs from all sensors
            imuData.setGyroX(imuData.getGyroX() * gyroXScaling);
            imuData.setGyroY(imuData.getGyroY() * gyroYScaling);
            imuData.setGyroZ(imuData.getGyroZ() * gyroZScaling);
            imuData.setAccelX(imuData.getAccelX() * accelXScaling);
            imuData.setAccelY(imuData.getAccelY() * accelYScaling);
            imuData.setAccelZ(imuData.getAccelZ() * accelZScaling);
            imuData.setMagX(imuData.getMagX() * magXScaling);
            imuData.setMagY(imuData.getMagY() * magYScaling);
            imuData.setMagZ(imuData.getMagZ() * magZScaling);

            // Adds normally distributed noise to all sensors
            imuData.setGyroX(imuData.getGyroX() + random.nextGaussian() * gyroXErrorSD);
            imuData.setGyroY(imuData.getGyroY() + random.nextGaussian() * gyroYErrorSD);
            imuData.setGyroZ(imuData.getGyroZ() + random.nextGaussian() * gyroZErrorSD);
            imuData.setAccelX(imuData.getAccelX() + random.nextGaussian() * accelXErrorSD);
            imuData.setAccelY(imuData.getAccelY() + random.nextGaussian() * accelYErrorSD);
            imuData.setAccelZ(imuData.getAccelZ() + random.nextGaussian() * accelZErrorSD);
            imuData.setMagX(imuData.getMagX() + random.nextGaussian() * magXErrorSD);
            imuData.setMagY(imuData.getMagY() + random.nextGaussian() * magYErrorSD);
            imuData.setMagZ(imuData.getMagZ() + random.nextGaussian() * magZErrorSD);

            // Adds offset to all sensors
            imuData.setGyroX(imuData.getGyroX() + gyroXOffset);
            imuData.setGyroY(imuData.getGyroY() + gyroYOffset);
            imuData.setGyroZ(imuData.getGyroZ() + gyroZOffset);
            imuData.setAccelX(imuData.getAccelX() + accelXOffset);
            imuData.setAccelY(imuData.getAccelY() + accelYOffset);
            imuData.setAccelZ(imuData.getAccelZ() + accelZOffset);
            imuData.setMagX(imuData.getMagX() + magXOffset);
            imuData.setMagY(imuData.getMagY() + magYOffset);
            imuData.setMagZ(imuData.getMagZ() + magZOffset);

            out.printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", imuData.getTimestamp(), imuData.getAccelX(),
                    imuData.getAccelY(), imuData.getAccelZ(), imuData.getGyroX(), imuData.getGyroY(),
                    imuData.getGyroZ(), imuData.getGyroX(), imuData.getGyroY(), imuData.getGyroZ());
        }
        bufferedReader.close();
        out.close();
    }
}

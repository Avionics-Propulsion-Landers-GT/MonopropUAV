import java.io.*;
import java.math.BigDecimal;
import java.util.Random;

/**
 * Emulates an IMU, returning data for the test benches.
 *
 * @author Anyi Lin
 * @version 1.0
 */
public class IMU {
    // Scales the inputs from each of the axes of the sensors
    private static double gyroXScaling = 1.05;
    private static double gyroYScaling = 1.03;
    private static double gyroZScaling = 1.01;
    private static double accelXScaling = 1.04;
    private static double accelYScaling = 1.02;
    private static double accelZScaling = 1.03;
    private static double magXScaling = 1.04;
    private static double magYScaling = 1.02;
    private static double magZScaling = 1.03;

    // Standard deviations of normally distributed error on each axis of each of the sensors
    private static double gyroXErrorNoiseScaling = 0.07;
    private static double gyroYErrorNoiseScaling = 0.07;
    private static double gyroZErrorNoiseScaling = 0.07;
    private static double accelXErrorNoiseScaling = 0.15;
    private static double accelYErrorNoiseScaling = 0.15;
    private static double accelZErrorNoiseScaling = 0.15;
    private static double magXErrorNoiseScaling = 0.0000001;
    private static double magYErrorNoiseScaling = 0.0000001;
    private static double magZErrorNoiseScaling = 0.0000001;

    // Adds an offset to the sensors (will result in gyro drifting due to integration)
    private static double gyroXOffset = 0.005;
    private static double gyroYOffset = 0.005;
    private static double gyroZOffset = 0.005;
    private static double accelXOffset = 0.0005;
    private static double accelYOffset = 0.0002;
    private static double accelZOffset = 0.0003;
    private static double magXOffset = 0.0000002;
    private static double magYOffset = 0.0000002;
    private static double magZOffset = 0.0000002;

    public static void main(String[] args) throws IOException {
        String line = "";
        String delimiter = ",";
        BufferedReader bufferedReader = new BufferedReader(new FileReader("monocopter_data.csv"));
        File outputFile = new File("noisy_monocopter_data.csv");
        PrintWriter out = new PrintWriter(outputFile);
        Random random = new Random();
        out.println("time,x ang vel,y ang vel,z ang vel,x accel,y accel,z accel,x mag,y mag,z mag");
        line = bufferedReader.readLine();

        while ((line = bufferedReader.readLine()) != null) {
            out.print("\n");
            // Gets the simulated data from the CSV file
            String[] inputData = line.split(delimiter);
            IMUData imuData = new IMUData(new BigDecimal(inputData[0]).doubleValue(), new BigDecimal(inputData[1]).doubleValue(),
                    new BigDecimal(inputData[2]).doubleValue(), new BigDecimal(inputData[3]).doubleValue(), new BigDecimal(inputData[4]).doubleValue(),
                    new BigDecimal(inputData[5]).doubleValue(), new BigDecimal(inputData[6]).doubleValue(), new BigDecimal(inputData[7]).doubleValue(),
                    new BigDecimal(inputData[8]).doubleValue(), new BigDecimal(inputData[9]).doubleValue());

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
            imuData.setGyroX(imuData.getGyroX() + random.nextGaussian() * gyroXErrorNoiseScaling);
            imuData.setGyroY(imuData.getGyroY() + random.nextGaussian() * gyroYErrorNoiseScaling);
            imuData.setGyroZ(imuData.getGyroZ() + random.nextGaussian() * gyroZErrorNoiseScaling);
            imuData.setAccelX(imuData.getAccelX() + random.nextGaussian() * accelXErrorNoiseScaling);
            imuData.setAccelY(imuData.getAccelY() + random.nextGaussian() * accelYErrorNoiseScaling);
            imuData.setAccelZ(imuData.getAccelZ() + random.nextGaussian() * accelZErrorNoiseScaling);
            imuData.setMagX(imuData.getMagX() + random.nextGaussian() * magXErrorNoiseScaling);
            imuData.setMagY(imuData.getMagY() + random.nextGaussian() * magYErrorNoiseScaling);
            imuData.setMagZ(imuData.getMagZ() + random.nextGaussian() * magZErrorNoiseScaling);

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


            out.printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f", imuData.getTimestamp(), imuData.getAccelX(),
                    imuData.getAccelY(), imuData.getAccelZ(), imuData.getGyroX(), imuData.getGyroY(),
                    imuData.getGyroZ(), imuData.getMagX(), imuData.getMagY(), imuData.getMagZ());
            
        }
        bufferedReader.close();
        out.close();
    }
}

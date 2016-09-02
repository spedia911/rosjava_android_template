/*
 * Copyright (C) 2014 Oliver Degener.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ollide.rosandroid;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorListener;
import android.hardware.SensorManager;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import java.text.SimpleDateFormat;
import java.util.Date;

import sensor_msgs.Imu;
import sensor_msgs.MagneticField;

public class SimplePublisherNode extends AbstractNodeMain implements NodeMain {

    private static final String TAG = SimplePublisherNode.class.getSimpleName();

    private SensorEventListener sensorEventListener;
    private SensorManager sensorManager;
    private Publisher<Imu> publisher;
    private Publisher<MagneticField> publisher_mag;

    private class ImuThread extends Thread {
        private final SensorManager sensorManager;
        private SensorEventListener sensorListener;
        private Looper threadLooper;

        private final Sensor accelSensor;
        private final Sensor gyroSensor;
        private final Sensor quatSensor;
        private final Sensor magSensor;

        private ImuThread(SensorManager sensorManager, SensorEventListener sensorListener) {
            this.sensorManager = sensorManager;
            this.sensorListener = sensorListener;
            this.accelSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            this.gyroSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
            this.quatSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
            this.magSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED);
        }


        public void run() {
            Looper.prepare();
            this.threadLooper = Looper.myLooper();
            this.sensorManager.registerListener(this.sensorListener, this.accelSensor, SensorManager.SENSOR_DELAY_FASTEST);
            this.sensorManager.registerListener(this.sensorListener, this.gyroSensor, SensorManager.SENSOR_DELAY_FASTEST);
            this.sensorManager.registerListener(this.sensorListener, this.quatSensor, SensorManager.SENSOR_DELAY_FASTEST);
            this.sensorManager.registerListener(this.sensorListener, this.magSensor, SensorManager.SENSOR_DELAY_FASTEST);
            Looper.loop();
        }


        public void shutdown() {
            this.sensorManager.unregisterListener(this.sensorListener);
            if (this.threadLooper != null) {
                this.threadLooper.quit();
            }
        }
    }

    private class SensorListener implements SensorEventListener {

        private Publisher<Imu> publisher;
        private Publisher<MagneticField> publisher_mag;

        private boolean hasAccel;
        private boolean hasGyro;
        private boolean hasQuat;
        private boolean hasMag;

        private long accelTime;
        private long gyroTime;
        private long quatTime;
        private long magTime;

        private Imu imu;
        private MagneticField mag;

        private SensorListener(Publisher<Imu> publisher, Publisher<MagneticField> publisher_mag, boolean hasAccel, boolean hasGyro, boolean hasQuat, boolean hasMag) {
            this.publisher = publisher;
            this.publisher_mag = publisher_mag;
            this.hasAccel = hasAccel;
            this.hasGyro = hasGyro;
            this.hasQuat = hasQuat;
            this.hasMag = hasMag;
            this.accelTime = 0;
            this.gyroTime = 0;
            this.quatTime = 0;
            this.magTime = 0;
            this.imu = this.publisher.newMessage();
            this.mag = this.publisher_mag.newMessage();
        }

        //	@Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }

        //	@Override
        public void onSensorChanged(SensorEvent event) {
            if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                this.imu.getLinearAcceleration().setX(event.values[0]);
                this.imu.getLinearAcceleration().setY(event.values[1]);
                this.imu.getLinearAcceleration().setZ(event.values[2]);

                double[] tmpCov = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};// TODO Make Parameter
                this.imu.setLinearAccelerationCovariance(tmpCov);
                this.accelTime = event.timestamp;
            } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                this.imu.getAngularVelocity().setX(event.values[0]);
                this.imu.getAngularVelocity().setY(event.values[1]);
                this.imu.getAngularVelocity().setZ(event.values[2]);
                double[] tmpCov = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};// TODO Make Parameter
                this.imu.setAngularVelocityCovariance(tmpCov);
                this.gyroTime = event.timestamp;
            } else if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
                float[] quaternion = new float[4];
                SensorManager.getQuaternionFromVector(quaternion, event.values);
                this.imu.getOrientation().setW(quaternion[0]);
                this.imu.getOrientation().setX(quaternion[1]);
                this.imu.getOrientation().setY(quaternion[2]);
                this.imu.getOrientation().setZ(quaternion[3]);
                double[] tmpCov = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};// TODO Make Parameter
                this.imu.setOrientationCovariance(tmpCov);
                this.quatTime = event.timestamp;
            } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) {
                this.mag.getMagneticField().setX(event.values[0]);
                this.mag.getMagneticField().setY(event.values[1]);
                this.mag.getMagneticField().setZ(event.values[2]);
                double[] tmpCov = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};// TODO Make Parameter
                this.mag.setMagneticFieldCovariance(tmpCov);
                this.magTime = event.timestamp;
            }

            // Currently storing event times in case I filter them in the future.  Otherwise they are used to determine if all sensors have reported.
            if ((this.accelTime != 0 || !this.hasAccel) && (this.gyroTime != 0 || !this.hasGyro) && (this.quatTime != 0 || !this.hasQuat) && (this.magTime != 0 || !this.hasMag)) {
                // Convert event.timestamp (nanoseconds uptime) into system time, use that as the header stamp
                long time_delta_millis = System.currentTimeMillis() - SystemClock.uptimeMillis();
                this.imu.getHeader().setStamp(Time.fromMillis(time_delta_millis + event.timestamp / 1000000));
                this.imu.getHeader().setFrameId("/imu");// TODO Make parameter

                publisher.publish(this.imu);

                // Create a new message
                this.imu = this.publisher.newMessage();

                // Reset times
                this.accelTime = 0;
                this.gyroTime = 0;
                this.quatTime = 0;
                this.magTime = 0;
            }
        }
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("SimplePublisher/TimeLoopNode");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        final Publisher<std_msgs.String> publisher = connectedNode.newPublisher(GraphName.of("time"), std_msgs.String._TYPE);

        final CancellableLoop loop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                // retrieve current system time
                String time = new SimpleDateFormat("HH:mm:ss").format(new Date());

                Log.i(TAG, "publishing the current time: " + time);

                // create and publish a simple string message
                std_msgs.String str = publisher.newMessage();
                str.setData("The current time is: " + time);
                publisher.publish(str);

                // go to sleep for one second
                Thread.sleep(1000);
            }
        };
        connectedNode.executeCancellableLoop(loop);
    }

}

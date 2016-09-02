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

import android.hardware.SensorManager;
import android.os.Bundle;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

public class MainActivity extends RosActivity {

    private SensorManager mSensorManager;
    private SimplePublisherNode simplePublisherNode;
    private MagneticFieldPublisher magnetic_field_pub;

    public MainActivity() {
        super("RosAndroidExample", "RosAndroidExample");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mSensorManager = (SensorManager)this.getSystemService(SENSOR_SERVICE);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        int currentapiVersion = android.os.Build.VERSION.SDK_INT;

        int sensorDelay = 20000; // 20,000 us == 50 Hz for Android 3.1 and above
        if(currentapiVersion <= android.os.Build.VERSION_CODES.HONEYCOMB){
            sensorDelay = SensorManager.SENSOR_DELAY_UI; // 16.7Hz for older devices.  They only support enum values, not the microsecond version.
        }

/*
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeConfiguration.setNodeName("android_sensors_driver_mag");
        this.simplePublisherNode = new SimplePublisherNode();
        nodeMainExecutor.execute(this.simplePublisherNode, nodeConfiguration);
*/

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration1 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration1.setMasterUri(getMasterUri());
            nodeConfiguration1.setNodeName("android_sensors_driver_magnetic_field");
            this.magnetic_field_pub = new MagneticFieldPublisher(mSensorManager, sensorDelay);
            nodeMainExecutor.execute(this.magnetic_field_pub, nodeConfiguration1);
        }
    }
}

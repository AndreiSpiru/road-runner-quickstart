/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

// Acest program este folosit pentru a testa detectia vizuala
public class Webcam
{
    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";
    public static final String VUFORIA_KEY ="AQ5qk8f/////AAABmVHF5MgFj052hxwcMzcrVCUFbGq/csKfD+WVqamlDPJ8eSd+4r5jn4upsuewNdyn3glBVOsM/ZRxWUOUFuGPSTPUMFh12Fv1mamPQgiUsiR2SRpS1PTa9FeqDkkhVbtha5Pwjr8WfK8fFZdldQkD9DgsvythDb/yb5FXXR+5p4V+wEQS7FnZadJKhkZoOKlQiETstzGJH35TfhcIPq6Qzs048U1hD+tij81FAGnvzMFihom+/Xb/kilYNO53zT15wnVAFdHNY+QIGKOzrk2RLdyAC73HoEndsfHeVjQ6XcioUgT1EfM0/YR7Co3zwKxqRR0ES6tQNtKvUTTo9HZLjCJl42fBYFuvCKQhrCIk6126";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public Webcam(HardwareMap hardwareMap)
    {
        initVuforia(hardwareMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        } else {

        }


        if (tfod != null) {
            tfod.activate();
        }

    }

    public int detection()
    {
        int poz=0;
        if (tfod != null) {

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size() == 2) {
                int stone1 = -1;
                int stone2 = -1;


                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT))
                        stone1 = (int) recognition.getLeft();
                    else if (stone2 == -1) stone2 = (int) recognition.getLeft();
                }

                if (stone1 == -1)
                {
                    poz=1;
                }
                else if (stone1 < stone2) {poz=2;}
                else {poz=3;}



            }
        }
        return poz;
    }



    public void tfodshutdown()
    {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
    private void initVuforia(HardwareMap hardwareMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        vuforia = ClassFactory.getInstance().createVuforia(parameters);


    }


    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
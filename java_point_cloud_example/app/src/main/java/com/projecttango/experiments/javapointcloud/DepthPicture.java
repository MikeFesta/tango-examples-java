package com.projecttango.experiments.javapointcloud;

// This code came from http://stackoverflow.com/questions/38216070/generating-depth-map-from-point-cloud on 1/5/2017



import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.widget.ImageView;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.projecttango.examples.java.pointcloud.R;
import com.projecttango.tangosupport.TangoPointCloudManager;
import com.projecttango.tangosupport.TangoSupport;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

public class DepthPicture extends Activity {

    private Tango mTango;
    private TangoConfig mTangoConfig;
    private TangoPointCloudManager mPointCloudManager;
    private AtomicBoolean tConnected = new AtomicBoolean(false);
    Random rand = new Random();
    private ImageView imageDepthMap;


    private static final ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();

    {
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                TangoPoseData.COORDINATE_FRAME_DEVICE));
    }
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        //initialize the imageView
        imageDepthMap = (ImageView)findViewById(R.id.imageView);

        //initialize pointCloudManager

        mPointCloudManager = new TangoPointCloudManager();


    }

    @Override
    protected void onResume(){

        super.onResume();
        //obtain the tango configuration

        if(tConnected.compareAndSet(false, true)) {


            try {

                setTango();

            } catch (TangoOutOfDateException tE) {

                tE.printStackTrace();
            }

        }
    }

    @Override
    protected void onPause(){

        super.onPause();

        if(tConnected.compareAndSet(true, false)) {
            try {
                //disconnect Tango service so other applications can use it
                mTango.disconnect();
            } catch (TangoException e) {
                e.printStackTrace();
            }
        }
    }


    private void setTango(){

        mTango = new Tango(DepthPicture.this, new Runnable() {
            @Override
            public void run() {

                TangoSupport.initialize();
                mTangoConfig = new TangoConfig();
                mTangoConfig = mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT);
                mTangoConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true); //activate depth sensing

                mTango.connect(mTangoConfig);

                mTango.connectListener(framePairs, new Tango.OnTangoUpdateListener() {
                    @Override
                    public void onPoseAvailable(TangoPoseData tangoPoseData) {
                        // required callback, not currently being used
                        Log.d("mfesta", "Pose Available");
                    }

                    @Override
                    public void onXyzIjAvailable(TangoXyzIjData pointCloud) {
                        Log.d("mfesta", "x-y-z available");
                    }


                    @Override
                    public void onPointCloudAvailable(TangoPointCloudData pointCloud) {

                        Log.d("mfesta", "point cloud available");
                        //TangoXyzIjData pointCloud = mPointCloudManager.getLatestXyzIj();
                        // Update current camera pose

                        if (pointCloud.numPoints > 0){
                            try {
                                // Calculate the last camera color pose.
                                TangoPoseData lastFramePose = TangoSupport.getPoseAtTime(0,
                                        TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                        TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR,
                                        TangoSupport.TANGO_SUPPORT_ENGINE_OPENGL, 0);


                                if (pointCloud != null) {

                                    //obtain depth info per pixel
                                    int h = mTango.getCameraIntrinsics(TangoCameraIntrinsics.TANGO_CAMERA_COLOR).height;
                                    int w = mTango.getCameraIntrinsics(TangoCameraIntrinsics.TANGO_CAMERA_COLOR).width;

                                    TangoSupport.DepthBuffer depthBuf = TangoSupport.upsampleImageNearestNeighbor(pointCloud, w, h, lastFramePose);

                                            /*
                                            (
                                            pointCloud, mTango.getCameraIntrinsics(TangoCameraIntrinsics.TANGO_CAMERA_COLOR), lastFramePose
                                    );
                                    */

                                    //create Depth map
                                    int[] intBuff = convertToInt(depthBuf.depths, depthBuf.width, depthBuf.height);

                                    final Bitmap Image = Bitmap.createBitmap(intBuff, depthBuf.width, depthBuf.height, Bitmap.Config.ARGB_8888);

                                    runOnUiThread(new Runnable() {
                                        @Override
                                        public void run() {
                                            imageDepthMap.setImageBitmap(Image);
                                        }
                                    });

                                }
                            } catch (TangoErrorException e) {
                                Log.e("gDebug", "Could not get valid transform");
                            }
                        }
                    }

                    @Override
                    public void onFrameAvailable(int i) {
                        Log.d("mfesta", "Frame Available from " + i);
                    }

                    @Override
                    public void onTangoEvent(TangoEvent tangoEvent) {
                        Log.d("mfesta", "Tango Event");
                    }
                });
            }
        });


    }

    private int[] convertToInt(FloatBuffer pointCloudData, int width, int height){
        double mulFact = 255.0/5.0;
        int byteArrayCapacity = width * height;
        int[] depthMap = new int[byteArrayCapacity];
        int grayPixVal = 0;

        pointCloudData.rewind();
        for(int i =0; i < byteArrayCapacity; i++){

            //obtain grayscale representation
            grayPixVal = (int)(mulFact * (5.0- pointCloudData.get(i)));
            depthMap[i] = Color.rgb(grayPixVal, grayPixVal, grayPixVal);

        }



        return depthMap;
    }

}
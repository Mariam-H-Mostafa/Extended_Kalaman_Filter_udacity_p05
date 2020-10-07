**Extended Kalman Filter Implementation**

The required changes to implement extended Kalman filter are done by editing three files:

 1. Fusion EKF.cpp :
     Initialization is done first(R_,P_,F_),then data processing.
     ![enter image description here](https://i.ibb.co/tYBcRb1/Capture.png)

     Prediction Step : 

     ![enter image description here](https://i.ibb.co/ZgJwSfM/Picture1.png)

    Update step:    
 Update is based if I have Lidar or radar measurements.
![enter image description here](https://i.ibb.co/4YJRdPy/Capture.png)

 2. Kalman_filter.cpp
      In this file,` Predict`, `update`, and `updateEKF` are implemented.
![enter image description here](https://i.ibb.co/1znCpTD/Capture.png)

 3. Tools.cpp:
      In this file, I implemented the Jacobian and RMSE same as in course notes.  
      Please find the behavior in case of running on DataSet1 and DataSet2.
      ![enter image description here](https://i.ibb.co/hgKYc82/Picture2.png)
      ![enter image description here](https://i.ibb.co/N1NMVzB/Picture3.png)

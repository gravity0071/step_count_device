# step_count_device
Data collection and analysis device of human body limb movement state

Hardware: 
	Arduino Uno , MPU6050, MAX30102, Ky-036, HC-08, LED .etc	

	Through the cooperation of internal gyroscope and heart rate sensor, the device can judge the moving status of the person and give an alarm when the elderly falls. Through Bluetooth transmission, the processed data will be transmitted to the mobile phone for further processing. It is based on the gyroscope triaxial data processed by Gaussian filter, and the number of steps is judged by the peak-detection algorithm combining with the threshold-detection algorithm, and the moving state is judged by the threshold of the difference between the crest and valley of the wave, while the fall detection is judged by the cooperative algorithm of gyroscope data and heart rate data. Moreover, to save energy, the device will dynamically adjust the refresh rate according to the different motion state and will decide whether to start work according to whether the user is wearing the device. 

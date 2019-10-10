!!!
https://github.com/stereolabs/zed-yolo
!!!

https://answers.ros.org/question/99211/how-to-combine-a-camera-image-and-a-laser-pointcloud-to-create-a-color-pointcloud/

https://github.com/stereolabs/zed-ros-wrapper/issues/370
https://answers.ros.org/question/191265/pointcloud2-access-data/
https://pastebin.com/i71RQcU2

https://github.com/stereolabs/zed-examples/tree/master/depth%20sensing
http://wiki.ros.org/ROSNodeTutorialC%2B%2B

loop within zed depth sensing node (from main.cpp):

```cpp
while (i < 1000) {
    	// A new image is available if grab() returns SUCCESS
    	if (zed.grab(runtime_parameters) == SUCCESS) {
        	// Retrieve left image
        	zed.retrieveImage(image, VIEW_LEFT);
        	// Retrieve depth map. Depth is aligned on the left image
        	zed.retrieveMeasure(depth, MEASURE_DEPTH);
        	// Retrieve colored point cloud. Point cloud is aligned on the left image.
        	zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA);

        	// Get and print distance value in mm at the center of the image
        	// We measure the distance camera - object using Euclidean distance
        	int x = image.getWidth() / 2;
        	int y = image.getHeight() / 2;
        	sl::float4 point_cloud_value;
        	point_cloud.getValue(x, y, &point_cloud_value);

        	float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
        	printf("Distance to Camera at (%d, %d): %f m\n", x, y, distance);

        	// Increment the loop
        	i++;
    	}
}
```

coordinates of bounding box (from image.c):

```cpp
        	int left  = (b.x-b.w/2.)*im.w;
        	int right = (b.x+b.w/2.)*im.w;
        	int top   = (b.y-b.h/2.)*im.h;
        	int bot   = (b.y+b.h/2.)*im.h;
```

b.x and b.y are center of box, give these coordinates to int x and int y within the depth loop to calculate the depth of that pixel within the camera and display it on the yolo screen

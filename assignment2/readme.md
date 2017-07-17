### Introduction
This is for the simple image stitching, and ransac algorithms.

Please follow:

```
mkdir build
cd build
camke ..
make
```

### Features
The original input image for this assignment is this. Now we need to merge the two images.

<table>
	<row>
	<td><img src="img/Picture1.png" alt="Picture"</td>
	<td><img src="img/Picture2.png" alt="Picture" </td>
	</row>
</table>

First, we need to find the matching points from these two image, and first we can do manual selection. Later on, we will study the auto selection of points.

<table>
	<row>
	<td><img src="img/q1_Picture1.png" alt="Picture"</td>
	<td><img src="img/q1_Picture2.png" alt="Picture" </td>
	</row>
</table>

Now the merge image becomes,

![](img/merged.png)

### Auto selection and Ransac Algorithm
![](img/ransac.png)

More information, please refer to the design documents.

# Assignment 4 - Paraview and TTK

* Clone this repository on your local machine. Either use the terminal or a git client like GitHub Desktop or Gitkraken. IMPORTANT: Do not download the repository as a zip folder.


### Preliminary steps: Installing Paraview and TTK

Instructions can be found [here](https://topology-tool-kit.github.io/installation.html).
You will find instructions for Linux, Mac and Windows.
Honestly, I strongly discourage you from installing this on Windows. Using a VM or a dual boot machine will save you from a lot of trubles.

There will be a few differences with respect to the standard installation process.
- Downloads section: ignore this step
- Installing dependencies: ignore optional dependencies
- Preparing the sources: ignore this. You will use the folders provided with this repository
- do all the remaining steps as reported on the website link

Once Paraview and TTK are installed, you can proceed with the remaining steps.
In the remaining of this assignment we will be working with the files contained in fiels_for_ass4 only.


### Part 2: Visualizing a point cloud in Paraview

Open clustering.csv with a text reader. As you can notice, this is a set of points with two coordinates per point.

Load clustering.csv in Paraview (simple drag and drop should work).
Before clicking "Apply" always remember to check if the input file have column names. If yes then you want "Have Headers" in the Properties panel to be checked. If not, then you want to remove the check and Paraview will generate fake names for the columns.

You will see a SpreadSheetView appearing. The file is not structured and Paraview does not know anything about the data you want to visualize. We need to convert this data into a set of points. From the tab Filters->Search, search for Table To Points. Then, in the Property panel, check `2D points` to tell Paraview to consider only X and Y coordinates, and set `X Column` and `Y Column` correctly. Also, check `Render Points As Spheres`. Then Click Apply.

The set of points are now ready but not yet visible. Click on the `RenderView1` window and activate the points by clicking on the closed eye in correspondence of `TableToPoints1` in the Pipeline channel.

Now our points are there but it would be nice to have some connections between them. Select the new fitler `Delaunay 2D`. This will compute a triangulation around the points. Change the type of the representation from `Surface` to `Surface With Edges` to see the triangles. This is nice but still not optimal. The shape that these points are representing really indicate two concentric ring. Our triangulation shows something more like a disk. We can carve out big triangles by playing with the parameter `Alpha` in the Properties panel. Try values between 0 and 1 and look at how the triangulation comes out. Once you have been able to get a triangulation that nicely represent the two concentric rings, save a screenshot of your triangulation and upload it on github in the folder `files_for_ass4`.

Screenshot #1:


![chart1](/files_for_ass4/ANS1/ans1.png)



Screenshot #2:

![chart1](/files_for_ass4/ANS1/ans1a.png)


Screenshot - of the entire Screen: 


![chart1](/files_for_ass4/ANS1/ans1b.png)




### Part 3: Visualizing a 2D image

Load the file 2d.vti.
Files that end in .vt* are VTK file formats, the last letter of which indicates what type of file.

This is a grayscale image that samples a 2D scalar field. By default, ParaView maps the scalar values as colors.

This image has pixels that correspond to height values associated with the Grand Canyon. Note that if you click and drag the dataset with the mouse you can reposition it. Since this data is two-dimensional, ParaView by default loads the render view in 2D mode (you’ll see this indicated in the upper left of the view window with `2D`).

Let’s try working with a simple Filter. Go to Filters->Common->Threshold and add a Threshold filter. This filter selects only those pixels that have values within a particular range. You’ll see the minimum and maximum initialized in the `Properties` panel.

What we’d like to do is try to only select the pixels that correspond to the canyon itself. To help in deciding which values are above the canyon flow, it might be helpful to use a histogram.

* Split the view horizontally (Button close to the `RenderView1` name)
* Select Histogram as the type of your new graph
* Select this view and activate the visualization for 2d.vti
* Adjust the number of bins based on the minimum and maximum values of your dataset. (you’ll know when you’re correct because you will see no gaps between the bars in the histogram).
* Select `RenderView1` and, based on the histogram, disable the view of the entire dataset and enable the view of the threshold.
* Set the maximum value based on the histogram. We would like to remove most of the high values showing information only for the minima. Select some value near the first big peak in the histogram.
* You should get a nice blue outline of the riverbed.

Save a screenshot of your result and upload it on github in the folder `files_for_ass4`.


Screenshot #1:


![chart1](/files_for_ass4/ANS2/ans2.png)



Screenshot - of the entire Screen: 


![chart1](/files_for_ass4/ANS2/ans2a.png)







### Part 4: Visualizing 3D Image

Finally, we’ll try out visualizing a three-dimensional image in ParaView. Load 3d.vti. You should see only an outline of the bounding box at this point.

3D images are commonly visualized using axis-aligned slices. ParaView has this capability built in with the `Slice` filter. Try it out now. Create a slice that is aligned with the X normal of the image.

Rotate the dataset and look at the slice from both sides. Note that where you click is important – if you click inside the red square you can actually adjust the depth of the slice. You can also adjust this manually in the `Properties` panel. You can also rotate the slice if you click on the arrow.

Slicing along a single axis is not that informative. Sometimes, it’s helpful to create slices along multiple axes at the same time to get a 3D feel of the data. Create two additional slice filters, one aligned on the Y normal and Z normal of dataset and view all three simultaneously. Remember to select the dataset 3d.vti before applying the new slices otherwise you will be slicing an existing slice. Save a screenshot of your result and upload it on github in the folder `files_for_ass4`.



Screenshot #1a:


![chart1](/files_for_ass4/ANS4/4.2.png)



Screenshot #1b:


![chart1](/files_for_ass4/ANS3/ans_3-view1.png)



Screenshot #2:

![chart1](/files_for_ass4/ANS3/ans_3-view2.png)



Screenshot #3:


![chart1](/files_for_ass4/ANS3/ans_3-view3.png)



Screenshot #4:

![chart1](/files_for_ass4/ANS3/ans_3-view4.png)










Now we’ll create a linked view in ParaView. To do so, we’ll view the same element of the Pipeline Browser in multiple renders. First, split the view vertically so that you have a top and bottom view. Next, in the bottom view, split it horizontally twice to create 3 small views (get them all of the same size).

Make the slices visible in the small view windows. We want to have each slice contained in a single view. You’ll see that Paraview will set the rendered as a 3D view by default. Change this to a 2D view and use the camera controls so that you’ll see all slices facing the screen. You’ve now created a multiple linked view. If you adjust the properties of the slice in one panel, the other views should adjust accordingly.
Save a screenshot of your result and upload it on github in the folder `files_for_ass4`.


Screenshot #1:

![chart1](/files_for_ass4/ANS4/4.2b.png)



Screenshot #2:


![chart1](/files_for_ass4/ANS4/4.2a.png)




Now let's play with a few different filters. Remove the slices and reset the render views back to one. We would like to study this dataset in three dimensions. Let's use the filter threshold once again.

* As before, create a histogram to deduce the best values to use as threshold. Unfortunately, the histogram of the entire image is not that informative. There are too many pixels with value equal to 0.
* Define a second threshold filter on the histogram to remove the pixels with 0 value. (Also adjust the number of bins).
* Based on the newly created histogram set suitable threshold values for the main view.
Save a screenshot of your results and upload it on github in the folder `files_for_ass4`.

Screenshot #1:


![chart1](/files_for_ass4/ANS4/ans4_a.png)





Screenshot #2:

![chart1](/files_for_ass4/ANS4/Ans4a-view2.png)



Two alternative methods for visualizing a 3d dataset are isocontours or transfer functions.

* Go back to having a single view with the selected threshold
* Split the central view horizontally and split the new view vertically.
* In the upper right view create a isosurface with the contour filter. An isosurface represents all the points in the dataset having a specific field value. This is the generalization of isocontours that we have seen in geographical maps. Explore how different values create different isosurface by modifying `Value Range` in the `Properties` panel.
* In the lower right view let's create a transfer functions. Activate the visualization of the whole image and change the representation method from `Outline` to `Volume`. Select `Edit Color Map` to modify the transfer function. Play around with the color editor to change the representation of the volume
Save a screenshot of your results and upload it on github in the folder `files_for_ass4`.



Screenshot #1:


![chart1](/files_for_ass4/ANS4/ans4_b.png)





Screenshot #2:

![chart1](/files_for_ass4/ANS4/ans4b-view2.png)


Screenshot #3:

![chart1](/files_for_ass4/ANS4/ans4_c.png)









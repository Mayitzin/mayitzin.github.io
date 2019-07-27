---
layout: post
title:  "Python AHRS"
date:   2019-06-24 10:00:00 +0000
categories: Robotics
tags:   [Sensors, Robotics, Kalman, Filtering, Signal Processing, AHRS, ]
excerpt_separator: <!--more-->
---

<center>
	<img src="../../images/2019-06-24/satellite.png" width="256" height="256">
</center>

I always wanted a tool to quickly build prototypes for tracking and localization in Python. Something simple, but intuitive and easily customizable. I could not find any that satisfied me. So, I made one.

<!--more-->

Python is, undeniably, the language to prototype (and even build) scientific applications. Highly modular, very well spread and very effective when it comes to get something tangible to show around.

In the field of Robotics there are tons of libraries supporting anything you want, but all of them are centered towards unique cases and there are few instances where they connect or easily interact with other libraries.

Originally **A**ttitude and **H**eading **R**eference **S**ystems were intended to be an area for driving and tracking systems in the defense, transportation and communication fields (satellites, planes, rockets, etc.), but it has found a new area with the inclusion of embedded systems everywhere. From autonomous cars to pencils, everything is now connected and most probably containing a tracking system.

However, modern prototyping tools require a huge amount of libraries to prototype, test and/or build applications for it. I personally love [ROS](https://www.ros.org/) and the attention it has gained in the scientific and engineering areas. If anyone wants to jump into the robotic playground, that's the way to go.

I just don't like its complicated maneuver to even start with the simplest example. Don't get me wrong, I love it and still think it's the best, but is not as fast to grasp as it could be.

So, I made my own AHRS library to make those steps easier for the unexperienced (but also for the advanced) user. I simply called it like that [AHRS](https://github.com/Mayitzin/ahrs) and it can be found in its Github repository or in the [Python Package Index](https://pypi.org/project/AHRS/) like that.

I chose Python 3, more precisely Python 3.6, because it was already time for me to move on and forget about Python 2, and I also think the world should do that.

There are two ways to install it, either with pip:

```shell
pip3 install ahrs
```

or from source:

```
git clone https://github.com/Mayitzin/ahrs.git
cd ahrs
python setup.py install
```

And with it you have now AHRS in your system. This first draft is in constant developing mode, while I'll keep adding nice tools around it. For now you can load, process and visualize your data.

I've included basic Sensor Fusion and tracking algorithms, like Kalman Filters, Attitude estimation, as well as conversion between orientation representations, e.g. Euler angles and Quaternions. All functions, classes and modules have full docstrings, so that it will enable me to build and upload a nice documentation for it.

Now to the juice. I have some recent sensor data and I want to visualize them

I can do it with three lines (including the import)

```python
import ahrs
data = ahrs.utils.load('ExampleData.mat')
orientation = ahrs.filters.EKF(data)
```

And that's it. I have a full estimation of the orientation using the Extended Kalman Filter.

Do you prefer an estimation using Madgwick's algorithm? No problem!

```python
orientation = ahrs.filters.Madgwick(data)
```

The object `orientation` is an instance of `Madgwick` with the estimated attitude stored in an array of quaternions `Q`.

Many other tools are present to filter signals, fuse sensors and a fairly simple way to plot all data.

For a better glimpse of the package, I would recommend to check the repository in [Github](https://github.com/Mayitzin/ahrs).

Future tests and benchmarks of algorithms using this package are planned. I just wanna have more time to do them. We'll see.

<div>Icon made by <a href="https://www.freepik.com/?__hstc=57440181.d7545b8c79f4070b96cb70c0cda67c88.1561375703743.1561375703743.1561375703743.1&__hssc=57440181.1.1561375703743&__hsfp=4103235252" title="Freepik">Freepik</a> from <a href="https://www.flaticon.com/"                 title="Flaticon">www.flaticon.com</a> is licensed by <a href="http://creativecommons.org/licenses/by/3.0/" title="Creative Commons BY 3.0" target="_blank">CC 3.0 BY</a></div>

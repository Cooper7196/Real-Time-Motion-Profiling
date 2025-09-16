#import "@preview/ilm:1.4.1": *
#import "@preview/cetz:0.4.1"
#import "@preview/cetz-plot:0.1.2"

#import "plots.typ" 

#set text(lang: "en")
#set math.mat(delim: "[")
#set math.vec(delim: "[")

#show: ilm.with(
  title: [Motion Profiling in Vex],
  author: "Cooper Brosgall",
  date: datetime(year: 2025, month: 09, day: 03),
  abstract: [
    2D Motion Profiling is becoming much more popular in the context of Vex Robotics. This document will not hand you the code for creating a motion profile, but by following through it, it will help you understand both how it works, and how you can go about implementing it.
  ],
  bibliography: bibliography("refs.bib"),
  figure-index: (enabled: false),
  table-index: (enabled: false),
  listing-index: (enabled: false),
)

= Introduction

== Background
To best understand this document, you should have some level of understanding of the following:
#list(
  indent: 1cm,
  [Vectors in 2D space: e.g. $vec(x, y)$],
  [Cartesian (rectangular) coordinates in 2D space],
  [2D kinematics (position, velocity, acceleration, etc.)]
  
)
== Motivation
When I started competing in Vex, Motion Profiling was nearly unheard of, and there were very few documents explaining how it worked, let alone how to implement it effectively. Having spent the time learning how to implement it, and seeing it being popularized by teams such as 2654Echo, I wanted to share this knowledge with the rest of the Vex community so that more people could understand it and push it further.

== Units
For the purposes of this document, I will be working primarily with inches / imperial units, as the Vex field is defined using them. You can easily change this to use meters, and this can be useful for some algorithms such as Ramsete@ramsete. All angles will be defined in radians, as that is the default unit system for trigonometric functions such as $sin()$ and $cos()$

= 1D Motion Profiling
The first thing you need to understand to implement a 2D motion profile is what a motion profile is at all. In this document, I will be covering primarily trapezoidal motion profiles, which are constant acceleration profiles, and thus when plotted as velocity vs time, look like a trapezoid:

#align(center, [
#cetz.canvas({
  import cetz.draw: *
  import cetz-plot: *
  let vel = (
    (0.0,   0.0),
    (0.25,  0.5),
    (0.5,   1.0),
    (1.5,   1.0),
    (1.75,  0.5),
    (2.0,   0.0)
  )
  plot.plot(
    size: (13, 3),
    x-label: [time],
    y-label: [velocity],
    axis-style: "left",
    x-tick-step: none, 
    y-tick-step: none,
    legend: none,
    {
      plot.add(vel, line: "raw", label: [Velocity])
      plot.add-vline(0.5, 1.5, label: [switch])
    }
  )
})])

Profiles like this are relatively simple to implement, as they can be done purely with logic and knowledge of the basic kinematic formulas. 

In the case of a 1D motion profile, it is easy to calculate the profile of v(t) or v(x), meaning with respect to either the current time, or current distance. Either of these options can be chosen to be followed. This will change when we get to the 2D profile, which can only be generated with respect to distance, at least using the method I will describe. For this profile, I will describe the equations for either method.

For the acceleration phase of the profile, the time needed to complete it is $t_"accel" = v_"max"/a_"max"$, and the distance
$d_"accel" =v_"max"^2/(2a_"max")$.

For the coasting phase, you use distance spent accelerating and decelerating, which are the same, and subtract it from the total distance. This looks like this:

$d_"coast" = d_"total" - 2d_"accel"$

$t_"coast" = d_"coast" / v_"max"$

As mentioned, the time and distance for deceleration will be the same as acceleration, as this profile is symmetrical. These formulas can be adjusted to allow for asymmetrical acceleration and deceleration rates, but I'll leave that as an exercise to implement yourself.

Based on these formulas, you can derive the following piecewise functions:

$ v(t) := cases(
  a_"max" * t\, & 0 <= t < t_"accel",
  v_"max"\, & t_"accel" <= t < t_"accel" + t_"coast",
  v_"max" - a_"max" * (t - t_"accel" - t_"coast")\, & t_"accel" + t_"coast" <= t <= t_"total",
) $


$ v(d) := cases(
  sqrt(2 * a_"max" * d)\, & 0 <= d < d_"accel",
  v_"max"\, & d_"accel" <= d < d_"accel" + d_"coast",
  sqrt(2 * a_"max" * (d_"total" - d))\, & d_"accel" + d_"coast" <= d <= d_"total",
) $


#pagebreak()

= 2D Motion Profiling
In this document, I will be covering a technique commonly known as double-pass motion profiling, which is quite robust to arbitrary kinematic constraints. As mentioned, this will be covering trapezoidal, or constant-acceleration, profiles, but it can be extended to constant-jerk with some effort.

== Path Generation
The first step of 2D motion profiling is generating a path to follow. The requirements for such a path are relatively simple. For this method, assuming you are using a differential drive (tank drive), they will need to be at least doubly differentiable, and they should be in the form of a parametric function of t ($vec(x, y) = f(t)$).

In this paper, I will be using cubic Beziers, but you can easily use any other similar curve, such as Hermite splines, etc.

Bezier curves are generally defined in the form of: $ f(t) = (1-t)^3P_0 + 3(1-t)^2 t P_1 + 3(1-t)t^2 P_2 + t^3P_3 $However, for this paper, I will use the matrix form, as it can often be computed much more efficiently. This matrix form of a cubic bezier@bezier_matrix looks like this:

$ f(t) = mat(t^3,t^2,t,1) mat(-1,3,-3,1;3,-6,3,0;-3,3,0,0;1,0,0,0) vec(P_0,P_1,P_2,P_3) $

There are similar matrix formulations for the first and second derivatives of a cubic bezier, which we will also find useful as a part of trajectory generation, and are as follows:

$ f'(t) = mat(t^2,t,1) mat(-3,9,-9,3;6,-12,6,0;-3,3,0,0) vec(P_0,P_1,P_2,P_3) $
$ f''(t) = mat(t,1) mat(-6,18,-18,6;6,-12,6,0) vec(P_0,P_1,P_2,P_3) $

For example, if we want to generate a path from (12, 36) to (36, 48), with control points (12, 48) and (36, 36), we will need to do the following calculation:

$ mat(x, y) = mat(t^3,t^2,t,1) mat(-1,3,-3,1;3,-6,3,0;-3,3,0,0;1,0,0,0) mat(12, 36; 12,60; 36,36; 36,60) $ <bezier_sample_equation>

by running this calculation over a range of $t #sym.in [0, 1]$, we can generate the following curve:

#align(center, [
#cetz.canvas({
  import cetz.draw: *
  import cetz-plot: *
  plot.plot(
    size: (5, 5),
    x-label: [x],
    y-label: [y],
    x-min:10,
    x-max:36,
    y-min:34,
    y-max:60,
    axis-style: "left",
    x-tick-step: 5, 
    y-tick-step: 5,
    legend: none,
    {
      plot.add(((12,36), (12,60)), line:"raw",mark:"o")
      plot.add(((36,60), (36,36)), line:"raw", mark:"o")
      plot.add(plots.curve, line: "raw", label: [Velocity])
    }
  )
})
])


== Trajectory Generation

As mentioned earlier, this method of trajectory generation in 2D motion profiling involves two passes, and also requires that we parametrize the path function by distance. 

The two passes are mostly identical, with one being from start to end, and one in reverse. The core idea of the passes are a loop over each point, that follows a series of specified constraints. This will include a maximum acceleration, but in the case of a differential drive, will also include limits on linear speed based on angular speed, and can even include additional constraints to prevent side-to-side slipping.

From here on, I will focus on differential drives, but much of this can be applied to any generic kinematic constraints.

=== Point Generation

The first issue we run into is converting the bezier curve into a set of evenly spaced points, with spacing $Delta d$ (I suggest ~0.1in). There are a few ways to do this, but the easiest and one of the most efficient methods is based on using the derivative of the path. Because the derivative of the bezier $f(t)$ is equal to $"dx"/"dt"$, we can calculate the proper increase in $t$ by taking $ Delta t = (Delta d)/ (f'(t)) $ <delta_dist_equation>

=== Kinematic Constraints

Now that we can generate evenly spaced points, we need to calculate velocity we can get to from each point. The idea of having two passes is that both passes are trying to accelerate as much as they can, and the final profile is made up of the minimum of the two.

For a given pass, we know the change is distance is $Delta d$, so we can calculate what the velocity change will be based on our current velocity and max acceleration using:

$ v_f = sqrt(v^2 + 2 * a_"max" * Delta d) $ <acceleration_equation>

=== Differential Drive Kinematic Constraints

The next constraint is specific to differential drives, and it is because the robot linear and angular speeds have to be traded off between each other. This means when we take a sharp turn, the robot can't go as fast. To derive the maximum speed at a given turn radius, we can look at the kinematic equations for a tank drive.

For these equations, you will need the following definitions:
#list(
  [$l$ is the track width of the robot, which is the distance between wheels], 
  [$omega$ is the angular velocity of the robot],
  [$v_"max"$ is the maximum linear speed of the robot],
  [$v_l$ and $v_r$ are the velocities of the left and right wheels respectively],
  [$r$ is the radius of the current turn/path],
  [$kappa$ is the curvature of the current turn/path, and is equal to $1/r$]
)

The equations we will need are $ v = omega R $ <angular_equation>  $ v_r = omega(r + l/2) $ <right_wheel_equation>
$ v_l = omega(r - l/2) $ <left_wheel_equation>
By assuming that our outer wheel is at the maximum speed, and therefore is the limiting factor, we can find what linear robot velocity would cause this at a given radius. With $v_r = v_"max"$, and @right_wheel_equation we can say $omega = v_"max" / (r+l / 2)$, and combining that with @angular_equation we can show $v = (r v_"max")/ (r+l / 2)$. By simplifying and solving this in terms of curvature, we can finally show:
$ v = (2v_"max") / (2+l kappa) $ <speed_limit_equation>

=== Forward and Backwards Pass

We now have almost all the knowledge we need to actually implement the forward and backwards passes. One further equation we will likely want is the formula for curvature, which is:
$ kappa = (x'y'' - x''y')/(x'^2 + y'^2)^(3/2) $ <curvature_formula>

where x' denote the first derivative of the x position of our path, and y'' denotes the second derivative of the y position.

With that in mind, here are the steps each pass should complete as it loops:

+ Get the current position, its derivative, and its second derivative.
+ Calculate the curvature based on @curvature_formula.
+ Calculate the max speed due to curvature based on @speed_limit_equation.
+ Calculate the maximum speed you can accelerate to using @acceleration_equation and the previous speed.
+ Take the minimum of these values, along with your overall maximum speed, and assign that as the speed for the current point.
+ Calculate the angular velocity, using $omega = v * kappa$.
+ Increment t using @delta_dist_equation. 


You need to follow this procedure both forward along the path, and backward. Once you have these two paths, take the minimum of the two at each point, and the result will be a completed motion profile that will follow your path.

== Example Visualizations
Below are some visualizations that may help to see what a motion profile will look like at different stages of development. Because they are profiled vs distance instead of time, the shapes will look slightly different. All the below examples use the same path as in @bezier_sample_equation.

Here is what your first profile might look like, with only velocity limiting and only a forward pass:
#align(center, [
#cetz.canvas({
  import cetz.draw: *
  import cetz-plot: *
  plot.plot(
    size: (13, 2),
    x-label: [distance],
    x-min:-2,
    y-label: [velocity],
    axis-style: "left",
    x-tick-step: 6, 
    y-tick-step: 20,
    legend: none,
    {
      plot.add(plots.basic_forward_profile, line: "raw", label: [Velocity])
    }
  )
})])
This is what it will look like once you add all of the constraints:
#align(center, [
#cetz.canvas({
  import cetz.draw: *
  import cetz-plot: *
  plot.plot(
    size: (13, 2),
    x-label: [distance],
    x-min:-2,
    y-label: [velocity],
    axis-style: "left",
    x-tick-step: 6, 
    y-tick-step: 20,
    legend: none,
    {
      plot.add(plots.forward_profile, line: "raw", label: [Velocity])
    }
  )
})])
This is what it will look like once you add in the proper backward pass:
#align(center, [
#cetz.canvas({
  import cetz.draw: *
  import cetz-plot: *
  plot.plot(
    size: (13, 2),
    x-label: [distance],
    x-min:-2,
    y-label: [velocity],
    axis-style: "left",
    x-tick-step: 6, 
    y-tick-step: 20,
    legend: none,
    {
      plot.add(plots.good_profile, line: "raw", label: [Velocity])
    }
  )
})])

= Extending Motion Profiling

There are still some things you will need beyond pure motion profiling to actually control a robot. Here I will briefly cover some of these things, and highlight some areas where this algorithm can be improved. 
== Velocity Controller
Once you have a motion profile, you need to actually follow it. Because it outputs only linear and angular velocity, you need to first convert this to wheel velocities, and then command the wheels to move at these velocities.

To calculate individual wheel velocities, you will need to use @right_wheel_equation and @left_wheel_equation to calculate $v_l$ and $v_r$. You then need to convert these velocities along the ground to rpm, which can be done based on the size of your wheels.

Finally, to convert the desired rpm to a voltage you can apply, you need a velocity controller. You can use the Vex internal velocity controller, but there are many ways to improve upon it. The simplest velocity controller is the feed forward model:
$ V = "kS" * "sgn"(v) + "kV" * v + "kA" * a $
where:
- $V$ is the applied voltage
- $v$ is the target velocity
- $a$ is the target acceleration
- $"kS"$ is the minimum voltage at which the motor will move
- $"kV"$ is the velocity gain, which is the voltage the motor need to maintain at a given speed when combined with $v$
- $"kA"$ is the acceleration gain, which is the voltage needed to maintain a constant acceleration when combined with $a$

This is often easiest to tune by looking at a 1D motion profile and adjusting the gains until it is followed very closely.

In addition, you can use a PID to maintain velocity, but I will not go into any more depth on velocity control here.

== Additional Constraints
There are many possible kinematic constraints you could add with this technique, such as limiting angular acceleration, or many others, but one I might suggest is limit speed based on side-to-side friction. This can prevent slipping when taking sharp turns. The formula for maximum speed for a given friction coefficient is:
$ v = sqrt(mu * r * g) $
where: 
- $mu$ is the friction coefficient
- $r$ is the radius of the turn, or $1/kappa$
- $g$ is gravitational acceleration, $9.81m/s^2$, or $32.174 "ft"/s^2$

== Time Parameterization
You may want to remap your profile to be velocity vs time at some point, as it can be easier to follow. This is relatively straightforward to do. Simple choose a time interval for spacing, generally 10ms for Vex, and iterate over the distance profile while simulating your current velocity and distance.

The loop might look like:

```cpp
  while (currDist < pathDist) {
    currVel = trajectory[(int)(currDist / deltaDist)]
    currDist += currVel * dt;
    timeTrajectory.push_back(currVel);
  }
```
This won't be perfect, and I suggest implementing some amount of interpolation, or oversampling to improve the sharpness of the graph. Such a time-parametrized motion profile can be seen below:

#align(center, [
#cetz.canvas({
  import cetz.draw: *
  import cetz-plot: *
  plot.plot(
    size: (13, 2),
    x-label: [time],
    y-label: [velocity],
    x-min:-0.1,
    axis-style: "left",
    x-tick-step: auto, 
    y-tick-step: 20,
    legend: none,
    {
      plot.add(plots.time_profile, line: "raw", label: [Velocity])
    }
  )
})])

== Feedback
Finally, on top of the open loop motion profiling, you will likely want error correction of some sort. The most commonly used algorithm for this is Ramsete@ramsete, which I will not go into in depth, but takes an input error and target, and will output a corrected angular and linear velocity. There are other methods as well, but I will not go into them here, and rather leave it as a suggestion that closing the loop is not necessary, but can allow for much more accurate or quicker movements.

= Conclusion
I hope you find this document helpful and informative. If you have any further questions, feel free to reach out to me, I am always happy to help people out with this or other programming problems. This is the first document I've written like this, so I am open to any kind of feedback you may have. Please contact me on the Discord as \@comodomo, or by email at #link("mailto:cbrosgall@gmail.com")[cbrosgall\@gmail.com].
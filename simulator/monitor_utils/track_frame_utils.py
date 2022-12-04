"""
this week/weekend:
- finish rrt, pure pursuit and simulation code -> go through graph code (geeksforgeeks) and graph theory (discrete math -> induction and recursion theory). Get the simulation up and running with breach (all signal generation and logging code)
- move to obsidian and figure out how it works with its note taking stuff (just pay if you need to)
- look into kdtrees and complete data structures if you get a chance (khosaravi)

next weekend:
- fix your pure pursuit and run rrt (take videos of your code)
- jumpstart your computervision stuff and get something working 

then go home lol

collect resources from all computers and move to obsidian! Become much better at computer programming through DSA, leetcode, ray-tracing, game-dev, etc. 

Come up with a data structure -> a way to store the points so you can quickly find its 'nearest neighbour' -> has applications to pure pursuit and track based co-ordinate system -> maybe something like a binary search tree? that lets you quickly maximise and minimise
current solution -> use a k-d tree (https://en.wikipedia.org/wiki/K-d_tree) and do a nearest neighbour search with euclidean distance
similar to binary trees with binary search
https://en.wikipedia.org/wiki/Nearest_neighbor_search
https://stackoverflow.com/questions/62915817/data-structure-to-find-value-for-closest-point-to-input-data
look at hashing and bidicting (revise your track co-ordinate architecture and your gap operator)

UPDATE: scipy implementation of kd trees: https://pythonguides.com/python-scipy-kdtree/ http://library.isr.ist.utl.pt/docs/scipy/spatial.html

study networking. fill in the computer science, dbms, os, compiler and engineering gaps and design and deploy your own big system (like a server that does stuff)
do research in physics, electronics (s-parameters), bio systems, etc
speak to sebastian and learn all the different research and work he is doing 

microsteps: 
- read about k-nearest neighbour algo -> done
- complete non-linear reading
- complete bezier reading
- make roadmap for implementation and implement it tomorrow -> revise your track architecture. Read up on bezier and complete implementation

write your rendering function/callback and pass it -> to render bezier curves ocgrids etc. 

analyse newton's/bisection method from a signals and systems pov and determine stability (hint: use z transform to find poles of the system)
also analyse convergence of that system
 """

 """
 do your misc reading until 5: signals and systems -> discrete appro. of continous systems and rate of convergence
 jordan canonical forms
 gradients and jacobians
 non-linear reading

 jordan canonical form: brilliant, stanford video

 move on to bezier curves and revise your co-ordinate system. start coding it up with all the caching you think you need!

 readings till 4:00:
 - diffy qs from paul (systems of qs)
 - quadtratic forms
 - light reading on matrix vs tensor
 - jordan canonical form
 - systems equations stuff from math 307
 - non-linear reading

 revise your dynamics and fluids and connect with RTT and multivariable calc and space continous math. Connect that with graph based methods and computer science
 look at phase planes and come back to linearity of a diffy qs matrix

 TODO:
 Need to reconcile ideas across the board - from continous math (higher order calculus) to signals and systems:
 - cover responses of these systems. Signals and systems representation and how their discrete/continuous representations are stable/unstable
 - systems of linear, non-linear, differential (linear and non-linear) equations and how to solve them. Their error and stability of these systems. Tie with signals and systems and convert these representations  (e.g. equation representation to laplace/z to block diagrams)
 - linear algebra and numerical solving 
 - linear algebra research
 - patch your linear algebra holes with pure + applied resources and then stitch with the rest
 - ultimately this all comes together with classical, modern and advanced control theory
 - signal processing, conditioning -> discrete and continous (theory + applied) -> fourier 

 connect vandermonde and polynomial interpolation with bezier curve. Revise what you need and solve stuff accordingly

 Simulation TODO:
 - track coord utils - >bezier curves and cubic splines -> normal is perpendicular to tangent (just rotate by 90 or differentiate or use -v,u)
 - pure pursuit (no vel profiles)
 - rrt
 - signal bagging (use pandas data frames and write to csv)
 - formalise stuff in latex and write the thing for the call to the paper

 microsteps: 
 - finish cubic spline interp from pwalls
 - figure out your track based co-ords
 
 WORK WITH BEZIER CURVES AS THEY CAN BE PARAMETERISED (and you can find progress along it) -> can do this with cubic splines too lol

 bexier primer
 berk python
 pwalls python
 scipy
 parameterise a polynomial

 complete above readings!
"""
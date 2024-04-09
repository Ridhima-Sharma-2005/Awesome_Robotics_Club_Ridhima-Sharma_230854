from fabrikSolver import FabrikSolver3D

arm = FabrikSolver3D()
a=float(input("enter the x coordinate:"))
b=float(input("enter the y coordinate :"))
c=float(input("enter the z coordinate:"))

l=[23,15,4]
#  target of 42 0 0 will show unreachable due to the error margin used 
for i in range(3):
    print("segment",i+1)
    zangle=float(input("enter initial angle of the segment along the z axis"))
    yangle=float(input("enter initial angle of the segment along the y axis"))
    arm.addSegment(l[i],zangle,yangle)
    # print(l[i],zangle,yangle)


arm.compute(a,b,c)
# arm.addSegment(23,0.0,0.0) # for these values the code is not working . dont know the reason but when given explicitly the same values it work fines
# arm.addSegment(15,0.0,0.0)
# arm.addSegment(4,0.0,0.0)
# arm.compute(41.99,0.0,0.0)
arm.plot()


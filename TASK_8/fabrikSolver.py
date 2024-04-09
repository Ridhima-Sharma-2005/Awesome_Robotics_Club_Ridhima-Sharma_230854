import numpy as np
import math
import sys
import matplotlib as mpl
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
from mpl_toolkits.mplot3d import axes3d

def unitVector(vector):
    """ 
        Returns the unit vector of a given input vector. 

        Params:
            vector -> input vector.

        Returns:
            numpy.array().
    """

    # Divide the input vector by its magnitude.
    return vector / np.linalg.norm(vector)


class Segment3D:
    """ 
        A part of the FabrikSolver3D to store a part of an inverse kinematics chain.
    """

    def __init__(self, referenceX, referenceY, referenceZ, length, zAngle, yAngle):

        """ 
            Params:
                referenceX -> x component of the reference point.

                referenceY -> y component of the reference point.

                referenceZ -> Z component of the reference point.

                length -> length of the segemnt.

                zAngle -> initial angle along the z axis of the segment.
                
                yAngle -> initial angle along the y axis of the segment.
        """

        self.zAngle = zAngle
        self.yAngle = yAngle

        # Store the length of the segment.
        self.length = length

        # Calculate new coördinates.
        deltaX = math.cos(math.radians(zAngle)) * length
        deltaY = math.sin(math.radians(zAngle)) * length
        deltaZ = math.sin(math.radians(yAngle)) * length

        # Calculate new coördinates with respect to reference.
        newX = referenceX + deltaX
        newY = referenceY + deltaY
        newZ = referenceZ + deltaZ

        # Store new coördinates.
        self.point = np.array([newX, newY, newZ])

class FabrikSolver3D:
    """ 
        An inverse kinematics solver in 3D. Uses the Fabrik Algorithm.
    """
    def __init__(self, baseX=0, baseY=0, baseZ=0,  marginOfError=0.01):
        """
            Params:
                baseX -> x component of the base.

                baseY -> y coördinate of the base.

                baseZ -> z coördinate of the base.

                marginOfError -> the margin of error for the algorithm.
        """

        # Create the base of the chain.
        self.basePoint = np.array([baseX, baseY, baseZ])

        # Initialize empty segment array -> [].
        self.segments = []

        # Initialize length of the chain -> 0.
        self.armLength = 0

        # Initialize the margin of error.
        self.marginOfError = marginOfError

    def addSegment(self, length, zAngle, yAngle):

        """ 
            Add new segment to chain with respect to the last segment.

            Params:
                length -> length of the segment.

                zAngle -> initial angle of the segment along the z axis.

                yAngle -> initial angle of the segment along the y axis.
        """

        if len(self.segments) > 0:

            segment = Segment3D(self.segments[-1].point[0], self.segments[-1].point[1], self.segments[-1].point[2], length, zAngle + self.segments[-1].zAngle, self.segments[-1].yAngle + yAngle)
        else:
            #if the segment length is less than 0 the new point will be the base point
            segment = Segment3D(self.basePoint[0], self.basePoint[1], self.basePoint[2], length, zAngle, yAngle)

        
        self.armLength += segment.length

        
        self.segments.append(segment)

    def isReachable(self, targetX, targetY, targetZ):
        """  
            Check if a point in space is reachable by the end-effector.

            Params:
                targetX -> the target x coördinate to check.
                
                targetY -> the target y coördinate to check.

                targetZ -> the target z coördinate to check. 

            Returns:
                Boolean.  
        """

        if np.linalg.norm(self.basePoint - np.array([targetX, targetY, targetZ])) < self.armLength:
            return True
        return False

    def inMarginOfError(self, targetX, targetY, targetZ):
        """  
            Check if the distance of a point in space and the end-effector is smaller than the margin of error.

            Params:
                targetX -> the target x coördinate to check.
                
                targetY -> the target y coördinate to check.

                targetZ -> the target z coördinate to check. 

            Returns:
                Boolean.  
        """
        if np.linalg.norm(self.segments[-1].point - np.array([targetX, targetY, targetZ])) < self.marginOfError:
            return True
        return False     

    def iterate(self, targetX, targetY, targetZ):
        """ 
            Do one iteration of the fabrik algorithm. Used in the compute function. 
            Use in simulations or other systems who require motion that converges over time.  

            Params:
                targetX -> the target x coördinate to move to.
            
                targetY -> the target y coördinate to move to.

                targetZ -> the target y coördinate to move to.
        """

        target = np.array([targetX, targetY, targetZ])

        # Backwards.
        for i in range(len(self.segments) - 1, 0, -1):

            # On the end, we must first use the endpoint in order to apply the formula.

            # See if the value of i is equal to the index of the last vector on the arm.
            if i == len(self.segments) - 1:
                # Go one index lower to the penultimate vector in the list. Then use the formula with the end vector and multiply by the length of the vector by the last index.

                # Replace old vector with new vector.
                self.segments[i-1].point = (unitVector(self.segments[i-1].point - target) * self.segments[i].length) + target

            else:
                self.segments[i-1].point = (unitVector(self.segments[i-1].point - self.segments[i].point) * self.segments[i].length) + self.segments[i].point

         # Forwards.
        for i in range(len(self.segments)):
            if i == 0:
                self.segments[i].point = (unitVector(self.segments[i].point - self.basePoint) * self.segments[i].length) + self.basePoint

            elif i == len(self.segments) - 1:
                self.segments[i].point = (unitVector(self.segments[i-1].point - target) * self.segments[i].length * -1) + self.segments[i-1].point

            else:
                self.segments[i].point = (unitVector(self.segments[i].point - self.segments[i-1].point) * self.segments[i].length) + self.segments[i-1].point

    def compute(self, targetX, targetY, targetZ):

        """  
            Iterate the fabrik algoritm until the distance from the end-effector to the target is within the margin of error.

            Params:
                targetX -> the target x coördinate to move to.
                
                targetY -> the target x coördinate to move to.

                targetZ -> the target z coördinate to move to.
        """
        
        if self.isReachable(targetX, targetY, targetZ):
            while not self.inMarginOfError(targetX, targetY, targetZ):
                self.iterate(targetX, targetY, targetZ)         
        else:
            print('Target not reachable.')
            sys.exit()

    def plot(self, save=False, name="graph"):
        """  
            Plot the chain.

            Params:
                save -> choose to save the plot to a file.

                name -> give the plot a name.
        """

        fig = plt.figure()
        ax1 = fig.add_subplot(111, projection="3d")

        # Plot arm.
        for segment in self.segments:
            print(segment.point[0],segment.point[1],segment.point[2])
            
            ax1.scatter(segment.point[2], segment.point[0], segment.point[1], c='r')
            # plt.text(segment.v[0], segment.v[1] + 1, '(x:{}, y:{})'.format(int(segment.v[0]), int(segment.v[1])))
        line1=np.array([self.segments[0].point[0]-self.basePoint[0],self.segments[0].point[1]-self.basePoint[1],self.segments[0].point[2]-self.basePoint[2]])
        line2=np.array([self.segments[1].point[0]-self.segments[0].point[0],self.segments[1].point[1]-self.segments[0].point[1],self.segments[1].point[2]-self.segments[0].point[2]])
        line3=np.array([self.segments[2].point[0]-self.segments[1].point[0],self.segments[2].point[1]-self.segments[1].point[1],self.segments[2].point[2]-self.segments[1].point[2]])
        dot_product = np.dot(line1,line2)
        dot_product2=np.dot(line2,line3)

# Calculate the magnitudes of the vectors
        magnitude_line1 = math.sqrt(line1[0]**2+line1[1]**2+line1[2]**2)
        magnitude_line2 = math.sqrt(line2[0]**2+line2[1]**2+line2[2]**2)
        magnitude_line3= math.sqrt(line3[0]**2+line3[1]**2+line3[2]**2)

# Calculate the cosine of the angle
        cos_angle = dot_product / (magnitude_line1 * magnitude_line2)
        cos_angle2=dot_product2/(magnitude_line2*magnitude_line3)

# Calculate the angle in radians
        angle_rad = np.arccos(cos_angle)
        angle_rad2=np.arccos(cos_angle2)

# Convert angle to degrees if desired
        angle_deg = np.degrees(angle_rad)
        angle_deg2 = np.degrees(angle_rad2)
        print("Angle between the segment1 and segment2:", angle_deg)
        print("Angle between the lines segment2 and segment3:", angle_deg2)
        # Startpunt
        ax1.scatter(self.basePoint[2], self.basePoint[0], self.basePoint[1])

        ax1.set_xlabel('z-axis')
        ax1.set_ylabel('x-axis')
        ax1.set_zlabel('y-axis')

        plt.show()
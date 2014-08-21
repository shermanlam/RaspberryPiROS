import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def run():
    """
    Opens a gps file and plots it
    """
    # get the file
    filename = "gps6.txt"
    data = np.loadtxt(filename)
    
    # get the lat, lon, and altitude
    latLonAlt = data[:,2:5]

    # convert lat and lon to meters use the first point for reference.
    dist = get_dist(latLonAlt)

    # outlier rejection
    length = np.shape(dist)[0]
    maxDist = np.multiply(np.ones((length,2)),2000)
    dist[:,0:2] = np.minimum(dist[:,0:2],maxDist)

    # plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(dist[:,0:1],dist[:,1:2],dist[:,2:3])
    plt.show()


def get_dist(array):
    """
    Input: numpy array where first two columns is lat lon
    Output: array of distances in meters
    """
    shape = np.shape(array)
    dist = np.zeros((shape[0],3),np.float64)
    ref = array[0,0:3]                  # reference position
    for i in range(shape[0]-1):
        pt = array[i,0:2]
        y = np.asarray((ref[0],pt[1]))    # get y change
        x = np.asarray((pt[0],ref[1]))    # get x change
        dist[i][0] = haversine(ref,x)
        dist[i][1] = haversine(ref,y)
        dist[i][2] = array[i][2]-ref[2]
    return dist


def haversine(point1,point2):
    """
    Input: latitude and longitude
    Output: distance between two points. uses haversine formula

    equation from: http://www.movable-type.co.uk/scripts/latlong.html
    """
    lat1 = point1[0]
    lon1 = point1[1]
    lat2 = point2[0]
    lon2 = point2[1]
    dLat = lat2-lat1    # change in latitude
    dLon = lon2-lon1    # change in longitude
    R = 6371*1000       # radius of earth in meters
    a = math.pow(math.sin(dLat/2),2)+math.cos(lat1)*math.cos(lat2)*math.pow(math.sin(dLon/2),2)
    c = 2*math.atan2(math.sqrt(a),math.sqrt(1-a))
    d = R*c             # distance in meters
    return d
    

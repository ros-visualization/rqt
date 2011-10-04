from math import *

def zero3():
    return (0.0,0.0,0.0)

def copy3(v):
    return (v[0],v[1],v[2])

def inverse3(v):
    return (-v[0],-v[1],-v[2])

def add3(v1,v2):
    return (v1[0]+v2[0],v1[1]+v2[1],v1[2]+v2[2])

def sub3(v1,v2):
    return (v1[0]-v2[0],v1[1]-v2[1],v1[2]-v2[2])

def scale3(v,s):
    return (v[0]*s,v[1]*s,v[2]*s)

def lengthSq3(v):
    return v[0]*v[0]+v[1]*v[1]+v[2]*v[2]

def length3(v):
    return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])

def normalize3(v):
    l = length3(v)
    if l == 0:
        return (0.0,0.0,0.0)
    return (v[0]/l,v[1]/l,v[2]/l)

def dot3(v1,v2):
    return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]

def cross3(v1,v2):
    return (v1[1]*v2[2]-v1[2]*v2[1],v1[2]*v2[0]-v1[0]*v2[2],v1[0]*v2[1]-v1[1]*v2[0])

def perpendicular3(v):
    if v[1]==0 and v[2]==0:
        return cross3(v,add3(v,(0,1,0)))
    return cross3(v,add3(v,(1,0,0)))

def zeroQ():
    return (1.0,0.0,0.0,0.0)

def copyQ(q):
    return (q[0],q[1],q[2],q[3])

def addQ(q1,q2):
    return (q1[0]+q2[0],q1[1]+q2[1],q1[2]+q2[2],q1[3]+q2[3])

def subQ(q1,q2):
    return (q1[0]-q2[0],q1[1]-q2[1],q1[2]-q2[2],q1[3]-q2[3])

def scaleQ(q,s):
    return (q[0]*s,q[1]*s,q[2]*s,q[3]*s)

def magnitudeSqQ(q):
    return q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]

def magnitudeQ(q):
    return sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3])

def conjugateQ(q):
    return (q[0],-q[1],-q[2],-q[3])

def multiplyQ(q1,q2):
    w1,x1,y1,z1 = q1[0],q1[1],q1[2],q1[3]
    w2,x2,y2,z2 = q2[0],q2[1],q2[2],q2[3]
    return (w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 + y1*w2 + z1*x2 - x1*z2,
            w1*z2 + z1*w2 + x1*y2 - y1*x2)

def normalizeQ(q):
    m = magnitudeQ(q)
    if m==0:
        return (1.0,0.0,0.0,0.0)
    return (q[0]/m,q[1]/m,q[2]/m,q[3]/m)

def inverseQ(q):
    m2 = magnitudeSqQ(q)
    return (q[0]/m2,-q[1]/m2,-q[2]/m2,-q[3]/m2)

def dotQ(q1,q2):
    return q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]

def fromAngleAxisQ(radians,x,y,z):
    radians/=2.0
    s = sin(radians)/sqrt(x*x+y*y+z*z)
    return normalizeQ((cos(radians),x*s,y*s,z*s))

def toMatrixQ(q):
    w,x,y,z = q[0], q[1], q[2], q[3]
    xx = 2.0*x*x
    yy = 2.0*y*y
    zz = 2.0*z*z
    xy = 2.0*x*y
    zw = 2.0*z*w
    xz = 2.0*x*z
    yw = 2.0*y*w
    yz = 2.0*y*z
    xw = 2.0*x*w
    return (1.0-yy-zz, xy-zw, xz+yw, 0.0,
             xy+zw, 1.0-xx-zz, yz-xw, 0.0,
             xz-yw, yz+xw, 1.0-xx-yy, 0.0,
             0.0, 0.0, 0.0, 1.0)

def rotateVectorQ(q,v):
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    x, y, z = v[0], v[1], v[2]

    ww = qw*qw
    xx = qx*qx
    yy = qy*qy
    zz = qz*qz
    wx = qw*qx
    wy = qw*qy
    wz = qw*qz
    xy = qx*qy
    xz = qx*qz
    yz = qy*qz

    return (ww*x + xx*x - yy*x - zz*x + 2*((xy-wz)*y + (xz+wy)*z),
            ww*y - xx*y + yy*y - zz*y + 2*((xy+wz)*x + (yz-wx)*z),
            ww*z - xx*z - yy*z + zz*z + 2*((xz-wy)*x + (yz+wx)*y))

def interpolateQ(q1, q2, s, shortest=True):
    ca = dotQ(q1,q2)
    if shortest and ca<0:
        ca = -ca
        neg_q2 = True
    else:
        neg_q2 = False
    o = acos(ca)
    so = sin(o)

    if (abs(so)<=1E-12):
        return copyQ(q1)

    a = sin(o*(1.0-s)) / so
    b = sin(o*s) / so
    if neg_q2:
        return subQ(scaleQ(q1,a),scaleQ(q2,b))
    else:
        return addQ(scaleQ(q1,a),scaleQ(q2,b))
    
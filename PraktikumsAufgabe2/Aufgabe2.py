from PraktikumsAufgabe1.Transformations import *
import numpy as np
import math
import matplotlib.pyplot as plt

def main():
    #a)
    print("Aufgabe a):")
    po = get_point(0.1, 0.1, 40, 30, -10)
    print("PO: " + str(po))

    #b)
    print("Aufgabe b):")
    pdb = get_point_db(0, 0, 40, 30, -10)
    print("PD: "+ str(pdb))

    alpha_r = np.arctan2(pdb[1], pdb[0])
    alpha = np.rad2deg(alpha_r)
    print("Alpha: " + str(alpha))

    t1 = (0, 0, 0)
    r1 = rotz(-alpha)
    r2 = rotx(-90)
    tdbd = np.dot(np.dot(trans(t1), rot2trans(r2)), rot2trans(r1))
    pd_rot = np.dot(tdbd, pdb)

    x2 = pd_rot[0]
    y2 = pd_rot[1]
    l1 = 0.5
    l2 = 0.5
    epsilon = -1

    l = math.sqrt(math.pow(x2, 2) + math.pow(y2, 2))
    c = (math.pow(l, 2) - math.pow(l1, 2) - math.pow(l2, 2)) / (2 * l1)
    b = epsilon * math.sqrt(math.pow(l2, 2) - math.pow(c, 2))

    beta_2_r = np.arctan2(b, c)
    beta_2 = np.rad2deg(beta_2_r)
    beta_1_r = np.arctan2(y2, x2) - np.arctan2(b, l1 + c)
    beta_1 = np.rad2deg(beta_1_r)
    print("Beta 1: " + str(beta_1))
    print("Beta 2: " + str(beta_2))

    print("Überprüfung mit Vorwärtskinemtik")
    po_test = get_point(0, 0, 40, 30, -10)
    print(po_test)

    po = get_point(0, 0, alpha, beta_1, beta_2)
    print(po)

    print("Aufgabe c):")
    alphas = []
    betas_1 = []
    betas_2 = []
    angles = []

    x =[]
    y = []

    for p in points_in_circum_offset(0.1):
        alpha, beta_1, beta_2 = reverse_point(p[:4])
        x.append(p[1]*100)
        y.append(p[2]*100)
        alphas.append(alpha)
        betas_1.append(beta_1)
        betas_2.append(beta_2)
        angles.append(p[4])

    fig, (offset, circle, line) =plt.subplots(1,3)
    offset.plot(angles, alphas, 'r', angles, betas_1 , 'b', angles, betas_2 , 'g')
    offset.set_title("circle with offset (0.5/0.5)")

    alphas = []
    betas_1 = []
    betas_2 = []
    angles = []

    x =[]
    y = []

    for p in points_in_circum(0.6):
        alpha, beta_1, beta_2 = reverse_point(p[:4])
        x.append(p[1]*100)
        y.append(p[2]*100)
        alphas.append(alpha)
        betas_1.append(beta_1)
        betas_2.append(beta_2)
        angles.append(p[4])


    circle.plot(angles, alphas, 'r', angles, betas_1 , 'b', angles, betas_2 , 'g')
    circle.set_title("circle")
    plt.draw()

    alphas = []
    betas_1 = []
    betas_2 = []
    angles = []

    x =[]
    y = []

    for p in points_in_line():
        alpha, beta_1, beta_2 = reverse_point(p[:4])
        x.append(p[1]*100)
        y.append(p[2]*100)
        alphas.append(alpha)
        betas_1.append(beta_1)
        betas_2.append(beta_2)
        angles.append(p[4])

    line.plot(angles, alphas, 'r', angles, betas_1 , 'b', angles, betas_2 , 'g')
    line.set_title("line")
    plt.show()

def points_in_circum(r,n=360):
    return [(0.5,math.cos(2*math.pi/n*x)*r, math.sin(2*math.pi/n*x)*r, 1, np.rad2deg(2*math.pi/n*x)) for x in range(0,n+1)]

def points_in_circum_offset(r,n=360):
    return [(0.5,math.cos(2*math.pi/n*x)*r+0.5, math.sin(2*math.pi/n*x)*r+0.5, 1, np.rad2deg(2*math.pi/n*x)) for x in range(0,n+1)]

def points_in_line():
    return [(0.5,  x*0.01, 0, 1, x*0.01) for x in range(-80,80)]

def reverse_point(p):
    alpha_r = np.arctan2(p[1], p[0])
    alpha = np.rad2deg(alpha_r)

    t1 = (0, 0, 0)
    r1 = rotz(-alpha)
    r2 = rotx(-90)
    tdbd = np.dot(np.dot(trans(t1), rot2trans(r2)), rot2trans(r1))
    pd_rot = np.dot(tdbd, p)

    x2 = pd_rot[0]
    y2 = pd_rot[1]
    l1 = 0.5
    l2 = 0.5
    epsilon = -1

    l = math.sqrt(math.pow(x2, 2) + math.pow(y2, 2))
    c = (math.pow(l, 2) - math.pow(l1, 2) - math.pow(l2, 2)) / (2 * l1)
    b = epsilon * math.sqrt(math.pow(l2, 2) - math.pow(c, 2))

    beta_2_r = np.arctan2(b, c)
    beta_2 = np.rad2deg(beta_2_r)
    beta_1_r = np.arctan2(y2, x2) - np.arctan2(b, l1 + c)
    beta_1 = np.rad2deg(beta_1_r)

    return alpha, beta_1, beta_2

def get_point(a, b, alpha, beta_1, beta_2):
    xr = 2
    yr = 1
    theta = 30

    l = 0.6
    h = 0.2
    r = 0.1

    l1 = 0.5
    l2 = 0.5

    t1 = (xr, yr, r)
    r1 = rotz(theta)
    tor = np.dot(trans(t1), rot2trans(r1))

    t1 = (l/2 - a/2, 0, h)
    trdb = trans(t1)

    t1 = (0, 0, b/2)
    r1 = rotz(alpha)
    r2 = rotx(90)
    tdbd = np.dot(np.dot(trans(t1), rot2trans(r1)), rot2trans(r2))

    t1 = (0, 0, a/2)
    r1 = rotz(beta_1)
    t2 = (l1, 0, 0)
    tda1 = np.dot(np.dot(trans(t1), rot2trans(r1)), trans(t2))

    r1 = rotz(beta_2)
    t1 = (l2, 0, 0)
    ta1a2 = np.dot(rot2trans(r1), trans(t1))

    toa2 = np.dot(np.dot(np.dot(np.dot(tor, trdb), tdbd), tda1), ta1a2)
    pa2 = (0, 0, 0, 1)
    po = np.dot(toa2, pa2)
    return po

def get_point_db(a, b, alpha, beta_1, beta_2):
    l1 = 0.5
    l2 = 0.5

    t1 = (0, 0, b/2)
    r1 = rotz(alpha)
    r2 = rotx(90)
    tdbd = np.dot(np.dot(trans(t1), rot2trans(r1)), rot2trans(r2))

    t1 = (0, 0, a/2)
    r1 = rotz(beta_1)
    t2 = (l1, 0, 0)
    tda1 = np.dot(np.dot(trans(t1), rot2trans(r1)), trans(t2))

    r1 = rotz(beta_2)
    t1 = (l2, 0, 0)
    ta1a2 = np.dot(rot2trans(r1), trans(t1))

    tdba2 = np.dot(np.dot(tdbd, tda1), ta1a2)
    pa2 = (0, 0, 0, 1)
    pdb = np.dot(tdba2, pa2)
    return pdb

if __name__ == "__main__":
    main()

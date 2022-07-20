from Transformations import *
import numpy as np


def main():
    # Aufgabe 2.1
    print("Aufgabe 2.1:")
    # a) https://stackoverflow.com/questions/18646477/why-is-sin180-not-zero-when-using-python-and-numpy
    print("a)")
    r1 = rotz(180)
    t1 = (-2, 0, 0)
    tab = np.dot(trans(t1), rot2trans(r1))
    print("T^A_B:")
    print(tab)

    r2 = rotz(270)
    t2 = (-4, -1, 0)
    tbc = np.dot(trans(t2), rot2trans(r2))
    print("T^B_C:")
    print(tbc)

    r3 = rotz(90)
    t3 = (2, 1, 0)
    tac = np.dot(trans(t3), rot2trans(r3))
    print("T^A_C:")
    print(tac)

    # b)
    print("\nb)")
    tac = np.dot(tab, tbc)
    print("T^A_C:")
    print(tac)

    # c)
    print("\nc)")
    r4 = rotz(270)
    t4 = (-1, 2, 0)
    tca = np.dot(trans(t4), rot2trans(r4))
    # r3_transposed = np.transpose(r3)
    # tac_inverted = np.dot(trans(-np.dot(r3_transposed, t3)), rot2trans(r3_transposed))
    tac_inverted = np.linalg.inv(tac)

    print("T^C_A:")
    print(tca)
    print("(T^A_C)^-1:")
    print(tac_inverted)

    # d)
    print("\nd)")
    pb_homogenous = np.array([[-3], [1], [0], [1]])
    pa_homogenous = np.dot(tab, pb_homogenous)
    print("p^A homogenous:")
    print(pa_homogenous)

    # Aufgabe 2.2
    print("\n\nAufgabe 2.2:")
    # a)
    print("a)")
    r1 = rot(0)
    t1 = (1, 1)
    toa = np.dot(trans(t1), rot2trans(r1))
    print("T^O_A:")
    print(toa)
    print("\n")

    r2 = rot(30)
    t2 = (3, 2)
    tob = np.dot(trans(t2), rot2trans(r2))
    print("T^O_B:")
    print(tob)

    # b)
    print("\nb)")
    pb_homogenous = np.array([[1], [1], [1]])
    po_homogenous = np.dot(tob, pb_homogenous)
    print("p^O homogenous:")
    print(po_homogenous)

    # c)
    print("\nc)")
    r1_transposed = np.transpose(r1)
    toa_inverted = np.dot(trans(-np.dot(r1_transposed, t1)), rot2trans(r1_transposed))
    tab = np.dot(toa_inverted, tob)
    print("T^A_B:")
    print(tab)

    # d)
    print("\nd)")
    pa = np.dot(tab, pb_homogenous)
    print("p^A:")
    print(pa)

    # e)
    print("\ne)")
    pa_homogenous = np.array([[pa[0, 0]], [pa[1, 0]], [1]])
    p_homogenous = np.dot(tab, pa_homogenous)
    print("translated and rotated homogenous point p:")
    print(p_homogenous)


if __name__ == "__main__":
    main()

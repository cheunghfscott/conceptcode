"""
===========================
ArduinoArm stimulation
===========================

"""

from numpy import sin, cos, arctan, double
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation
from sympy import *
from numpy.linalg import inv, pinv
from tempfile import TemporaryFile
outfile_abcd = TemporaryFile()

def init():
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text


def animate(i):
    thisx = [0, x1[i], x2[i], x3[i], x4[i]]
    thisy = [0, y1[i], y2[i], y3[i], y4[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i * dt))
    return line, time_text


if __name__ == "__main__":
    # Linkage Length
    L = 40
    A, B, C, D = symbols(" A B C D")

    Xe = L * (cos(A) + cos(A + B) + cos(A + B + C) + cos(A + B + C + D))
    Ye = L * (sin(A) + sin(A + B) + sin(A + B + C) + sin(A + B + C + D))

    Xedot = np.array([-L * (sin(A) + sin(A + B) + sin(A + B + C) + sin(A + B + C + D)),
                      -L * (sin(A + B) + sin(A + B + C) + sin(A + B + C + D)),
                      -L * (sin(A + B + C) + sin(A + B + C + D)), -L * (sin(A + B + C + D))])
    Yedot = np.array([L * (cos(A) + cos(A + B) + cos(A + B + C) + cos(A + B + C + D)),
                      L * (cos(A + B) + cos(A + B + C) + cos(A + B + C + D)),
                      L * (cos(A + B + C) + cos(A + B + C + D)), L * (cos(A + B + C + D))])

    oridot = np.array([1, 1, 1, 1])

    # Jacobiean 1 and 2
    J11 = Xedot[0]
    J12 = Xedot[1]
    J13 = Xedot[2]
    J14 = Xedot[3]
    J15 = Yedot[0]
    J16 = Yedot[1]
    J17 = Yedot[2]
    J18 = Yedot[3]

    J2 = np.matrix(oridot)
    J2p = pinv(J2)

    # create a time array from 0..100 sampled at 0.05 second steps
    dt = 0.05
    td = 10
    Num = int(td / dt)
    # t = np.arange(0.0, td, dt)

    # Variables a,b,c,d are the A,B,C,D joint angles
    a = np.zeros(2*Num + 1)
    b = np.zeros(2*Num + 1)
    c = np.zeros(2*Num + 1)
    d = np.zeros(2*Num + 1)
    xe = np.zeros(2*Num)
    ye = np.zeros(2*Num)
    x1 = np.zeros(2*Num)
    y1 = np.zeros(2*Num)
    x2 = np.zeros(2*Num)
    y2 = np.zeros(2*Num)
    x3 = np.zeros(2*Num)
    y3 = np.zeros(2*Num)
    x4 = np.zeros(2*Num)
    y4 = np.zeros(2*Num)
    time = np.linspace(0.0, 2*td, 2*Num + 1)
    G1 = 1
    G2 = 1
    # Initial Conditions
    a[0] = 2.7
    b[0] = -0.55
    c[0] = -1.2
    d[0] = -1.2

    # Trajatory
    t = time
    xt = t + 100
    yt = 100 #+ np.zeros(Num)
    xt2 = -t+120
    yt2= 100 #+ np.zeros(Num)
    xtdot = 1
    xt2dot=-1
    ytdot = 0
    # rd=np.array([[xt],[yt]])
    # rdot = np.array([[xtdot],[ytdot]])
    rd1 = [xt, xt2]
    rd2 = yt
    rdot1 = [xtdot,xt2dot]
    rdot2 = ytdot
    j=0

    # print(rd1)
    I4 = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])

    for i in range(0, 2*Num):
        if i>Num:
            j=1
            k=i-Num
        else:
            k = i
        print(k)

        # Jacobiean value
        J11v = J11.subs([(A, a[i]), (B, b[i]), (C, c[i]), (D, d[i])])
        J12v = J12.subs([(A, a[i]), (B, b[i]), (C, c[i]), (D, d[i])])
        J13v = J13.subs([(A, a[i]), (B, b[i]), (C, c[i]), (D, d[i])])
        J14v = J14.subs([(A, a[i]), (B, b[i]), (C, c[i]), (D, d[i])])
        J15v = J15.subs([(A, a[i]), (B, b[i]), (C, c[i]), (D, d[i])])
        J16v = J16.subs([(A, a[i]), (B, b[i]), (C, c[i]), (D, d[i])])
        J17v = J17.subs([(A, a[i]), (B, b[i]), (C, c[i]), (D, d[i])])
        J18v = J18.subs([(A, a[i]), (B, b[i]), (C, c[i]), (D, d[i])])

        J1v = np.array([[J11v, J12v, J13v, J14v], [J15v, J16v, J17v, J18v]])
        # End effector location

        pos1 = Xe.subs([(A, a[i]), (B, b[i]), (C, c[i]), (D, d[i])])
        pos2 = Ye.subs([(A, a[i]), (B, b[i]), (C, c[i]), (D, d[i])])
        # print(pos1)
        # print(pos2)
        # posv=np.array([[pos1],[pos2]])
        oriv = a[i] + b[i] + c[i] + d[i]
        #print(pos1)
        # h1 and h2
        #print(rdot1[j])
        h11 = rdot1[j] + G1 * (rd1[j][k] - pos1)
        h12 = rdot2 + G1 * (rd2 - pos2)
        h1 = np.array([[h11], [h12]])

        h2 = G2 * (-oriv)
        J1test = np.array([[-120, -80, -80, -40], [40, 40, 0, 0]])

        # J1vp=np.transpose(J1v).dot(inv(J1v.dot(np.transpose(J1v))))
        # J1vp=(inv(J1v.dot(np.transpose(J1v))))

        thetadot = pinv(double(J1v)).dot(h1) + (I4 - pinv(double(J1v)).dot(J1v)).dot(J2p).dot(h2)
        # print(thetadot)
        #print(thetadot)
        a[i + 1] = a[i] + thetadot[0] * dt
        b[i + 1] = b[i] + thetadot[1] * dt
        c[i + 1] = c[i] + thetadot[2] * dt
        d[i + 1] = d[i] + thetadot[3] * dt

        # joint locations

        x1[i] = L * cos(a[i])
        y1[i] = L * sin(a[i])

        x2[i] = L * cos(a[i] + b[i]) + x1[i]
        y2[i] = L * sin(a[i] + b[i]) + y1[i]
        x3[i] = L * cos(a[i] + b[i] + c[i]) + x2[i]
        y3[i] = L * sin(a[i] + b[i] + c[i]) + y2[i]
        x4[i] = L * cos(a[i] + b[i] + c[i] + d[i]) + x3[i]
        y4[i] = L * sin(a[i] + b[i] + c[i] + d[i]) + y3[i]

        #  # print(y2)
        #   # print(x2)
        #   # print(y2)
    np.save('outfile_a',a)
    np.save('outfile_b',b)
    np.save('outfile_c',c)
    np.save('outfile_d',d)
    fig = plt.figure(1)
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-150, 150), ylim=(-150, 150))
    ax.set_aspect('equal')
    ax.grid()
    #
    line, = ax.plot([], [], 'o-', lw=2)
    time_template = 'time = %.1fs'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
    #
    # #  ani.save('slidercrank', fps=15)
    ani = animation.FuncAnimation(fig, animate, np.arange(1, 2*Num),
                                  interval=10, blit=True, init_func=init)
    #   print(time)
    plt.figure(2)
    print(len(time))
    print(len(a))
    plt.plot(time, a)
    plt.suptitle("output angle plot ")
    plt.xlabel("time")
    plt.ylabel("theta1 in rad")

    plt.figure(3)
    print(len(time))
    print(len(a))
    plt.plot(time, b)
    plt.suptitle("output angle plot ")
    plt.xlabel("time")
    plt.ylabel("theta2 in rad")

    plt.figure(4)
    print(len(time))
    print(len(a))
    plt.plot(time, c)
    plt.suptitle("output angle plot ")
    plt.xlabel("time")
    plt.ylabel("theta3 in rad")

    plt.figure(5)
    print(len(time))
    print(len(a))
    plt.plot(time, d)
    plt.suptitle("output angle plot ")
    plt.xlabel("time")
    plt.ylabel("theta4 in rad")
    #
    #
    #   plt.figure(3)
    #   plt.plot(time,x2)
    #   plt.suptitle("input distance")
    #   plt.xlabel("time")
    #   plt.ylabel("Input distance" )
    #
    #   plt.figure(4)
    #   plt.plot(time,xedot)
    #   plt.suptitle("input velocity")
    #   plt.xlabel("time")
    #   plt.ylabel("Input velocity" )
    #
    #
    plt.show()

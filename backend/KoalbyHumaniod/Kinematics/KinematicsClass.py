import math
import time

import numpy

from collections import namedtuple

"""
This module can be used to compute the forward and inverse kinematics for a chain of revolute joints.
It has been largerly inspired by the Matlab Robotics Toolbox.

"""


class Link(namedtuple('Link', ('theta', 'd', 'a', 'alpha', 'length'))):
    """ Link object as defined by the standard DH representation.

    UNITS FOR THETA AND ALPHA ARE IN RADIANS, NOT DEGREES!!!!!!!!!

    Due to floating point error, values that should be zero like numpy.cos(math.pi/2) will come out as a very very small nonzero number,
    this shouldn't matter too much as the number is basically zero

    This representation is based on the following information:
    :param float theta: angle about previous z from old x to new x
    :param float d: offset along previous z to the common normal
    :param float a: offset along previous x to the common normal
    :param float alpha: angle about common normal, from old z axis to new z axis

    .. note:: We are only considering revolute joint.

    Please refer to http://en.wikipedia.org/wiki/Denavit-Hartenberg_parameters for more details.

    """

    def get_transformation_matrix(self, theta):  # WORKS CORRECTLY
        """ Computes the homogeneous transformation matrix for this link """
        ct = numpy.cos(theta + self.theta)
        st = numpy.sin(theta + self.theta)
        ca = numpy.cos(self.alpha)
        sa = numpy.sin(self.alpha)

        return numpy.matrix(((ct, -st * ca, st * sa, self.a * ct),
                             (st, ct * ca, -ct * sa, self.a * st),
                             (0, sa, ca, self.d),
                             (0, 0, 0, 1)))

    def get_length(self):  # WORKS CORRECTLY
        """ Returns the length of the link """
        return self.length


# def inverse_kinematics_leg(p):  # CHANGE FROM STATIC ONCE IT WORKS MAYBE
#     """ Calculates the inverse kinematics of a vector of task space coordinates (p) and returns
#         the corresponding angle values as a vector of the same size (q) """
#     l1 = 17.29
#     l2 = 19.60
#     l3 = 3.50
#
#     l23 = 23.10
#
#     xc = p[0]
#     yc = p[1]
#     zc = p[2]
#
#     r = math.sqrt(xc**2 + yc**2)
#     s = zc - l1
#
#     d = xc/r
#     e = r/l23
#     f = (r**2 + s**2 - l2**2 - l3**2) / (2 * l2 * l3)
#
#     theta1 = math.atan2(math.sqrt(1 - d ** 2), d)
#     theta2 = math.atan2(math.sqrt(1 - e ** 2), e)
#     theta3 = math.atan2(math.sqrt(1 - f ** 2), f)
#
#     q = [theta1, theta2, theta3]
#
#     return q


class Chain(namedtuple('Chain', ('links', 'base', 'tool'))):
    """ Chain of Link that can be used to perform forward and inverse kinematics.

    :param list links: list of Link that compose the chain
    :param base: the base homogeneous transformation matrix
    :param tool: the end tool homogeneous transformation matrix

    """

    def __new__(cls, links, base=numpy.identity(4), tool=numpy.identity(4)):
        return super(Chain, cls).__new__(cls, links, base, tool)

    def forward_kinematics(self, q):  # WORKS CORRECTLY
        """ Computes the homogeneous transformation matrix of the end effector of the chain.

        :param vector q: vector of the joint angles (theta 1, theta 2, ..., theta n)

        """
        q = numpy.array(q).flatten()

        if len(q) != len(self.links):
            raise ValueError('q must contain as element as the number of links')

        tr = self.base.copy()

        l = []  # name this better

        for link, theta in zip(self.links, q):
            tr = tr * link.get_transformation_matrix(theta)

            l.append(tr)

        tr = tr * self.tool
        l.append(tr)
        return tr, numpy.asarray(l)

    def inverse_kinematics_leg(self, end_effector_matrix):
        """ Calculates the inverse kinematics of a vector of task space coordinates (p) and returns
            the corresponding angle values as a vector of the same size (q) """
        l1 = self.links[0].length
        l2 = self.links[1].length
        l3 = self.links[2].length

        l23 = l2 + l3
        # print(l23)

        p = get_task_space(end_effector_matrix)  # each direction with motors in zero position
        print(p)
        # p = [0.0000000000000000000000000001, 0.0000000000000000000000000001, 0.00000000000000000000000000001]

        xc = p[0]
        yc = p[1]
        zc = p[2]
        # print(p[0], p[1], p[2])

        # r2 = zc - l1
        # r3 = math.sqrt(r1**2+r2**2)
        # phi1 = math.acos((l3**2-l2**2-r3**2)/(-2*l2*r3))
        # phi2 = math.atan2(r2, r1)
        # phi3 = math.acos((r3**2-l2**2-l3**2)/(-2*l2*l3))
        # print(r)
        r = math.sqrt(xc ** 2 + yc ** 2)
        s = zc - l1
        d = xc / r
        e = (l2 ** 2 + r ** 2 + s ** 2 - l3 ** 2) / (2 * l2 * math.sqrt(r ** 2 + s ** 2))
        f = math.sqrt(1 - e ** 2)
        g = (l3 ** 2 + l2 ** 2 - (r ** 2 + s ** 2)) / (2 * l2 * l3)
        # print(r**2+s**2)
        # print(l3 ** 2 - l2 ** 2 - (r ** 2 + s ** 2))
        # print(2 * l2 * l3)
        # print(g)
        # print(1 - g ** 2)
        h = math.sqrt(1 - g ** 2)
        alpha = math.atan2(s, r)
        beta = math.atan2(e, f)
        theta1 = math.atan2(math.sqrt(1 - d ** 2), d)
        theta2 = -(alpha - beta)
        theta3 = -(math.atan2(h, g) - math.pi / 2)
        # print(d)
        # e = (2/3)*r / l2  # REDO MATH FOR THIS AND F  ratio of l23/l2 = 6/4 = 3/2  hypo = l2  horizontal = 2/3 r
        # print(e)
        # f = (xc ** 2 + yc ** 2 + (zc - l1) ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3)  # 16 + 4 - 4 - 4 / 2 * 2 * 2
        # f = (1/3)*r / l3
        # print(f)

        # theta1 = math.atan2(math.sqrt(1 - d ** 2), d)  # WORKS CORRECTLY
        # theta1 = math.atan2(yc, xc)
        # theta2 = phi2 - phi1
        # theta3 = math.pi - phi3
        # theta2 = math.atan2(math.sqrt(1 - e ** 2), e)
        # # print("Error Occurrence")
        # # print(f)
        # # print(math.atan2(1 - f ** 2, f))
        # theta3 = math.atan2(math.sqrt(1 - f ** 2), f)

        q = [theta1, theta2, theta3]

        return q

    # def inverse_kinematics(self, end_effector_transformation,
    #                        q=None,
    #                        max_iter=1000, tolerance=0.05,
    #                        mask=numpy.ones(6),
    #                        use_pinv=False):
    #     """ Computes the joint angles corresponding to the end effector transformation.
    #
    #     :param end_effector_transformation: the end effector homogeneous transformation matrix
    #     :param vector q: initial estimate of the joint angles
    #     :param int max_iter: maximum number of iteration
    #     :param float tolerance: tolerance before convergence
    #     :param mask: specify the cartesian DOF that will be ignored (in the case of a chain with less than 6 joints).
    #     :rtype: vector of the joint angles (theta 1, theta 2, ..., theta n)
    #
    #     """
    #     if q is None:
    #         q = numpy.zeros((len(self.links), 1))
    #     q = numpy.matrix(q.reshape(-1, 1))
    #
    #     best_e = numpy.ones(6) * numpy.inf
    #     best_q = None
    #     alpha = 1.0
    #
    #     for _ in range(max_iter):
    #         e = numpy.multiply(transform_difference(self.forward_kinematics(q)[0], end_effector_transformation), mask)
    #         d = numpy.linalg.norm(e)
    #         # print(d)
    #
    #         if d < numpy.linalg.norm(best_e):
    #             best_e = e.copy()
    #             best_q = q.copy()
    #             alpha *= 2.0 ** (1.0 / 8.0)
    #         else:
    #             q = best_q.copy()
    #             e = best_e.copy()
    #             alpha *= 0.5
    #
    #         if use_pinv:
    #             dq = numpy.linalg.pinv(self._jacob0(q)) * e.reshape((-1, 1))
    #         else:
    #             dq = self._jacob0(q).T * e.reshape((-1, 1))
    #         q += alpha * dq
    #
    #         # d = numpy.linalg.norm(dq)
    #         if d < tolerance:
    #             return q
    #
    #     else:
    #         raise ValueError('could not converge d={}'.format(numpy.linalg.norm(best_e)))

    def _jacob0(self, q):
        Jn = self._jacobn(q)
        Rn = rotation_from_transf(self.forward_kinematics(q)[0])

        return numpy.concatenate((numpy.concatenate((Rn, numpy.zeros((3, 3))), axis=1),
                                  numpy.concatenate((numpy.zeros((3, 3)), Rn), 1))) * Jn

    def _jacobn(self, q):
        q = numpy.array(q).flatten()
        U = self.tool.copy()
        J = numpy.matrix([[]] * 6)
        print(list(reversed(list(zip(self.links, q)))))
        # print(q)

        for link, theta in reversed(list(zip(self.links, q))):
            U = link.get_transformation_matrix(theta) * U

            d = numpy.matrix((-U[0, 0] * U[1, 3] + U[1, 0] * U[0, 3],
                              -U[0, 1] * U[1, 3] + U[1, 1] * U[0, 3],
                              -U[0, 2] * U[1, 3] + U[1, 2] * U[0, 3]))
            delta = U[2, 0:3]

            J = numpy.concatenate((numpy.concatenate((d, delta), axis=1).T, J), axis=1)

        return J


# MARK: - Utility functions

def get_task_space(end_effector_transformation):
    """ Pulls the task space position from the values in the first three rows of the last column
        (known as the p vector) for use in inverse kinematics """

    return end_effector_transformation[0:3, 3]


def transform_difference(t1, t2):
    t1 = numpy.array(t1)
    t2 = numpy.array(t2)

    return numpy.concatenate(((t2[0:3, 3] - t1[0:3, 3]).reshape(3),
                              0.5 * (numpy.cross(t1[0:3, 0], t2[0:3, 0]) +
                                     numpy.cross(t1[0:3, 1], t2[0:3, 1]) +
                                     numpy.cross(t1[0:3, 2], t2[0:3, 2])).reshape(3)))


def rotation_from_transf(tm):
    return tm[0:3, 0:3]


def translation_from_transf(tm):
    return numpy.array(tm[0:3, 3]).reshape(3)


def components_from_transf(tm):
    return rotation_from_transf(tm), translation_from_transf(tm)


def transf_from_components(R, T):
    return numpy.matrix(numpy.vstack((numpy.hstack((R, T.reshape(3, 1))),
                                      (0, 0, 0, 1))))


def transl(x, y, z):
    M = numpy.matrix(numpy.identity(4))
    M[0:3, 3] = numpy.matrix([x, y, z]).T
    return M


def trotx(theta):
    ct = numpy.cos(theta)
    st = numpy.sin(theta)

    R = numpy.matrix(((1, 0, 0),
                      (0, ct, -st),
                      (0, st, ct)))

    return transf_from_components(R, numpy.zeros(3))


def troty(theta):
    ct = numpy.cos(theta)
    st = numpy.sin(theta)

    R = numpy.matrix(((ct, 0, st),
                      (0, 1, 0),
                      (-st, 0, ct)))

    return transf_from_components(R, numpy.zeros(3))


def trotz(theta):
    ct = numpy.cos(theta)
    st = numpy.sin(theta)

    R = numpy.matrix(((ct, -st, 0),
                      (st, ct, 0),
                      (0, 0, 1)))

    return transf_from_components(R, numpy.zeros(3))

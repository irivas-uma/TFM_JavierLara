import math
import warnings
import numpy as np


eps = 1e-7



def check_axis_index(name, i):
    """Checks axis index.

    Parameters
    ----------
    name : str
        Name of the axis. Required for the error message.

    i : int from [0, 1, 2]
        Index of the axis (0: x, 1: y, 2: z)

    Raises
    ------
    ValueError
        If basis is invalid
    """
    if i not in [0, 1, 2]:
        raise ValueError("Axis index %s (%d) must be in [0, 1, 2]" % (name, i))
    


def check_quaternion(q, unit=True):
    """Input validation of quaternion representation.

    Parameters
    ----------
    q : array-like, shape (4,)
        Quaternion to represent rotation: (w, x, y, z)

    unit : bool, optional (default: True)
        Normalize the quaternion so that it is a unit quaternion

    Returns
    -------
    q : array-like, shape (4,)
        Validated quaternion to represent rotation: (w, x, y, z)

    Raises
    ------
    ValueError
        If input is invalid
    """
    q = np.asarray(q, dtype=np.float64)
    if q.ndim != 1 or q.shape[0] != 4:
        raise ValueError("Expected quaternion with shape (4,), got "
                         "array-like object with shape %s" % (q.shape,))
    if unit:
        return norm_vector(q)
    return q



def check_matrix(R, tolerance=1e-6, strict_check=True):
    r"""Input validation of a rotation matrix.

    We check whether R multiplied by its inverse is approximately the identity
    matrix

    .. math::

        \boldsymbol{R}\boldsymbol{R}^T = \boldsymbol{I}

    and whether the determinant is positive

    .. math::

        det(\boldsymbol{R}) > 0

    Parameters
    ----------
    R : array-like, shape (3, 3)
        Rotation matrix

    tolerance : float, optional (default: 1e-6)
        Tolerance threshold for checks. Default tolerance is the same as in
        assert_rotation_matrix(R).

    strict_check : bool, optional (default: True)
        Raise a ValueError if the rotation matrix is not numerically close
        enough to a real rotation matrix. Otherwise we print a warning.

    Returns
    -------
    R : array, shape (3, 3)
        Validated rotation matrix

    Raises
    ------
    ValueError
        If input is invalid
    """
    R = np.asarray(R, dtype=np.float64)
    if R.ndim != 2 or R.shape[0] != 3 or R.shape[1] != 3:
        raise ValueError("Expected rotation matrix with shape (3, 3), got "
                         "array-like object with shape %s" % (R.shape,))
    RRT = np.dot(R, R.T)
    if not np.allclose(RRT, np.eye(3), atol=tolerance):
        error_msg = ("Expected rotation matrix, but it failed the test "
                     "for inversion by transposition. np.dot(R, R.T) "
                     "gives %r" % RRT)
        if strict_check:
            raise ValueError(error_msg)
        warnings.warn(error_msg)
    R_det = np.linalg.det(R)
    if R_det < 0.0:
        error_msg = ("Expected rotation matrix, but it failed the test "
                     "for the determinant, which should be 1 but is %g; "
                     "that is, it probably represents a rotoreflection"
                     % R_det)
        if strict_check:
            raise ValueError(error_msg)
        warnings.warn(error_msg)
    return R



def norm_axis_angle(a):
    """Normalize axis-angle representation.

    Parameters
    ----------
    a : array-like, shape (4,)
        Axis of rotation and rotation angle: (x, y, z, angle)

    Returns
    -------
    a : array-like, shape (4,)
        Axis of rotation and rotation angle: (x, y, z, angle). The length
        of the axis vector is 1 and the angle is in [0, pi). No rotation
        is represented by [1, 0, 0, 0].
    """
    angle = a[3]
    norm = np.linalg.norm(a[:3])
    if angle == 0.0 or norm == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])

    res = np.empty(4)
    res[:3] = a[:3] / norm

    angle = norm_angle(angle)
    if angle < 0.0:
        angle *= -1.0
        res[:3] *= -1.0

    res[3] = angle

    return res



def norm_vector(v):
    """Normalize vector.

    Parameters
    ----------
    v : array-like, shape (n,)
        nd vector

    Returns
    -------
    u : array, shape (n,)
        nd unit vector with norm 1 or the zero vector
    """
    norm = np.linalg.norm(v)
    if norm == 0.0:
        return v

    return np.asarray(v) / norm



def norm_angle(a):
    """Normalize angle to (-pi, pi].

    Parameters
    ----------
    a : float or array-like, shape (n,)
        Angle(s) in radians

    Returns
    -------
    a_norm : float or array-like, shape (n,)
        Normalized angle(s) in radians
    """
    # Source of the solution: http://stackoverflow.com/a/32266181
    return -((np.pi - np.asarray(a)) % (2.0 * np.pi) - np.pi)



def axis_angle_from_quaternion(q):
    """Compute axis-angle from quaternion.

    This operation is called logarithmic map.

    We usually assume active rotations.

    Parameters
    ----------
    q : array-like, shape (4,)
        Unit quaternion to represent rotation: (w, x, y, z)

    Returns
    -------
    a : array-like, shape (4,)
        Axis of rotation and rotation angle: (x, y, z, angle). The angle is
        constrained to [0, pi) so that the mapping is unique.
    """
    q = check_quaternion(q)
    p = q[1:]
    p_norm = np.linalg.norm(p)

    if p_norm < np.finfo(float).eps:
        return np.array([1.0, 0.0, 0.0, 0.0])

    axis = p / p_norm
    angle = (2.0 * np.arccos(q[0]),)
    return norm_axis_angle(np.hstack((axis, angle)))



def matrix_from_quaternion(q):
    """Compute rotation matrix from quaternion.

    This typically results in an active rotation matrix.

    Parameters
    ----------
    q : array-like, shape (4,)
        Unit quaternion to represent rotation: (w, x, y, z)

    Returns
    -------
    R : array-like, shape (3, 3)
        Rotation matrix
    """
    q = check_quaternion(q)
    uq = norm_vector(q)
    w, x, y, z = uq
    x2 = 2.0 * x * x
    y2 = 2.0 * y * y
    z2 = 2.0 * z * z
    xy = 2.0 * x * y
    xz = 2.0 * x * z
    yz = 2.0 * y * z
    xw = 2.0 * x * w
    yw = 2.0 * y * w
    zw = 2.0 * z * w

    R = np.array([[1.0 - y2 - z2, xy - zw, xz + yw],
                  [xy + zw, 1.0 - x2 - z2, yz - xw],
                  [xz - yw, yz + xw, 1.0 - x2 - y2]])
    return R



def quaternion_from_matrix(R, strict_check=True):
    """Compute quaternion from rotation matrix.

    We usually assume active rotations.

    .. warning::

        When computing a quaternion from the rotation matrix there is a sign
        ambiguity: q and -q represent the same rotation.

    Parameters
    ----------
    R : array-like, shape (3, 3)
        Rotation matrix

    strict_check : bool, optional (default: True)
        Raise a ValueError if the rotation matrix is not numerically close
        enough to a real rotation matrix. Otherwise we print a warning.

    Returns
    -------
    q : array-like, shape (4,)
        Unit quaternion to represent rotation: (w, x, y, z)
    """
    R = check_matrix(R, strict_check=strict_check)
    q = np.empty(4)

    # Source:
    # http://www.euclideanspace.com/maths/geometry/rotations/conversions
    trace = np.trace(R)
    if trace > 0.0:
        sqrt_trace = np.sqrt(1.0 + trace)
        q[0] = 0.5 * sqrt_trace
        q[1] = 0.5 / sqrt_trace * (R[2, 1] - R[1, 2])
        q[2] = 0.5 / sqrt_trace * (R[0, 2] - R[2, 0])
        q[3] = 0.5 / sqrt_trace * (R[1, 0] - R[0, 1])
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            sqrt_trace = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[0] = 0.5 / sqrt_trace * (R[2, 1] - R[1, 2])
            q[1] = 0.5 * sqrt_trace
            q[2] = 0.5 / sqrt_trace * (R[1, 0] + R[0, 1])
            q[3] = 0.5 / sqrt_trace * (R[0, 2] + R[2, 0])
        elif R[1, 1] > R[2, 2]:
            sqrt_trace = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[0] = 0.5 / sqrt_trace * (R[0, 2] - R[2, 0])
            q[1] = 0.5 / sqrt_trace * (R[1, 0] + R[0, 1])
            q[2] = 0.5 * sqrt_trace
            q[3] = 0.5 / sqrt_trace * (R[2, 1] + R[1, 2])
        else:
            sqrt_trace = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[0] = 0.5 / sqrt_trace * (R[1, 0] - R[0, 1])
            q[1] = 0.5 / sqrt_trace * (R[0, 2] + R[2, 0])
            q[2] = 0.5 / sqrt_trace * (R[2, 1] + R[1, 2])
            q[3] = 0.5 * sqrt_trace
    return q



def euler_from_quaternion(q, i, j, k, extrinsic):
    """General method to extract any Euler angles from quaternions.

    Parameters
    ----------
    q : array-like, shape (4,)
        Unit quaternion to represent rotation: (w, x, y, z)

    i : int from [0, 1, 2]
        The first rotation axis (0: x, 1: y, 2: z)

    j : int from [0, 1, 2]
        The second rotation axis (0: x, 1: y, 2: z)

    k : int from [0, 1, 2]
        The third rotation axis (0: x, 1: y, 2: z)

    extrinsic : bool
        Do we use extrinsic transformations? Intrinsic otherwise.

    Returns
    -------
    euler_angles : array, shape (3,)
        Extracted rotation angles in radians about the axes i, j, k in this
        order. The first and last angle are normalized to [-pi, pi]. The middle
        angle is normalized to either [0, pi] (proper Euler angles) or
        [-pi/2, pi/2] (Cardan / Tait-Bryan angles).

    Raises
    ------
    ValueError
        If basis is invalid

    References
    ----------
    Bernardes, Evandro; Viollet, Stephane: Quaternion to Euler angles
    conversion: A direct, general and computationally efficient method,
    https://doi.org/10.1371/journal.pone.0276302
    """
    q = check_quaternion(q)

    check_axis_index("i", i)
    check_axis_index("j", j)
    check_axis_index("k", k)

    i += 1
    j += 1
    k += 1

    # The original algorithm assumes extrinsic convention. Hence, we swap
    # the order of axes for intrinsic rotation.
    if not extrinsic:
        i, k = k, i

    # Proper Euler angles rotate about the same axis in the first and last
    # rotation. If this is not the case, they are called Cardan or Tait-Bryan
    # angles and have to be handled differently.
    proper_euler = i == k
    if proper_euler:
        k = 6 - i - j

    sign = (i - j) * (j - k) * (k - i) // 2
    a = q[0]
    b = q[i]
    c = q[j]
    d = q[k] * sign

    if not proper_euler:
        a, b, c, d = a - c, b + d, c + a, d - b

    # Equation 34 is used instead of Equation 35 as atan2 it is numerically
    # more accurate than acos.
    angle_j = 2.0 * math.atan2(math.hypot(c, d),
                               math.hypot(a, b))

    # Check for singularities
    if abs(angle_j) <= eps:
        singularity = 1
    elif abs(angle_j - math.pi) <= eps:
        singularity = 2
    else:
        singularity = 0

    # Equation 25
    # (theta_1 + theta_3) / 2
    half_sum = math.atan2(b, a)
    # (theta_1 - theta_3) / 2
    half_diff = math.atan2(d, c)

    if singularity == 0:  # no singularity
        # Equation 32
        angle_i = half_sum + half_diff
        angle_k = half_sum - half_diff
    elif extrinsic:  # singularity
        angle_k = 0.0
        if singularity == 1:
            angle_i = 2.0 * half_sum
        else:
            assert singularity == 2
            angle_i = 2.0 * half_diff
    else:  # intrinsic, singularity
        angle_i = 0.0
        if singularity == 1:
            angle_k = 2.0 * half_sum
        else:
            assert singularity == 2
            angle_k = -2.0 * half_diff

    if not proper_euler:
        # Equation 43
        angle_j -= math.pi / 2.0
        # Equation 44
        angle_i *= sign

    angle_k = norm_angle(angle_k)
    angle_i = norm_angle(angle_i)

    if extrinsic:
        return np.array([angle_k, angle_j, angle_i])

    return np.array([angle_i, angle_j, angle_k])








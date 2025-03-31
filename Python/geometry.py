import copy
import numpy as np

# 20250321 Python 几何运算库

class Quat(object):
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):  # 顺序: w x y z
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.normalize()  # 构建默认进行单位化
    
    # 转换为四元数向量
    def vec(self):
        return np.array([self.w, self.x, self.y, self.z])
    
    # 从角轴形式构建四元数
    def setFromAngleAxis(self, angle, axis):
        self.w = np.cos(angle/2)
        self.x = np.sin(angle/2)*axis[0]
        self.y = np.sin(angle/2)*axis[1]
        self.z = np.sin(angle/2)*axis[2]
        self.normalize()
    
    # 从旋转矩阵构建四元数
    def setFromRotationMatrix(self, mat):
        # Copied from Eigen.
        # This algorithm comes from  "Quat Calculus and Fast Animation",
        # Ken Shoemake, 1987 SIGGRAPH course notes
        assert mat.shape == (3, 3)
        t = np.trace(mat)
        if t > 0:
            t = np.sqrt(t + 1.0)
            self.w = 0.5*t
            t = 0.5 / t
            self.x = (mat[2, 1] - mat[1, 2]) * t
            self.y = (mat[0, 2] - mat[2, 0]) * t
            self.z = (mat[1, 0] - mat[0, 1]) * t
        else:
            i = 0
            if (mat[1, 1] > mat[0, 0]):
                i = 1
            if (mat[2, 2] > mat[i, i]):
                i = 2
            j = (i+1) % 3
            k = (j+1) % 3
            t = np.sqrt(mat[i, i] - mat[j, j] - mat[k, k] + 1.0)
            if i == 0:
                self.x = 0.5 * t
            elif i == 1:
                self.y = 0.5 * t
            elif i == 2:
                self.z = 0.5 * t
            elif i == 3:
                self.w = 0.5 * t
            t = 0.5 / t
            self.w = (mat[k, j] - mat[j, k])*t
            if j == 0:
                self.x = (mat[j, i] + mat[i, j])*t
            elif j == 1:
                self.y = (mat[j, i] + mat[i, j])*t
            elif j == 2:
                self.z = (mat[j, i] + mat[i, j])*t
            if k == 0:
                self.x = (mat[k, i] + mat[i, k])*t
            elif k == 1:
                self.y = (mat[k, i] + mat[i, k])*t
            elif k == 2:
                self.z = (mat[k, i] + mat[i, k])*t
        self.normalize()
        return self

    # 四元数计算模值
    def norm(self):
        return np.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)
    
    # 四元数单位化
    def normalize(self):
        n = self.norm()
        self.w /= n
        self.x /= n
        self.y /= n
        self.z /= n
        return self

    # 从四元数构建旋转矩阵
    def getRotationMatrix(self):
        # Copied from Eigen
        tx  = 2.0*self.x
        ty  = 2.0*self.y
        tz  = 2.0*self.z
        twx = tx*self.w
        twy = ty*self.w
        twz = tz*self.w
        txx = tx*self.x
        txy = ty*self.x
        txz = tz*self.x
        tyy = ty*self.y
        tyz = tz*self.y
        tzz = tz*self.z

        R = np.zeros((3, 3))
        R[0, 0] = 1.0-(tyy+tzz)
        R[0, 1] = txy-twz
        R[0, 2] = txz+twy
        R[1, 0] = txy+twz
        R[1, 1] = 1.0-(txx+tzz)
        R[1, 2] = tyz-twx
        R[2, 0] = txz-twy
        R[2, 1] = tyz+twx
        R[2, 2] = 1.0-(txx+tyy)
        return R

    # Spherical linearl interpolation (from Eigen)
    def slerp(self, t, other):
        vec0 = self.vec()
        vec1 = other.vec()

        thresh = float(1.0)  - np.spacing(1.0)
        d = np.dot(vec0, vec1)
        abs_d = np.abs(d)

        scale0 = 0
        scale1 = 0

        if abs_d >= thresh:
            scale0 = 1.0 - t
            scale1 = t
        else:
            theta = np.arccos(abs_d)
            sin_theta = np.sin(theta)

            scale0 = np.sin((1.0 - t ) * theta)/sin_theta;
            scale1 = np.sin(( t * theta))/sin_theta;

        if d < 0:
            scale1 = -scale1

        new_vec =  scale0 * vec0 + scale1 * vec1
        q = Quat(new_vec[0], new_vec[1], new_vec[2], new_vec[3])
        return q.normalize()

    # For print
    def __str__(self):
        return "[%.5f, %.5f, %.5f, %.5f]" %(self.w, self.x, self.y, self.z)

# 四元数乘积
def quaternion_multiply(a, b):
    p_w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    p_x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
    p_y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
    p_z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    return Quat(p_w, p_x, p_y, p_z)

# 四元数缩放
def quaternion_scale(q, s):
    return Quat(q.w*s, q.x*s, q.y*s, q.z*s)

# 四元数转置
def quaternion_conjugate(q):
    return Quat(q.w, -q.x, -q.y, -q.z)

# 四元数对向量进行旋转
def quaternion_rotate(q, v):
    v_quat = Quat(0, v[0], v[1], v[2])
    q_conj = quaternion_conjugate(q)
    _v_quat = quaternion_multiply(q, v_quat)
    __v_quat = quaternion_multiply(_v_quat, q_conj)
    _v = np.array([__v_quat.x, __v_quat.y, __v_quat.z])
    return _v

# 四元数对向量进行旋转
def quaternion_rotate_inv(q, v):
    v_quat = Quat(0, v[0], v[1], v[2])
    q_conj = quaternion_conjugate(q)
    _v_quat = quaternion_multiply(q_conj, v_quat)
    __v_quat = quaternion_multiply(_v_quat, q)
    _v = np.array([__v_quat.x, __v_quat.y, __v_quat.z])
    return _v

# 从旋转向量构建四元数
def quaternion_from_rotation_vector(v):
    angle = np.linalg.norm(v)
    if angle < 1e-6:  # 角度过小 返回单位四元数
        return Quat(1.0, 0.0, 0.0, 0.0)
    axis = v/angle
    quat = Quat(1.0, 0.0, 0.0, 0.0)
    quat.setFromAngleAxis(angle, axis)
    return quat

# 四元数转换为欧拉角 rad
def quaternion_to_euler(q):
    roll = np.arctan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = np.arcsin(2.0 * (q.w * q.y - q.z * q.x))
    yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return np.array([roll, pitch, yaw])

# 欧拉角转换为旋转矩阵
def rotation_matrix_from_angles(roll, pitch, yaw):
    cosx = np.cos(roll)
    sinx = np.sin(roll)
    cosy = np.cos(pitch)
    siny = np.sin(pitch)
    cosz = np.cos(yaw)
    sinz = np.sin(yaw)
    coszcosx = cosz * cosx
    sinzcosx = sinz * cosx
    coszsinx = sinx * cosz
    sinzsinx = sinx * sinz
    rmat = np.array([
        [cosz * cosy, -cosy * sinz, siny],
        [sinzcosx + coszsinx * siny, coszcosx - sinzsinx * siny, -sinx * cosy],
        [sinzsinx - coszcosx * siny, coszsinx + sinzcosx * siny, cosy * cosx]
    ])
    return rmat

# 反对称矩阵转换为so3向量
# Extracts an R3 vector from an so3 (Lie Algebra) skew-symmetric matrix
def veemap(A):
    assert(A.shape == (3, 3))
    ret = np.zeros(3)
    ret[0] = A[2, 1]
    ret[1] = A[0, 2]
    ret[2] = A[1, 0]
    return ret

# so3向量转换为反对称矩阵
# Maps an R3 vector to an so3 (Lie Algebra) skew-symmetric matrix
def hatmap(w):
    assert(len(w) == 3)
    ret = np.zeros((3, 3))
    ret[1, 0] = w[2]
    ret[2, 0] = -w[1]
    ret[0, 1] = -w[2]
    ret[2, 1] = w[0]
    ret[0, 2] = w[1]
    ret[1, 2] = -w[0]
    return ret

# so3向量转换为反对称矩阵
# Convert a 3 vector to a skew-symmetric matrix
def skewSym(w):
    assert(len(w) == 3)
    ret = np.zeros((3, 3))
    ret[1, 0] = w[2]
    ret[2, 0] = -w[1]
    ret[0, 1] = -w[2]
    ret[2, 1] = w[0]
    ret[0, 2] = w[1]
    ret[1, 2] = -w[0]
    return ret

# so3向量转换为旋转矩阵
# Converts Lie algebra in SO3 to rotation matrix using
# exponential map (i.e. Rodrigues formula).
# http://ethaneade.com/latex2html/lie/node16.html
def so3LieToMat(w):
    assert(len(w) == 3)
    theta = np.linalg.norm(w)  # Get Angle
    if theta > 1e-3:
        A = np.sin(theta)/theta
        B = (1 - np.cos(theta))/(theta**2)
    else:
        A = 1
        B = 0.5
    wx = skewSym(w)  # 转换为反对称矩阵
    # Rodrigues formula
    R = np.eye(3)
    R += A*wx + B*np.dot(wx, wx)
    return R

# se3六维向量转换为R t
# Converts Lie algebra in SE3 (rotation params in w, translation
# params in u) to Rotation matrix and translation vector using
# exponential map.
# See http://ethaneade.com/latex2html/lie/node16.html
def se3LieToRotTrans3(w, u):
    assert(len(w) == 3)
    assert(len(u) == 3)
    # First get rotation matrix.
    theta = np.linalg.norm(w)
    if theta > 1e-3:
        A = np.sin(theta)/theta
        B = (1 - np.cos(theta))/(theta**2)
    else:
        A = 1
        B = 0.5
    wx = skewSym(w)
    # Rodrigues formula
    R = np.eye(3)
    R += A*wx + B*np.dot(wx, wx)
    # Now get translation vector
    if theta > 1e-3:
        C = (1 - A)/(theta**2)
    else:
        C = 1.0/6
    V = np.eye(3)
    V += B*wx + C*np.dot(wx, wx)
    t = np.dot(V, u)
    return R, t

if __name__ == '__main__':
    print("Quaternion Test")
    # _quat = Quat(1.0, 0.0, 0.0, 0.0)
    # _angle = 0.5*np.pi
    # _axis = np.array([0.0, 0.0, 1.0])
    # _vector = _angle * _axis
    # _quat.setFromAngleAxis(_angle, _axis)
    # print(_quat)
    # rotate_quat = quaternion_from_rotation_vector(_vector)  
    # print(rotate_quat)
    # x_axis = np.array([1.0, 0.0, 0.0])
    # x_rot = quaternion_rotate(_quat, x_axis)
    # print(x_rot)

    # q = Quat(0.7071, 0.7071, 0, 0)  # 示例四元数
    # euler = quaternion_to_euler(q)
    # print(f"Roll: {euler[0]} rad, Pitch: {euler[1]} rad, Yaw: {euler[2]} rad")

    # 示例用法
    roll = np.radians(0)  # 将角度转换为弧度
    pitch = np.radians(0)  # 将角度转换为弧度
    yaw = np.radians(0)  # 将角度转换为弧度

    rotation_matrix = rotation_matrix_from_angles(roll, pitch, yaw)
    print("rotation matrix:")
    print(rotation_matrix)
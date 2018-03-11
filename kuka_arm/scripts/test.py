from sympy import *

def dh_trans_unit(p, a, d, q): #p as alpha, q as theta
    ret = Matrix([
        [       cos(q),       -sin(q),      0,          a], 
        [sin(q)*cos(p), cos(q)*cos(p), -sin(p), -sin(p)*d],
        [sin(q)*sin(p), cos(q)*sin(p),  cos(p),  cos(p)*d],
        [            0,             0,       0,         1]])
    
    return ret 


q1, q2, q3, q4, q5, q6 = symbols('q1:7')

def dh_build_R3_6():

    # DH Table from urdf
    # alpha[i-1], a[i-1], d[i], theta[i]
    dh_params = Matrix([
            [-pi/2, -0.054, 1.5,    q4],
            [pi/2, 0,       0,      q5],
            [-pi/2, 0,      0,      q6],
    ])  
    ret = eye(4)
    for i in range(0, dh_params.rows):
        trans_unit = dh_trans_unit(dh_params[i,0], dh_params[i,1], dh_params[i,2], dh_params[i,3])
        ret = ret * trans_unit

    ret = simplify(ret)
    print(ret)
#R3_6 = Matrix([
#    [-sin(q4)*cos(q6)+cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6)-cos(q4)*cos(q5)*sin(q6), -cos(q4)*sin(q5)],
#    [sin(q5)*cos(q6),                          -sin(q5)*sin(q6),                         cos(q5)],
#    [-sin(q4)*cos(q5)*cos(q6)-cos(q4)*sin(q6), sin(q4)*cos(q5)*sin(q6)-cos(q4)*cos(q6),  sin(q4)*sin(q5)]
#])

dh_build_R3_6()

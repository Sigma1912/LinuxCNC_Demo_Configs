#!/usr/bin/env python3

from vtk_vismach import *
import hal
import sys

#for setting in sys.argv[1:]: exec(setting)
options = sys.argv[1:]


comp_name = 'vtk-dh-simulator'

if not hal.component_exists(comp_name):
    # create a hal component for this model
    c = hal.component(comp_name)
    c.newpin("a1", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a2", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a3", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a4", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a5", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a6", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha1", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha2", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha3", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha4", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha5", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha6", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d1", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d2", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d3", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d4", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d5", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d6", hal.HAL_S32, hal.HAL_IN)
    c.ready()

#guivars
myvars = []
myvars.append(["a1", -1000, 1000,   85])
myvars.append(["alpha1", -90, 90,  -90])
myvars.append(["d1", -1000, 1000,  350])

myvars.append(["a2", -1000, 1000, 380])
myvars.append(["alpha2", -90, 90,   0])
myvars.append(["d2", -1000, 1000,   0])

myvars.append(["a3", -1000, 1000, 100])
myvars.append(["alpha3", -90, 90, -90])
myvars.append(["d3", -1000, 1000,   0])

myvars.append(["a4", -1000, 1000,   0])
myvars.append(["alpha4", -90, 90,  90])
myvars.append(["d4", -1000, 1000, 425])

myvars.append(["a5", -1000, 1000,   0])
myvars.append(["alpha5", -90, 90, -90])
myvars.append(["d5", -1000, 1000,   0])

myvars.append(["a6", -1000, 1000,   0])
myvars.append(["alpha6", -90, 90,   0])
myvars.append(["d6", -1000, 1000, 120])


def matrix_multiplication(A, B):
    # Determine the matrices' dimensions.
    rows_A = len(A)
    cols_A = len(A[0])
    rows_B = len(B)
    cols_B = len(B[0])
    # Set the result matrix to zeroes.
    result_matrix = [['(0' for row in range(cols_B)] for col in range(rows_A)]
    # Iterate through rows of A
    for s in range(rows_A):
        # Iterate through columns of B
        for j in range(cols_B):
            # Iterate through rows of B
            for k in range(cols_A):
                product = None
                # check if one factor is zero (because we are using strings we also need to check for '-0')
                if str(A[s][k]) == '0' or str(B[k][j]) == '0' or str(A[s][k]) == '-0' or str(B[k][j]) == '-0' :
                    product = '0'
                # 1 * 1 = 1
                elif str(A[s][k]) == '1' and str(B[k][j]) == '1':
                    product = '1'
                # 1 * b = b
                elif str(A[s][k]) == '1' and str(B[k][j]) != '1':
                    product = str(B[k][j])
                # a * 1 = a
                elif str(A[s][k]) != '1' and str(B[k][j]) == '1':
                    product = str(A[s][k])
                else:
                    product = (str(A[s][k]) + '*' + str(B[k][j]))
                # build sum
                if product != '0':
                    if result_matrix[s][j] == '(0':
                        result_matrix[s][j] = ( '(' +  product)
                    else:
                        result_matrix[s][j] += ( '+' +  product)
                if k == cols_A-1:
                    result_matrix[s][j] += ')'

    return result_matrix




T1 = [[1,          0,                              0,              '{a1}'],
      [0, 'cos(radians({alpha1}))', '-sin(radians({alpha1}))',       0 ],
      [0, 'sin(radians({alpha1}))', ' cos(radians({alpha1}))',     '{d1}'],
      [0,          0,                              0,                1 ]]

T2 = [[1,          0,                              0,              '{a2}'],
      [0, 'cos(radians({alpha2}))', '-sin(radians({alpha2}))',       0 ],
      [0, 'sin(radians({alpha2}))', ' cos(radians({alpha2}))',     '{d2}'],
      [0,          0,                              0,                1 ]]

T3 = [[1,          0,                              0,              '{a3}'],
      [0, 'cos(radians({alpha3}))', '-sin(radians({alpha3}))',       0 ],
      [0, 'sin(radians({alpha3}))', ' cos(radians({alpha3}))',     '{d3}'],
      [0,          0,                              0,                1 ]]

T4 = [[1,          0,                              0,              '{a4}'],
      [0, 'cos(radians({alpha4}))', '-sin(radians({alpha4}))',       0 ],
      [0, 'sin(radians({alpha4}))', ' cos(radians({alpha4}))',     '{d4}'],
      [0,          0,                              0,                1 ]]

T5 = [[1,          0,                              0,              '{a5}'],
      [0, 'cos(radians({alpha5}))', '-sin(radians({alpha5}))',       0 ],
      [0, 'sin(radians({alpha5}))', ' cos(radians({alpha5}))',     '{d5}'],
      [0,          0,                              0,                1 ]]

T6 = [[1,          0,                              0,              '{a6}'],
      [0, 'cos(radians({alpha6}))', '-sin(radians({alpha6}))',       0 ],
      [0, 'sin(radians({alpha6}))', ' cos(radians({alpha6}))',     '{d6}'],
      [0,          0,                              0,                1 ]]


col = 'Gainsboro'   # joint/link color
alpha = 0.8         # joint/link color alpha
d_joint = 40/2      # diameter joint cylinder
d_link = 30/2       # diameter link cylinder

j0 = Collection([Axes(150),
                 Translate([ Color([ CylinderZ(50, d_joint) ], col, alpha ) ],0,0,-25),
                 Color([ CylinderZ(c, 'd1', d_link) ], col, alpha),
                 Translate([ Color([ CylinderX(c, 'a1', d_link) ], col, alpha) ],c,0,0,'d1')
                 ])




col = 'BurlyWood'   # joint/link color
j1 = Collection([Axes(80),
                 Translate([ Color([ CylinderZ(50, d_joint) ], col, alpha ) ],0,0,-25),
                 Color([ CylinderZ(c, 'd2', d_link) ], col, alpha),
                 Translate([ Color([ CylinderX(c, 'a2', d_link) ], col, alpha) ],c,0,0,'d2')
                 ])

j1 = MatrixTransform([ j1 ],
                      c,
                      *( T1[0][0], T1[1][0], T1[2][0] ),
                      *( T1[0][2], T1[1][2], T1[2][2] ),
                      *( T1[0][3], T1[1][3], T1[2][3] ) )




T12 = matrix_multiplication(T1, T2)

j2 = Collection([Axes(80),
                 Translate([ Color([ CylinderZ(50, 20) ],'DarkOrange',0.5) ],0,0,-25)
                 ])

j2 = MatrixTransform([ j2 ],
                      c,
                      *( T12[0][0], T12[1][0], T12[2][0] ),
                      *( T12[0][2], T12[1][2], T12[2][2] ),
                      *( T12[0][3], T12[1][3], T12[2][3] ) )




T13 = matrix_multiplication(T12, T3)
j3 = Collection([Axes(80),
                 Translate([ Color([ CylinderZ(50, 20) ],'Yellow',0.5) ],0,0,-25)
                 ])
vx = ( T13[0][0], T13[1][0], T13[2][0] )
vz = ( T13[0][2], T13[1][2], T13[2][2] )
vo = ( T13[0][3], T13[1][3], T13[2][3] )
j3 = MatrixTransform([ j3 ], c, *vx, *vz, *vo )



T14 = matrix_multiplication(T13, T4)

j4 = Collection([Axes(80),
                 Translate([ Color([ CylinderZ(50, 20) ],'Green',0.5) ],0,0,-25)
                 ])
vx = ( T14[0][0], T14[1][0], T14[2][0] )
vz = ( T14[0][2], T14[1][2], T14[2][2] )
vo = ( T14[0][3], T14[1][3], T14[2][3] )
j4 = MatrixTransform([ j4 ], c, *vx, *vz, *vo )



T15 = matrix_multiplication(T14, T5)

j5 = Collection([Axes(80),
                 Translate([ Color([ CylinderZ(50, 20) ],'Teal',0.5) ],0,0,-25)
                 ])
vx = ( T15[0][0], T15[1][0], T15[2][0] )
vz = ( T15[0][2], T15[1][2], T15[2][2] )
vo = ( T15[0][3], T15[1][3], T15[2][3] )
j5 = MatrixTransform([ j5 ], c, *vx, *vz, *vo )



T16 = matrix_multiplication(T15, T6)

hand = Collection([Axes(120),
                 #Translate([ Color([ CylinderZ(50, 20) ],'Violet',1) ],0,0,-25)
                 ])
vx = ( T16[0][0], T16[1][0], T16[2][0] )
vz = ( T16[0][2], T16[1][2], T16[2][2] )
vo = ( T16[0][3], T16[1][3], T16[2][3] )
hand = MatrixTransform([ hand ], c, *vx, *vz, *vo )



# Create machine base
base = Color([Grid(200,10)],1,0,1,0.3)
model = Collection([
        base,
        j0,
        j1,
        j2,
        j3,
        j4,
        j5,
        hand
        ])
#hud
myhud = Hud(color='mint',opacity=0.4) # This will always be displayed
myhud.add_txt('DH-Simulator')
myhud.add_txt('------ -----')


main(options,
     comp_name,
     model,
     huds = myhud,
     guivars = myvars,
     window_width = 1400,
     window_height = 1000,
     window_title = 'DH-Simulator')
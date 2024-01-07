# please select a model index
index = 0
model_names = ['solarPlant','hoaHakanaia', 'bigBen', 'church']
model_name = model_names[index]
print('selected model is: ', model_name)

# model params
if model_name == 'bigBen':
    INCIDENCE_ANGLE = 30.0
    DISTANCE_MIN = 10.0
    DISTANCE_MAX = 30.0
    DISTANCE_SHARPNESS = 30.0
    FOV = 169                   # h,v 120
    Z_MIN = -45.0
    start_point = [25.0, 25.0, Z_MIN, 0.0, 0.0, 0.0]
    end_point = [25.0, 25.0, Z_MIN, 0.0, 0.0, 0.0]
    mesh_folder = 'meshes/compare/mesh_ccpp'
    filename = 'bigBen.stl'

elif model_name == 'hoaHakanaia':
    INCIDENCE_ANGLE = 25.0
    DISTANCE_MIN = 4.0          # 4
    DISTANCE_MAX = 12.0         # 12
    DISTANCE_SHARPNESS = 20.0   # 20
    FOV = 120                   # h:108.4, v:92.2
    Z_MIN = -10
    start_point = [15.0, 15.0, Z_MIN, 0.0, 0.0, 0.0]
    end_point = [15.0, 15.0, Z_MIN, 0.0, 0.0, 0.0]
    mesh_folder = 'meshes/compare/mesh_ccpp'
    filename = 'hoaHakanaia.stl'

elif model_name == 'solarPlant':
    INCIDENCE_ANGLE = 30.0     # 30.0
    DISTANCE_MIN = 5.0         # 5.0
    DISTANCE_MAX = 10.0        # 10.0
    DISTANCE_SHARPNESS = 15.0  # 15.0
    FOV = 151                  # h:120, v:90
    Z_MIN = 3.0
    start_point = [-35.0, 90.0, Z_MIN, 0.0, 0.0, 0.0]
    end_point = [-35.0, 90.0, Z_MIN, 0.0, 0.0, 0.0]
    mesh_folder = 'meshes/compare/mesh_ccpp'
    filename = 'solarPlant.stl'

GIMBAL_PITCH_UPPER_LIMIT = 30
GIMBAL_PITCH_LOWER_LIMIT = -90

# fitness params
DIS_TOLERANCE = 1.0
w_vq1 = 0.5           # dis
w_vq2 = 1 - w_vq1     # ang
w_vq = 0.5
w_dis = round(1 - w_vq, 2)

# population params
pop_size = 30  # pop >= 2
iter_num = 30  # iter >= 1


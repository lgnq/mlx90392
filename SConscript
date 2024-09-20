from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add mlx90392 src files.
if GetDepend('PKG_USING_MLX90392'):
    src += Glob('src/mlx90392.c')

if GetDepend('PKG_MLX90392_USING_SENSOR_V1'):
    src += Glob('src/melexis_mlx90392_sensor_v1.c')

if GetDepend('PKG_USING_MLX90392_SAMPLE'):
    src += Glob('examples/mlx90392_sample.c')

# add mlx90393 include path.
path  = [cwd + '/inc']

# add src and include to group.
group = DefineGroup('mlx90392', src, depend = ['PKG_USING_MLX90392'], CPPPATH = path)

Return('group')

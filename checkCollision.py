import vtk

def get_vtk_meshes(meshfiles):
    vtk_meshs = [0]*len(meshfiles)
    for i in range(len(meshfiles)):
        f = meshfiles[i]
        if f.endswith('.stl'):
            readMesh = vtk.vtkSTLReader()
        else:
            readMesh = vtk.vtkPLYReader()
        readMesh.SetFileName(f)
        readMesh.Update()
        polydata = readMesh.GetOutput()
        if polydata.GetNumberOfPoints() == 0:
            raise ValueError(
                "No point data could be loaded from '" + f
            )
            vtk_meshs[i] = None
        else:
            vtk_meshs[i] = polydata
    return vtk_meshs

# the meshs must be list of vtk meshes
def check_collisions_with_ObbTree_wp(meshs, wp, rad=1.0):
    checkers = [0]*len(meshs)
    for i in range(len(meshs)):
        if meshs[i] is not None:
            obbTree = vtk.vtkOBBTree()
            obbTree.SetDataSet(meshs[i])
            obbTree.BuildLocator()  # create obb tree
            obbTree.SetMaxLevel(1)
            # wp collision detector
            checker_ = obbTree.InsideOrOutside((wp[0], wp[1], wp[2]))

            intersection = vtk.vtkPoints()
            cellIds = vtk.vtkIdList()
            checker_x_p = obbTree.IntersectWithLine((wp[0], wp[1], wp[2]), (wp[0]+rad, wp[1], wp[2]), intersection,
                                                cellIds)
            checker_x_n = obbTree.IntersectWithLine((wp[0], wp[1], wp[2]), (wp[0]-rad, wp[1], wp[2]), intersection,
                                                cellIds)
            checker_y_p = obbTree.IntersectWithLine((wp[0], wp[1], wp[2]), (wp[0], wp[1]+rad, wp[2]), intersection,
                                                cellIds)
            checker_y_n = obbTree.IntersectWithLine((wp[0], wp[1], wp[2]), (wp[0], wp[1]-rad, wp[2]), intersection,
                                                cellIds)
            checker_z_p = obbTree.IntersectWithLine((wp[0], wp[1], wp[2]), (wp[0], wp[1], wp[2]+rad), intersection,
                                                cellIds)
            checker_z_n = obbTree.IntersectWithLine((wp[0], wp[1], wp[2]), (wp[0], wp[1], wp[2]-rad), intersection,
                                                cellIds)

            checkers_ = [checker_, checker_x_p, checker_y_p, checker_z_p, checker_x_n, checker_y_n, checker_z_n]
            #print(checkers_)
            checker = 0
            if -1 in checkers_:
                checker = -1
            checkers[i] = checker
    if -1 in checkers:
        coll = True
    else:
        coll = False
    return coll, checkers
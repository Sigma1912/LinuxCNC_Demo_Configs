# This is a modified version of 'vismach.py' to visualize Tilted Work Plane (TWP)
# Author: David Mueller 2025
# email: mueller_david@hotmail.com

import hal, signal
import vtk
import os

from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt5 import Qt, QtWidgets
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QTimer


class ArgsBase(object):
    def __init__(self, *args):
        self.stored_scale = None
        self.group = None
        if isinstance(args[0], list): # an object manipulator is being created (ie the first argument is [parts])
            has_parts = True # used to adjust number of expected arguments
            parts = args[0]
            args = args[1:]
            self.SetUserTransform(vtk.vtkTransform())
            # Collect parts
            for part in parts:
                self.AddPart(part)
                if hasattr(part, 'tracked_parts'):
                    if not hasattr(self, 'tracked_parts'):
                        self.tracked_parts = []
                    self.tracked_parts += part.tracked_parts
        else: # an object creator is being created (ie the first argument is NOT [parts])
            has_parts = False # used to adjust the number of expected arguments
        # parse args
        if args and (isinstance(args[0], hal.component) or isinstance(args[0],type(hal))): #halpin passed
            self.comp = args[0]
            args = args[1:]
            self.needs_updates = True
        else:  # no halpin passed
            self.comp = None
            self.needs_updates = False
        # check number of arguments against expected number, need to adjust for '[parts]' and '(comp)'
        args_count = len(args) + has_parts + 1
        if hasattr(self, 'get_expected_args'):
            args_expected = self.get_expected_args()
            # if a class accepts more than one combination of arguments it will return them in a list
            if not isinstance(args_expected,list):
                args_expected = [args_expected]
            # check if number of passed args match any of the possibilities returned
            res = [args_count == len(a) for a in args_expected]
            if not any(res):
                # if none match we raise an error
                raise ValueError('Expected arguments are', self.get_expected_args())
        # store parsed args
        self._coords = args
        # prepare so at least the first update is run as instances with static values are not updated later
        self.first_update = True
        if hasattr(self, 'create'):
            self.create()
        # We cannot wait for the 1. update cycle because camera needs something to set the view on startup
        if hasattr(self, 'update'):
            self.update()

    def coords(self):
        if len(self._coords) == 1: # 'self._coords' is set in 'parse_arguments() it's args w/o comp'
            return list(map(self._coord, self._coords))[0] # for a single argument
        return list(map(self._coord, self._coords))

    def _coord(self, v):
        s = 1 # default scale factor
        if isinstance(v,tuple):
            # tuple syntax has been used, ie (<halpin_name>, scalefactor)
            tup = v
            v = tup[0]
            s = tup[1]
        if isinstance(v, str) and isinstance(self.comp, hal.component):
            # comp = 'c' passed (ie string value might be a local halpin name)
            if os.path.isdir(v): # Needed for ReadPolyData()
                # string is a path (eg for ReadPolyData())
                return v
            try:
                # if the string is a local halpin name we will get a nummeric value that can be scaled
                return s*self.comp[v]
            except Exception as e:
                # if that fails we return the string as we got it (eg 'x' for Axes())
                return v
        elif isinstance(v, str) and isinstance(self.comp,type(hal)):
            # comp = 'hal' passed (ie string value might be a global halpin name)
            if os.path.isdir(v):
                # string is a path (eg for ReadPolyData())
                return v
            try:
                # if the string is a global halpin name we will get a nummeric value that can be scaled
                return s*hal.get_value(v)
            except Exception as e:
                # if that fails we return the string as we got it (eg 'x' for Axes())
                return v
        else:
            # no comp passed (ie none of the values are halpin names)
            if isinstance(v,str) or v == None:
                # eg a string filename from 'ReadPolyData()' or None for Color() to set opacity
                return v
            # contant nummeric value
            return s*v

    def capture(self):
        if hasattr(self, 'tracked_parts'):
            if hasattr(self, 'transformation'):
                for tracked_part in self.tracked_parts:
                    tracked_part.GetUserTransform().Concatenate(self.transformation)

    def set_group(self, group):
        self.group = group
        # we need to return self so we can call 'set_group()' and initialize the class in one line
        return self

    def store_scale(self):
        if self.GetScale() != (0,0,0):
            self.stored_scale = self.GetScale()

    def restore_scale(self):
        if self.stored_scale:
            self.SetScale(*self.stored_scale)


# Creates a box centered on the origin
# Either specify the width in X and Y, and the height in Z
# or specify the two points across the diagonal
class Box(ArgsBase, vtk.vtkActor):
    def get_expected_args(self):
        return [('(comp)','x1', 'y1', 'z1', 'x2', 'y2', 'z2'),('(comp)','xw', 'yw', 'zw')]

    def create(self, *args):
        self.cube = vtk.vtkCubeSource()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.cube.GetOutput())
        self.SetMapper(mapper)

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            dims = self.coords()
            if len(dims) == 3:
                xw, yw, zw = self.coords()
                self.cube.SetXLength(xw)
                self.cube.SetYLength(yw)
                self.cube.SetZLength(zw)
                self.cube.Update()
            if len(dims) == 6:
                x1, y1, z1, x2, y2, z2 = self.coords()
                if x1 > x2:
                    tmp = x1
                    x1 = x2
                    x2 = tmp
                if y1 > y2:
                    tmp = y1
                    y1 = y2
                    y2 = tmp
                if z1 > z2:
                    tmp = z1
                    z1 = z2
                    z2 = tmp
                self.cube.SetXLength(x2-x1)
                self.cube.SetYLength(y2-y1)
                self.cube.SetZLength(z2-z1)
                self.cube.Update()
                self.SetPosition(x1,y1,z1)
                self.AddPosition((x2-x1)/2,(y2-y1)/2,(z2-z1)/2)


# specify the width in X and Y, and the height in Z
# the box is centered on the origin
class Sphere(ArgsBase, vtk.vtkActor):
    def get_expected_args(self):
        return ('(comp)','x', 'y', 'z', 'r')

    def create(self, *args):
        self.sphere = vtk.vtkSphereSource()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.sphere.GetOutput())
        self.SetMapper(mapper)

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            x, y, z, r = self.coords()
            self.sphere.SetRadius(r)
            self.sphere.Update()
            self.SetPosition(x,y,z)


# Create cylinder along Y axis (default direction for vtkCylinderSource)
class CylinderY(ArgsBase, vtk.vtkActor):
    def get_expected_args(self):
        return ('(comp)','length', 'radius')

    def create(self, *args):
        self.resolution = 10
        self.cylinder = vtk.vtkCylinderSource()
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputData(self.cylinder.GetOutput())
        self.SetMapper(self.mapper)

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            length, radius = self.coords()
            self.cylinder.SetRadius(radius)
            self.cylinder.SetHeight(length)
            self.cylinder.SetResolution(self.resolution)
            self.SetUserTransform(vtk.vtkTransform())
            self.GetUserTransform().Translate(0,length/2,0)
            self.cylinder.Update()


# Create cylinder along Z axis
class CylinderZ(CylinderY):
    def create(self, *args):
        super().create(self)
        self.RotateX(90)

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            length, radius = self.coords()
            self.cylinder.SetRadius(radius)
            self.cylinder.SetHeight(length)
            self.SetUserTransform(vtk.vtkTransform())
            self.GetUserTransform().Translate(0,0,length/2)
            self.cylinder.Update()


# Create cylinder along X axis
class CylinderX(CylinderY):
    def create(self, *args):
        super().create(self)
        self.RotateZ(-90)

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            length, radius = self.coords()
            self.cylinder.SetRadius(radius)
            self.cylinder.SetHeight(length)
            self.SetUserTransform(vtk.vtkTransform())
            self.GetUserTransform().Translate(length/2,0,0)
            self.cylinder.Update()


# draw a line from one point to another
class Line(ArgsBase, vtk.vtkActor):
    def get_expected_args(self):
        return ('(comp)','x_start', 'y_start', 'z_start', 'x_end', 'y_end', 'z_end')

    def create(self):
        self.lineSource = vtk.vtkLineSource()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(self.lineSource.GetOutputPort())
        self.SetMapper(mapper)

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            xs, ys, zs, xe, ye, ze = self.coords()
            self.lineSource.SetPoint1(xs,ys,zs)
            self.lineSource.SetPoint2(xe,ye,ze)
            self.GetProperty().SetLineWidth(1)


# Creates a 3d cylinder from (xs,ys,zs) to (xe,ye,ze)
class CylinderOriented(ArgsBase, vtk.vtkActor):
    def get_expected_args(self):
        return ('(comp)','x_start', 'y_start', 'z_start', 'x_end', 'y_end', 'z_end', 'radius')

    def create(self):
        self.resolution = 10
        # Create a cylinder (cylinders are created along Y axis by default)
        # Cylinder center is in the middle of the cylinder
        self.cylinderSource = vtk.vtkCylinderSource()
        self.cylinderSource.SetResolution(self.resolution)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(self.cylinderSource.GetOutputPort())
        self.SetMapper(mapper)

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            xs, ys, zs, xe, ye, ze, radius = self.coords()
            self.cylinderSource.SetRadius(radius)
            self.cylinderSource.SetResolution(self.resolution)
            # Generate a random start and end point
            startPoint = [xs, ys, zs]
            endPoint = [xe, ye, ze]
            # Compute a basis
            normalizedX = [0] * 3
            normalizedY = [0] * 3
            normalizedZ = [0] * 3
            # The X axis is a vector from start to end
            vtk.vtkMath.Subtract(endPoint, startPoint, normalizedX)
            length = vtk.vtkMath.Norm(normalizedX)
            vtk.vtkMath.Normalize(normalizedX)
            vtk.vtkMath.Cross(normalizedX, [0,0,1], normalizedZ)
            vtk.vtkMath.Normalize(normalizedZ)
            # The Y axis is Z cross X
            vtk.vtkMath.Cross(normalizedZ, normalizedX, normalizedY)
            matrix = vtk.vtkMatrix4x4()
            # Create the direction cosine matrix
            matrix.Identity()
            for i in range(0, 3):
                matrix.SetElement(i, 0, normalizedX[i])
                matrix.SetElement(i, 1, normalizedY[i])
                matrix.SetElement(i, 2, normalizedZ[i])
            # Apply the transforms
            transform = vtk.vtkTransform()
            transform.Translate(startPoint)  # translate to starting point
            transform.Concatenate(matrix)  # apply direction cosines
            transform.RotateZ(-90.0)  # align cylinder to x axis
            transform.Scale(1.0, length, 1.0)  # scale along the height vector
            transform.Translate(0, .5, 0)  # translate to start of cylinder
            self.SetUserMatrix(transform.GetMatrix())


# Creates a 3d arrow pointing from (xs,ys,zs) to (xe,ye,ze)
class Arrow(ArgsBase, vtk.vtkActor):
    def get_expected_args(self):
        return ('(comp)','x_start', 'y_start', 'z_start', 'x_end', 'y_end', 'z_end', 'radius')

    def create(self):
        self.resolution = 10
        # Create arrow (arrows are created along the X axis by default)
        self.arrowSource = vtk.vtkArrowSource()
        self.arrowSource.SetTipResolution(self.resolution)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(self.arrowSource.GetOutputPort())
        self.SetMapper(mapper)

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            xs, ys, zs, xe, ye, ze, radius,  = self.coords()
            # Create arrow (arrows are created along the X axis by default)
            self.arrowSource.SetShaftRadius(radius)
            self.arrowSource.SetTipRadius(radius*1.6)
            self.arrowSource.SetTipResolution(self.resolution)
            # Generate a random start and end point
            startPoint = [xs, ys, zs]
            endPoint = [xe, ye, ze]
            # Compute a basis
            normalizedX = [0] * 3
            normalizedY = [0] * 3
            normalizedZ = [0] * 3
            # The X axis is a vector from start to end
            vtk.vtkMath.Subtract(endPoint, startPoint, normalizedX)
            length = vtk.vtkMath.Norm(normalizedX)
            if length < 0.1: length = 1
            self.arrowSource.SetTipLength(radius*8/length)
            vtk.vtkMath.Normalize(normalizedX)
            vtk.vtkMath.Cross(normalizedX, [0,0,1], normalizedZ)
            vtk.vtkMath.Normalize(normalizedZ)
            # The Y axis is Z cross X
            vtk.vtkMath.Cross(normalizedZ, normalizedX, normalizedY)
            matrix = vtk.vtkMatrix4x4()
            # Create the direction cosine matrix
            matrix.Identity()
            for i in range(0, 3):
                matrix.SetElement(i, 0, normalizedX[i])
                matrix.SetElement(i, 1, normalizedY[i])
                matrix.SetElement(i, 2, normalizedZ[i])
            # Apply the transforms
            transform = vtk.vtkTransform()
            transform.Translate(startPoint)  # translate to starting point
            transform.Concatenate(matrix)  # apply direction cosines
            transform.Scale(length, 1.0, 1.0)  # scale along the height vector
            transform.Translate(0, .5, 0)  # translate to start of cylinder
            self.SetUserMatrix(transform.GetMatrix())


# Loads 3D geometry from file
class ReadPolyData(ArgsBase, vtk.vtkActor):
    def get_expected_args(self):
        return ('(comp)','filename','path')

    def create(self):
        pass

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            filename, path = self.coords()
            if not isinstance(filename,str): # ie filename is a numeric value from a halpin
                filename = str(filename) + '.stl'
            filepath = path + filename
            mapper = vtk.vtkPolyDataMapper()
            if not os.path.isfile(filepath):
                # If the file is not there we want to print a message, but only once
                if not hasattr(self, 'error_filepath'):
                    print('Vtk_Vismach Error: Unable to read file ', filepath)
                else:
                    if filepath != self.error_filepath:
                        print('Vtk_Vismach Error: Unable to read file ', filepath)
                self.error_filepath = filepath
                # create a dummy sphere instead
                sphereSource = vtk.vtkSphereSource()
                sphereSource.SetCenter(0.0, 0.0, 0.0)
                sphereSource.SetRadius(0.5)
                mapper.SetInputConnection(sphereSource.GetOutputPort())
            else:
                # create from stl file
                path, extension = os.path.splitext(filepath)
                extension = extension.lower()
                if extension == '.ply':
                    reader = vtk.vtkPLYReader()
                    reader.SetFileName(filepath)
                    reader.Update()
                    poly_data = reader.GetOutput()
                elif extension == '.vtp':
                    reader = vtk.vtkXMLPolyDataReader()
                    reader.SetFileName(filepath)
                    reader.Update()
                    poly_data = reader.GetOutput()
                elif extension == '.obj':
                    reader = vtk.vtkOBJReader()
                    reader.SetFileName(filepath)
                    reader.Update()
                    poly_data = reader.GetOutput()
                elif extension == '.stl':
                    reader = vtk.vtkSTLReader()
                    reader.SetFileName(filepath)
                    reader.Update()
                    poly_data = reader.GetOutput()
                elif extension == '.vtk':
                    reader = vtk.vtkXMLPolyDataReader()
                    reader.SetFileName(filepath)
                    reader.Update()
                    poly_data = reader.GetOutput()
                elif extension == '.g':
                    reader = vtk.vtkBYUReader()
                    reader.SetGeometryFileName(filepath)
                    reader.Update()
                    poly_data = reader.GetOutput()
                else:
                    print('ReadPolyData Error: Unable to read file ', filepath)
                mapper.SetInputConnection(reader.GetOutputPort())
            self.SetMapper(mapper)
            # Avoid visible backfaces on Linux with some video cards like intel
            # From: https://stackoverflow.com/questions/51357630/vtk-rendering-not-working-as-expected-inside-pyqt?rq=1#comment89720589_51360335
            self.GetProperty().SetBackfaceCulling(1)


# create a plane, use quad_size to define the size of a quadrant
class Plane(ArgsBase,vtk.vtkActor):
    def get_expected_args(self):
        return ('(comp)','quad_size')

    def create (self):
        # Create a plane in xy with origin at (0,0,0)
        planeSource = vtk.vtkPlaneSource()
        planeSource.SetNormal(0,0,1)
        planeSource.Update()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(planeSource.GetOutputPort())
        self.SetMapper(mapper)

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            scale = self.coords()*2
            self.SetScale(scale,scale,scale)


# Create a trihedron indicating coordinate orientation
# Basically the same as the vtkAxesActor but Color() can be used to change color and opacity
class Axes(ArgsBase, vtk.vtkAssembly):
    def get_expected_args(self):
        return ('(comp)','scale')

    def create(self):
        for axis in ('x','y','z'):
            arrow = Arrow(0,0,0,1,0,0,0.02)
            transform = vtk.vtkTransform()
            colors = vtk.vtkNamedColors()
            color = 'red'
            if axis == 'y':
                transform.RotateZ(90)
                color = 'lime'
            elif axis == 'z':
                transform.RotateY(-90)
                color = 'blue'
            arrow.SetUserTransform(transform)
            arrow.GetProperty().SetColor(colors.GetColor3d(color))
            self.AddPart(arrow)

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            scale = self.coords()
            self.SetScale(scale,scale,scale)


# draw a grid, use quad_size to define the size of a quadrant
# As for why we are not using vtkRectilinearGrid() with wireframe for this see:
# https://gitlab.kitware.com/vtk/vtk/-/issues/18453
class Grid(ArgsBase,vtk.vtkAssembly):
    def get_expected_args(self):
        return ('(comp)','quad_size','spacing')

    def create (self):
        self.qs, self.sp = self.coords()
        qs = self.qs
        sp = self.sp
        line_values = range(-qs,qs+sp,sp)
        dim = len(line_values)
        for i in line_values:
            # create line in X direction
            line_x = Line(-qs, i, 0, qs, i, 0)
            self.AddPart(line_x)
            # create line in Y direction
            line_y = Line(i,-qs, 0,  i, qs, 0)
            self.AddPart(line_y)

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            self.qs, self.sp = self.coords()
            #TODO find a way for this to work with halpin values


class Translate(ArgsBase,vtk.vtkAssembly):
    def get_expected_args(self):
        return [('[parts]','(comp)','x','y','z'),('[parts]','(comp)','x','y','z','vel_mode')]

    def update(self):
        if self.needs_updates or self.first_update:
            args = self.coords()
            if len(args) == 3:
                x,y,z = args
                self.transformation = vtk.vtkTransform()
            else:
                x,y,z,vel_mode = args
                if self.first_update:
                    self.transformation = vtk.vtkTransform()
                if not vel_mode:
                    self.transformation = vtk.vtkTransform()
            self.first_update = False
            self.transformation.Translate(x,y,z)

    def transform(self):
        self.SetUserTransform(self.transformation)


class Rotate(ArgsBase,vtk.vtkAssembly):
    def get_expected_args(self):
        return [('[parts]','(comp)','th','x','y','z'),('[parts]','(comp)','th','x','y','z','vel_mode')]

    def update(self):
        if self.needs_updates or self.first_update:
            args = self.coords()
            if len(args) == 4:
                th,x,y,z = args
                vel_mode = False
            else:
                th,x,y,z,vel_mode = args
            if not vel_mode or (vel_mode and self.first_update):
                self.transformation = vtk.vtkTransform()
            self.first_update = False
            self.transformation.PreMultiply()
            self.transformation.RotateWXYZ(th,x,y,z)

    def transform(self):
        self.SetUserTransform(self.transformation)


# Collects a list of Actors and Assemblies into a new assembly
class Collection(ArgsBase,vtk.vtkAssembly):
    pass


class Color(ArgsBase,vtk.vtkAssembly):
    # Color property needs to be set in each individual actor in the vtkAssembly, parts that have been created by
    # a transformation (eg Translate(), Rotate(), Scale()) will always inherit and change with the parent part.
    def get_expected_args(self):
        return [('[parts]','(comp)','color', 'opacity'),('[parts]','(comp)','red','green','blue','opacity')]

    def create (self):
        def find_actors(parts):
            if hasattr(parts, 'GetParts'):
                for item in parts.GetParts():
                    if isinstance(item, vtk.vtkActor):
                        self.parts_to_update.append(item)
                    elif isinstance(item, vtk.vtkAssembly):
                        find_actors(item)
            else:
                self.parts_to_update.append(parts)
        self.parts_to_update = []
        for part in self.GetParts():
            find_actors(part)

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            args = self.coords() # can be (r,g,b,a) or (color,a)
            opacity_only = False
            if isinstance(args[0],str):  # ie (color, a) has been passed
                color, opacity = args
            elif args[0] == None: # None instead of color string passed
                opacity = args[1]
                opacity_only = True
            else:
                color = (args[0],args[1],args[2])
                opacity = args[3]
            for part in self.parts_to_update:
                if not opacity_only:
                    if not isinstance(color, tuple):
                        try:
                            colors = vtk.vtkNamedColors()
                            part.GetProperty().SetColor(colors.GetColor3d(color))
                        except:
                            pass
                    else:
                        part.GetProperty().SetColor(color)
                part.GetProperty().SetOpacity(opacity)


class RotateEuler(ArgsBase,vtk.vtkAssembly):
    def get_expected_args(self):
        return ('[parts]','(comp)','order','th1','th2','th3')

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            order, th1, th2, th3 = self.coords()
            order = str(int(order))
            if order == '131':
                rotation1 = (th1, 1, 0, 0)
                rotation2 = (th2, 0, 0, 1)
                rotation3 = (th3, 1, 0, 0)
            elif order =='121':
                rotation1 = (th1, 1, 0, 0)
                rotation2 = (th2, 0, 1, 0)
                rotation3 = (th3, 1, 0, 0)
            elif order =='212':
                rotation1 = (th1, 0, 1, 0)
                rotation2 = (th2, 1, 0, 0)
                rotation3 = (th3, 0, 1, 0)
            elif order =='232':
                rotation1 = (th1, 0, 1, 0)
                rotation2 = (th2, 0, 0, 1)
                rotation3 = (th3, 0, 1, 0)
            elif order =='323':
                rotation1 = (th1, 0, 0, 1)
                rotation2 = (th2, 0, 1, 0)
                rotation3 = (th3, 0, 0, 1)
            elif order =='313':
                rotation1 = (th1, 0, 0, 1)
                rotation2 = (th2, 1, 0, 0)
                rotation3 = (th3, 0, 0, 1)
            elif order =='123':
                rotation1 = (th1, 1, 0, 0)
                rotation2 = (th2, 0, 1, 0)
                rotation3 = (th3, 0, 0, 1)
            elif order =='132':
                rotation1 = (th1, 1, 0, 0)
                rotation2 = (th2, 0, 0, 1)
                rotation3 = (th3, 0, 1, 0)
            elif order =='213':
                rotation1 = (th1, 0, 1, 0)
                rotation2 = (th2, 1, 0, 0)
                rotation3 = (th3, 0, 0, 1)
            elif order =='231':
                rotation1 = (th1, 0, 1, 0)
                rotation2 = (th2, 0, 0, 1)
                rotation3 = (th3, 1, 0, 0)
            elif order =='321':
                rotation1 = (th1, 0, 0, 1)
                rotation2 = (th2, 0, 1, 0)
                rotation3 = (th3, 1, 0, 0)
            elif order =='312':
                rotation1 = (th1, 0, 0, 1)
                rotation2 = (th2, 1, 0, 0)
                rotation3 = (th3, 0, 1, 0)
            euler_transform = vtk.vtkTransform()
            euler_transform.RotateWXYZ(*rotation1)
            euler_transform.RotateWXYZ(*rotation2)
            euler_transform.RotateWXYZ(*rotation3)

    def transform(self):
        self.SetUserMatrix(euler_transform.GetMatrix())


# shows an object if const=var and hides it otherwise, behavior can be changed
# using the optional arguments for scalefactors when true or false
class Scale(ArgsBase,vtk.vtkAssembly):
    def get_expected_args(self):
        return [('[parts]','(comp)','scale_x','scale_y','scale_z'),('[parts]','(comp)','const','var','scalefactor_if_true','scalefactor_if_false')]

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            args = self.coords()
            if len(args) == 3:
                self.SetScale(*args)
            else:
                const, var, s_t, s_f = args
                if not isinstance(const, list):
                    const = [const]
                if var in const:
                    self.SetScale(s_t,s_t,s_t)
                else:
                    self.SetScale(s_f,s_f,s_f)


# creates a transformaation matrix from given X and Z orientation and a translation vector
# input parts will first be rotated and then translated
class MatrixTransform(ArgsBase,vtk.vtkAssembly):
    def get_expected_args(self):
        return ('[parts]','(comp)','xx','xy','xz','zx','zy','zz','px','py','pz')

    def cross(self, a, b):
        return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

    def update(self):
        if self.needs_updates or self.first_update:
            self.first_update = False
            xx, xy, xz, zx, zy, zz, px, py, pz  = self.coords()
            # create transformation
            vp = [px, py, pz]
            vx = [xx, xy, xz]
            vz = [zx, zy, zz]
            # calculate the missing y vector
            vy = [yx, yy, yz] = self.cross(vz,vx)
            matrix = [[ xx, yx, zx, px],
                      [ xy, yy, zy, py],
                      [ xz, yz, zz, pz],
                      [  0,  0,  0,  1]]
            transform_matrix = vtk.vtkMatrix4x4()
            for column in range (0,4):
                for row in range (0,4):
                    transform_matrix.SetElement(column, row, matrix[column][row])
            self.SetUserMatrix(transform_matrix)


# Finds the Capture('tool') and Capture('work') in a model and draws a polyline showing the path of 'tooltip' with respect to 'work'
class _Plotter(vtk.vtkActor):
    def __init__(self, model, comp, clear, color='magenta'):
        self.model = model          # machine model containing a least Capture('tool') and Capture('work') object
        self.comp = comp            # instance of the halcomponent used in the model
        self.clear = clear          # halpin that clears the backplot
        self.color = color          # color of backplot in eiter nomalized RGB or one of vtkNamedColors
        self.tool_tracker = None    # Capture object with '.matrix' holding the current transformation tool->world
        self.work_tracker = None    # Capture object with '.matrix' holding the current transformation work->world
        self.tool2work = vtk.vtkTransform()
        self.get_trackers(model)     # find the Capture('tool') and Capture('work') objects in the model
        if not self.tool_tracker:
            self.ready = False
            print("Backplot Error: Unable to find the Capture('tool') object in the model")
            return
        if not self.work_tracker:
            self.ready = False
            print("Backplot Error: Unable to find the Capture('work') object in the model")
            return
        self.ready = True
        # We initialize at the origin, this is cleared and set to the actual
        # machine reference position during the 1. update loop
        self.setup_points([0,0,0])
        self.initial_run = True

    def get_trackers(self, objects):
        for item in objects.GetParts():
            if hasattr(item, 'GetProperty'):
                if item.GetProperty().GetObjectName() == 'tool':
                    self.tool_tracker = item
                if item.GetProperty().GetObjectName() == 'work':
                    self.work_tracker = item
            if isinstance(item, vtk.vtkAssembly):
                self.get_trackers(item)

    def setup_points(self, pos):
        self.index = 0
        self.num_points = 2
        self.points = vtk.vtkPoints()
        self.points.InsertNextPoint(pos)
        self.lines = vtk.vtkCellArray()
        self.lines.InsertNextCell(1)  # number of points
        self.lines.InsertCellPoint(0)
        self.lines_poligon_data = vtk.vtkPolyData()
        self.lines_poligon_data.SetPoints(self.points)
        self.lines_poligon_data.SetLines(self.lines)
        colors = vtk.vtkNamedColors()
        self.GetProperty().SetColor(colors.GetColor3d(self.color))
        self.GetProperty().SetLineWidth(2.5)
        self.GetProperty().SetOpacity(0.5)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.lines_poligon_data)
        mapper.Update()
        self.SetMapper(mapper)

    def update(self):
        tool2world = vtk.vtkTransform()
        tool2world.Concatenate(self.tool_tracker.current_matrix)
        #world2tool = vtk.vtkTransform()
        #world2tool = tool2world.GetInverse()
        work2world = vtk.vtkTransform()
        work2world.Concatenate(self.work_tracker.current_matrix)
        world2work = vtk.vtkTransform()
        world2work = work2world.GetInverse()
        plot_transform = vtk.vtkTransform()
        plot_transform.Concatenate(world2work) # work position [0,0,0] > World [0,y,0]
        plot_transform.Concatenate(tool2world) # tool position [0,0,0] > World [x,0,z]
        x = plot_transform.GetMatrix().GetElement(0,3)
        y = plot_transform.GetMatrix().GetElement(1,3)
        z = plot_transform.GetMatrix().GetElement(2,3)
        current_position = (x, y, z)
        if self.comp[self.clear] or self.initial_run:
            self.points.Reset()
            self.setup_points([x,y,z])
            self.initial_run = False
        self.index += 1
        self.points.InsertNextPoint(current_position)
        self.points.Modified()
        self.lines.InsertNextCell(self.num_points)
        self.lines.InsertCellPoint(self.index - 1)
        self.lines.InsertCellPoint(self.index)
        self.lines.Modified()
        self.SetUserTransform(vtk.vtkTransform())
        self.GetUserTransform().Concatenate(work2world)
        # Calculate tool2work to experiment
        self.tool2work = vtk.vtkTransform()
        self.tool2work.Concatenate(world2work)
        self.tool2work.Concatenate(tool2world) # tool position [0,0,0] > Work [x,y,z]


# create (invisible) actor that can be used to track combined transformation to world coordinates
class Capture(vtk.vtkActor):
    def __init__(self, name):
        self.SetUserTransform(vtk.vtkTransform())
        self.GetProperty().SetObjectName(name)
        self.current_matrix = self.GetMatrix()
        self.tracked_parts = [self]

    def update(self):
        self.current_matrix = self.GetMatrix()    # store the total transformation from this cycle
        self.SetUserTransform(vtk.vtkTransform()) # reset tranform for next update cycle


# Create a text overlay (HUD)
# color can be either name string (eg 'red','magenta') or normalized RGB as tuple (eg (0.7,0.7,0.1))
class Hud(vtk.vtkActor2D):
    def __init__(self, comp=None, var=True, const=True, color='white', opacity=1, font_size=20, line_spacing=1):
        self.comp = comp
        self.var = var
        self.const = const
        self.strs = []
        self.hud_lines = []
        self.show_tags = []
        self.hide_huds = []
        self.extra_text_enable = False
        self.extra_text = None
        self.textMapper = vtk.vtkTextMapper()
        tprop = self.textMapper.GetTextProperty()
        tprop.SetLineSpacing(line_spacing)
        tprop.SetFontSize(font_size)
        tprop.SetFontFamilyAsString('Courier')
        tprop.SetJustificationToLeft()
        tprop.SetVerticalJustificationToTop()
        colors = vtk.vtkNamedColors()
        if not isinstance(color, tuple):
            tprop.SetColor(colors.GetColor3d(color))
        else:
            tprop.SetColor(color)
        tprop.SetOpacity(opacity)
        self.SetMapper(self.textMapper)
        self.GetPositionCoordinate().SetCoordinateSystemToNormalizedDisplay()
        self.GetPositionCoordinate().SetValue(0.05, 0.95)

    # displays a string, optionally a tag or list of tags can be assigned
    def add_txt(self, string, tag=None):
        self.hud_lines += [[str(string), None, None, tag]]

    # displays a formatted pin value (can be embedded in a string)
    def add_pin(self, string, comp, pin, tag=None):
        self.hud_lines += [[str(string), comp, pin, tag]]

    # shows all lines with the specified tags if the pin value = val
    def show_tags_if_pin_eq_val(self, tags, comp, pin, val=True):
        self.show_tags += [[tags, comp, pin, val]]

    # shows all lines with a tag equal to the pin value + offset
    def show_tag_eq_pin_offs(self, comp, pin, offs=0):
        self.show_tags += [[None, comp, pin, offs]]

    # hides the complete hud if the pin value is equal to val
    def hide_hud(self,comp, pin, val=True):
        self.hide_huds += [[comp, pin, val]]

    # update the lines in the hud using the lists created above
    def update(self):
        if isinstance(self.comp, hal.component):
            # if the component has been passed then we need to get the value using that
            var = self.comp[self.var]
        elif isinstance(self.comp,type(hal)):
            # if the comp variable is None then we need to get the value through hal
            var = hal.get_value(self.var)
        else:
            var = self.var
        hide_hud = 0 if var == self.const else 1
        strs = []
        show_list = [None]
        # check if hud should be hidden
        for a in self.hide_huds:
            comp = a[0]
            pin = a[1]
            const = a[2]
            var = hal.get_value(pin) if isinstance(comp,type(hal)) else comp[pin]
            if  var == const:
                hide_hud = 1
        if hide_hud == 0:
            # create list of all line tags to be shown
            for b in self.show_tags:
                tags = b[0]
                comp = b[1]
                pin  = b[2]
                val_offs = b[3]
                if tags == None: # show_tag_eq_pin_offs
                    var = hal.get_value(pin) if isinstance(comp,type(hal)) else comp[pin]
                    tag = int(var + val_offs)
                else: # show_tags_if_pin_eq_val
                    var = hal.get_value(pin) if isinstance(comp,type(hal)) else comp[pin]
                    if  var == val_offs:
                        tag = tags
                if not isinstance(tag, list):
                    tag = [tag]
                show_list = show_list + tag
            # build the strings
            for c in self.hud_lines:
                text = c[0]
                comp = c[1]
                pins = c[2]
                tags = c[3]
                if pins and not isinstance(pins, list):
                    pins = [pins]
                if not isinstance(tags, list):
                    tags = [tags]
                if any(tag in tags for tag in show_list):
                    if comp == None and pins == None: # txt
                        strs += [text]
                    elif pins: # pins
                        values = []
                        for pin in pins:
                            val = hal.get_value(pin) if isinstance(comp,type(hal)) else comp[pin]
                            values.append(val)
                        strs += [text.format(*tuple(values))]
        combined_string = ''
        for string in strs:
            combined_string += (string + '\n')
        if self.extra_text_enable and self.extra_text and not hide_hud:
            combined_string += self.extra_text
        self.textMapper.SetInput(combined_string)


class ModelGroups(object):
    def __init__(self, model):
        self.groups_in_model = {}
        self.get_groups(model)
        self.has_groups = len(self.groups_in_model.keys()) > 0

    def get_groups(self, objects):
        for item in objects.GetParts():
            if hasattr(item, 'group'):
                if item.group:
                    if item.group in self.groups_in_model.keys():
                        self.groups_in_model[item.group].append(item)
                    else:
                        self.groups_in_model[item.group] = [item]
            if isinstance(item, vtk.vtkAssembly):
                self.get_groups(item)

    def update(self, checkboxes):
        # SetVisibility does not seem to work on modifier objects so we use Scale
        for group in self.groups_in_model.keys():
            for checkbox in checkboxes:
                if checkbox.objectName() == group:
                    if checkbox.isChecked():
                        for item in self.groups_in_model[group]:
                            item.restore_scale()
                    else:
                        for item in self.groups_in_model[group]:
                            item.store_scale()
                            item.SetScale(0,0,0)


class MainWindow(Qt.QMainWindow):
    def __init__(self, width, height, title, argv_options, backplot, huds, model_groups):
        super().__init__()
        self.resize(width, height)
        self.setWindowTitle(title)
        self.backplot = backplot
        self.parProj = False
        self.view = ' '
        self.trackTool = False
        self.trackWork = False
        self.last_tracking_position = (0,0,0)
        self.no_buttons = False
        self.groups_checkbox_clicked = False
        if '--no-buttons' in argv_options:
            self.no_buttons = True
            print('VISMACH: Window without buttons requested')
            # VTK Interactor (this is where the model is going to be)
            self.vtkInteractor = QVTKRenderWindowInteractor(self)
            self.setCentralWidget(self.vtkInteractor)
        else:
            # Side panel for the buttons
            sdePnlLyt = QtWidgets.QVBoxLayout()
            # Projection
            grpProjLyt = QtWidgets.QVBoxLayout()
            self.rbtnPrsptve = QtWidgets.QRadioButton("Perspective")
            self.rbtnPrsptve.setChecked(True)
            self.rbtnPrsptve.projection = "Perspective"
            self.rbtnPrsptve.toggled.connect(self.rbtnPrsptve_clicked)
            grpProjLyt.addWidget(self.rbtnPrsptve)
            self.rbtnPrsptve = QtWidgets.QRadioButton("Parallel")
            self.rbtnPrsptve.projection = "Parallel"
            self.rbtnPrsptve.toggled.connect(self.rbtnPrsptve_clicked)
            grpProjLyt.addWidget(self.rbtnPrsptve)
            grpProj = QtWidgets.QGroupBox("Projection")
            grpProj.setLayout(grpProjLyt)
            sdePnlLyt.addWidget(grpProj)
            # Orthographic View
            grpViewLyt = QtWidgets.QVBoxLayout()
            self.btnYZ = QtWidgets.QPushButton()
            self.btnYZ.setText('YZ / -YZ')
            self.btnYZ.clicked.connect(self.btnYZ_clicked)
            grpViewLyt.addWidget(self.btnYZ)
            self.btnXZ = QtWidgets.QPushButton()
            self.btnXZ.setText('XZ / -XZ')
            self.btnXZ.clicked.connect(self.btnXZ_clicked)
            grpViewLyt.addWidget(self.btnXZ)
            self.btnXY = QtWidgets.QPushButton()
            self.btnXY.setText('XY / -XY')
            self.btnXY.clicked.connect(self.btnXY_clicked)
            grpViewLyt.addWidget(self.btnXY)
            self.btnDimtrc = QtWidgets.QPushButton()
            self.btnDimtrc.setText('Dimetric')
            self.btnDimtrc.clicked.connect(self.btnDimtrc_clicked)
            grpViewLyt.addWidget(self.btnDimtrc)
            grpView = QtWidgets.QGroupBox("Ortho View")
            grpView.setLayout(grpViewLyt)
            sdePnlLyt.addWidget(grpView)
            # Others
            self.btnZUp = QtWidgets.QPushButton()
            self.btnZUp.setText('Z Upright')
            self.btnZUp.clicked.connect(self.btnZUp_clicked)
            sdePnlLyt.addWidget(self.btnZUp)
            self.btnShowAll = QtWidgets.QPushButton()
            self.btnShowAll.setText('Show All')
            self.btnShowAll.clicked.connect(self.btnShowAll_clicked)
            sdePnlLyt.addWidget(self.btnShowAll)
            # Camera tracking
            grpTrkgLyt = QtWidgets.QVBoxLayout()
            self.rbtnTrkg = QtWidgets.QRadioButton("None")
            self.rbtnTrkg.setChecked(True)
            self.rbtnTrkg.tracking = "None"
            self.rbtnTrkg.toggled.connect(self.rbtnTrkg_clicked)
            grpTrkgLyt.addWidget(self.rbtnTrkg)
            self.rbtnTrkg = QtWidgets.QRadioButton("Tool")
            self.rbtnTrkg.tracking = "Tool"
            self.rbtnTrkg.toggled.connect(self.rbtnTrkg_clicked)
            grpTrkgLyt.addWidget(self.rbtnTrkg)
            self.rbtnTrkg = QtWidgets.QRadioButton("Work")
            self.rbtnTrkg.tracking = "Work"
            self.rbtnTrkg.toggled.connect(self.rbtnTrkg_clicked)
            grpTrkgLyt.addWidget(self.rbtnTrkg)
            grpTrkg = QtWidgets.QGroupBox("Tracking")
            grpTrkg.setLayout(grpTrkgLyt)
            sdePnlLyt.addWidget(grpTrkg)
            # clear backplot
            if backplot.ready:
                self.btnClrPlot = QtWidgets.QPushButton()
                self.btnClrPlot.setText('Clear Backplot')
                self.btnClrPlot.clicked.connect(self.btnClrPlot_clicked)
                sdePnlLyt.addWidget(self.btnClrPlot)
            if huds:
                self.rbtnHud = QtWidgets.QRadioButton("Show Overlay")
                self.rbtnHud.setChecked(True)
                sdePnlLyt.addWidget(self.rbtnHud)
            if len(model_groups.groups_in_model.keys()) > 0:
                grpGrpLyt = QtWidgets.QVBoxLayout()
                self.checkboxes_group =  []
                for group in model_groups.groups_in_model.keys():
                    self.ckbxGrp = QtWidgets.QCheckBox("Show " + group)
                    self.ckbxGrp.setObjectName(group)
                    self.ckbxGrp.setChecked(True)
                    self.ckbxGrp.clicked.connect(self.ckbxGrp_clicked)
                    self.checkboxes_group.append(self.ckbxGrp)
                    grpGrpLyt.addWidget(self.ckbxGrp)
                grpGrp = QtWidgets.QGroupBox("Model Groups")
                grpGrp.setLayout(grpGrpLyt)
                sdePnlLyt.addWidget(grpGrp)
            sdePnlLyt.addStretch()
            # VTK Interactor (this is where the model is going to be)
            self.vtkInteractor = QVTKRenderWindowInteractor(self)
            # Main layout
            mainHLyt = QtWidgets.QHBoxLayout()
            mainHLyt.addWidget(self.vtkInteractor)
            mainHLyt.addLayout(sdePnlLyt)
            mainHLyt.setStretchFactor(self.vtkInteractor,10)
            mainHLyt.setStretchFactor(sdePnlLyt,1)
            # Centralwidget
            centralwidget = QtWidgets.QWidget()
            centralwidget.setLayout(mainHLyt)
            self.setCentralWidget(centralwidget)

    def rbtnPrsptve_clicked(self):
        renderer = self.vtkInteractor.GetRenderWindow().GetRenderers().GetFirstRenderer()
        camera = renderer.GetActiveCamera()
        self.rbtnPrsptve = self.sender()
        if self.rbtnPrsptve.projection == "Perspective":
            camera.SetParallelProjection(False)
        elif self.rbtnPrsptve.projection == "Parallel":
            camera.SetParallelProjection(True)

    def btnYZ_clicked(self):
        self.set_ortho_view('x',(0,0,1))

    def btnXZ_clicked(self):
        self.set_ortho_view('y',(0,0,1))

    def btnXY_clicked(self):
        self.set_ortho_view('z',(0,1,0))

    def set_ortho_view(self,axis,up):
        renderer = self.vtkInteractor.GetRenderWindow().GetRenderers().GetFirstRenderer()
        camera = renderer.GetActiveCamera()
        camera.SetViewUp(*up)
        pos = [0,0,0]
        a = ['x','y','z'].index(axis)
        if self.view == axis:
            s = -10
            self.view = '-'+axis
        else:
            s = 10
            self.view = axis
        pos[a] = s
        camera.SetPosition(*pos)
        camera.SetFocalPoint(0,0,0)
        renderer.ResetCamera()

    def btnDimtrc_clicked(self):
        renderer = self.vtkInteractor.GetRenderWindow().GetRenderers().GetFirstRenderer()
        camera = renderer.GetActiveCamera()
        camera.SetViewUp(0,0,1)
        camera.SetPosition(10,0,0)
        camera.SetFocalPoint(0,0,0)
        if self.view[0] == 'p':
            next_idx = int(self.view[1])+1
            camera.Azimuth(-45 + next_idx*90)
            self.view = 'p'+str(next_idx)
        else:
            camera.Azimuth(-45)
            self.view = 'p0'
        # isometric view creates flat lighting so we use dimetric
        camera.Elevation(30)
        renderer.ResetCamera()

    def btnZUp_clicked(self):
        renderer = self.vtkInteractor.GetRenderWindow().GetRenderers().GetFirstRenderer()
        camera = renderer.GetActiveCamera()
        # a renderer warning is produced if the current viewplane is already parallel to XY
        if not (camera.GetViewUp() == (0,0,1) or camera.GetViewUp() == (0,1,0)):
            camera.SetViewUp(0,0,1)

    def btnShowAll_clicked(self):
        renderer = self.vtkInteractor.GetRenderWindow().GetRenderers().GetFirstRenderer()
        camera = renderer.GetActiveCamera()
        renderer.ResetCamera()

    def rbtnTrkg_clicked(self):
        self.rbtnTrkg = self.sender()
        if self.rbtnTrkg.tracking == "Tool":
            self.trackTool = True
            self.trackWork = False
        elif self.rbtnTrkg.tracking == "Work":
            self.trackWork = True
            self.trackTool = False
        else:
            self.trackWork = False
            self.trackTool = False

    def btnClrPlot_clicked(self):
        self.backplot.initial_run = True

    def ckbxGrp_clicked(self):
        self.groups_checkbox_clicked = True


def main(argv_options, comp, model, huds=None,
         window_title='Vtk-Vismach', window_width=600, window_height=300,
         camera_azimuth=-50, camera_elevation=30,
         background_rgb = (0.2, 0.3, 0.4)):

    # Event loop to periodically update the model
    def update():
        def get_actors_to_update(objects):
            for item in objects.GetParts():
                if hasattr(item, 'update'):
                    item.update()
                if hasattr(item, 'transform'):
                    item.transform()
                if hasattr(item, 'capture'):
                    item.capture()
                if isinstance(item, vtk.vtkAssembly):
                    get_actors_to_update(item)
        # Update model
        get_actors_to_update(model)
        # Update backplot
        if backplot.ready:
            # Update tool->world matrix
            t2w = backplot.tool2work.GetMatrix()
            r1=('{:6.3f} {:6.3f} {:6.3f} {:9.3f}'.format(t2w.GetElement(0,0), t2w.GetElement(0,1), t2w.GetElement(0,2), t2w.GetElement(0,3)))
            r2=('{:6.3f} {:6.3f} {:6.3f} {:9.3f}'.format(t2w.GetElement(1,0), t2w.GetElement(1,1), t2w.GetElement(1,2), t2w.GetElement(1,3)))
            r3=('{:6.3f} {:6.3f} {:6.3f} {:9.3f}'.format(t2w.GetElement(2,0), t2w.GetElement(2,1), t2w.GetElement(2,2), t2w.GetElement(2,3)))
            t2w_matrix_text = '\nTool -> Work Transformation:\n'+'   X      Y      Z      Pos\n'+r1+'\n'+r2+'\n'+r3
            backplot.update()
        else:
            t2w_matrix_text = ''
            backplot.update()
        # Update HUD
        if huds:
            for hud in huds:
                hud.extra_text = t2w_matrix_text
                hud.update()
        # With the sidepanel we have some more things to update depending on the button states
        if not mainWindow.no_buttons:
            # Update HUD visibility
            if huds:
                if mainWindow.rbtnHud.isChecked():
                    for hud in huds:
                        hud.VisibilityOn()
                else:
                    for hud in huds:
                        hud.VisibilityOff()
            # Update model group visibility
            if mainWindow.groups_checkbox_clicked:
                model_groups.update(mainWindow.checkboxes_group)
                mainWindow.groups_checkbox_clicked = False
            # Update camera tracking
            if mainWindow.trackTool or mainWindow.trackWork:
                renderWindow = mainWindow.vtkInteractor.GetRenderWindow()
                renderer = renderWindow.GetRenderers().GetFirstRenderer()
                camera = renderer.GetActiveCamera()
                fp = camera.GetFocalPoint()
                cp = camera.GetPosition()
                if mainWindow.trackTool:
                    matrix = backplot.tool_tracker.current_matrix
                else:
                    matrix = backplot.work_tracker.current_matrix
                x, y, z = matrix.GetElement(0,3), matrix.GetElement(1,3), matrix.GetElement(2,3)
                xl,yl,zl = mainWindow.last_tracking_position
                camera.SetFocalPoint(fp[0] + x - xl, fp[1] + y - yl, fp[2] + z - zl)
                camera.SetPosition  (cp[0] + x - xl, cp[1] + y - yl, cp[2] + z - zl)
                mainWindow.last_tracking_position = x,y,z
        # Render updated data
        mainWindow.vtkInteractor.GetRenderWindow().Render()

    vcomp = hal.component('vismach')
    vcomp.newpin('plotclear',hal.HAL_BIT,hal.HAL_IN)
    vcomp.ready()
    # create the backplot to be added to the renderer
    backplot = _Plotter(model, vcomp, 'plotclear')
    # collect group tags
    model_groups = ModelGroups(model)
    # close vismach if linuxcnc is closed
    def quit(*args):
        raise SystemExit
    signal.signal(signal.SIGTERM, quit)
    signal.signal(signal.SIGINT, quit)
    # Create the qt app
    if '--dark-theme' in argv_options:
        try:
            import qdarktheme
        except Exception as e:
            print(e)
            print('Try: $ pip install pyqtdarktheme-fork')
        else:
            qdarktheme.enable_hi_dpi()
            app = Qt.QApplication([])
            qdarktheme.setup_theme()
    else:
        app = Qt.QApplication([])
    # Qt Window
    mainWindow = MainWindow(window_width, window_height, window_title,
                            argv_options,
                            backplot,
                            huds,
                            model_groups)
    # A renderer
    renderer = vtk.vtkRenderer()
    renderer.AddActor(model)
    renderer.AddActor(backplot)
    renderer.SetBackground(*background_rgb)
    # Huds
    if huds:
        if not isinstance(huds, list):
            huds = [huds]
        for hud in huds:
            renderer.AddActor(hud)
    # A render window
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    # An interactor
    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(renderWindow)
    # A widget to indicate the orientation of the model in the window
    def trihedron():
        axes = vtk.vtkAxesActor()
        axes.SetShaftTypeToCylinder()
        axes.SetXAxisLabelText('X')
        axes.SetYAxisLabelText('Y')
        axes.SetZAxisLabelText('Z')
        axes.SetCylinderRadius(0.5 * axes.GetCylinderRadius())
        return axes
    orientation_marker = vtk.vtkOrientationMarkerWidget()
    orientation_marker.SetOrientationMarker(trihedron())
    # Put everything in the Qt window
    mainWindow.vtkInteractor.GetRenderWindow().AddRenderer(renderer)
    # Set initial view
    mainWindow.btnDimtrc_clicked()
    # Set interactor style and initialize
    interactor = mainWindow.vtkInteractor.GetRenderWindow().GetInteractor()
    orientation_marker.SetInteractor(interactor)
    orientation_marker.EnabledOn()
    interactor_style = vtk.vtkInteractorStyleTrackballCamera()
    interactor.SetInteractorStyle(interactor_style)
    interactor.Initialize()
    # NOTE
    # We really only used Qt because we need a timer outside of VTK. Due to a vtk bug we cannot use
    # the vtk timer as it stops reporting when we interact with the window (eg rotating the scene)
    # Set up a Qt timer to create update events
    timer = QTimer()
    timer.timeout.connect(update)
    timer.start(100)
    # Show Qt window
    mainWindow.show()
    app.exec_()

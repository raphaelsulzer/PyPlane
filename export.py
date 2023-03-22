import os
import numpy as np
from scipy.spatial import ConvexHull
from copy import deepcopy
from pyplane import PyPlane


class PlaneExporter:

    def __init__(self):
        pass


    def export_deleted_points(self,path,points,count,subfolder="deleted_points",color=None):

        c = color if color is not None else [0.5,0.5,0.5]


        """this writes one group of points"""

        path = os.path.join(path,subfolder)
        os.makedirs(path,exist_ok=True)
        filename = os.path.join(path, str(count) + '.obj')
        f = open(filename, 'w')
        for i,v in enumerate(points):
            f.write('v {} {} {} {} {} {}\n'.format(v[0], v[1], v[2], c[0], c[1], c[2]))
        f.close()


    def export_plane(self,path,plane,points,count,subpaths=["planes","point_groups"],color=None):

        plane = deepcopy(plane)

        c = color if color is not None else [0.5,0.5,0.5]


        os.makedirs(os.path.join(path,subpaths[1]), exist_ok=True)
        filename = os.path.join(path, subpaths[1], str(count) + '.obj')
        f = open(filename, 'w')
        for j, v in enumerate(points):
            f.write('v {} {} {} {} {} {}\n'.format(v[0], v[1], v[2], c[0], c[1], c[2]))
        f.close()

        plane+=(np.random.rand(4,)*0.001)


        ## project verts to plane
        ## https://www.baeldung.com/cs/3d-point-2d-plane
        k = (-plane[-1] - plane[0] * points[:, 0] - plane[1] * points[:, 1] - plane[2] * points[:, 2]) / \
            (plane[0] ** 2 + plane[1] ** 2 + plane[2] ** 2)
        pp = np.asarray([points[:, 0] + k * plane[0], points[:, 1] + k * plane[1], points[:, 2] + k * plane[2]]).transpose()

        # plt.figure()
        # plt.scatter(pp[:,0],pp[:,1])
        # plt.axis('equal')
        # plt.show()

        ch = ConvexHull(pp[:, :2])
        verts = ch.points[ch.vertices]
        verts = np.hstack((verts, pp[ch.vertices, 2, np.newaxis]))

        os.makedirs(os.path.join(path,subpaths[0]), exist_ok=True)
        filename = os.path.join(path, subpaths[0], str(count) + '.obj')
        f = open(filename, 'w')
        fstring = 'f'
        for j, v in enumerate(verts):
            f.write('v {} {} {} {} {} {}\n'.format(v[0], v[1], v[2], c[0], c[1], c[2]))
            fstring += ' {}'.format(j + 1)
        f.write(fstring)

        f.close()


    def export_planes(self,path,planes,points,color=None):

        """this writes all planes"""

        os.makedirs(os.path.join(path,"planes"), exist_ok=True)
        os.makedirs(os.path.join(path,"point_groups"), exist_ok=True)

        for i, plane in enumerate(planes):

            plane = deepcopy(plane)

            plane += (np.random.rand(4, ) * 0.001)

            c = np.random.random(size=3)

            filename = os.path.join(path, "point_groups", str(i) + '.obj')
            f = open(filename, 'w')
            for j,v in enumerate(points[i]):
                f.write('v {} {} {} {} {} {}\n'.format(v[0],v[1],v[2],c[0],c[1],c[2]))
            f.close()

            p = points[i]

            # project verts to plane
            # https://www.baeldung.com/cs/3d-point-2d-plane
            k = (-plane[-1] -plane[0]*p[:, 0] -plane[1]*p[:, 1] -plane[2]*p[:, 2]) / \
                (plane[0] ** 2 + plane[1] ** 2 + plane[2] ** 2)
            pp = np.asarray([p[:, 0] + k * plane[0], p[:, 1] + k * plane[1], p[:, 2] + k * plane[2]]).transpose()


            ch = ConvexHull(pp[:,:2])
            verts = ch.points[ch.vertices]
            verts = np.hstack((verts, pp[ch.vertices,2,np.newaxis]))




            filename = os.path.join(path, "planes", str(i) + '.obj')
            f = open(filename, 'w')
            fstring='f'
            for j,v in enumerate(verts):
                f.write('v {} {} {} {} {} {}\n'.format(v[0],v[1],v[2],c[0],c[1],c[2]))
                fstring+=' {}'.format(j+1)
            f.write(fstring)

            f.close()


    def export_planes_to_ply(self,filename,points,group_num_points,group_points,group_parameters,colors=None):

        """this writes all planes to a ply file"""

        if colors is None:
            colors = np.random.randint(0,255,size=(group_parameters.shape[0],3))

        all_count=0
        hull_count=0
        all_hull_points = []
        all_hull_verts = []
        for i, plane in enumerate(group_parameters):

            ids = group_points[all_count:(group_num_points[i]+all_count)]

            # p = points[ids]
            # plane += (np.random.rand(4, ) * 0.001)
            # # project verts to plane
            # # https://www.baeldung.com/cs/3d-point-2d-plane
            # k = (-plane[-1] -plane[0]*p[:, 0] -plane[1]*p[:, 1] -plane[2]*p[:, 2]) / \
            #     (plane[0] ** 2 + plane[1] ** 2 + plane[2] ** 2)
            # pp = np.asarray([p[:, 0] + k * plane[0], p[:, 1] + k * plane[1], p[:, 2] + k * plane[2]]).transpose()
            # ch = ConvexHull(pp[:,:2])
            # hull_points = ch.points[ch.vertices]
            # all_hull_points.append(np.hstack((hull_points, pp[ch.vertices,2,np.newaxis])))

            hull_points = PyPlane(plane).get_hull_points_of_projected_points(points[ids],dim=3)
            all_hull_points.append(hull_points)

            hull_verts = np.arange(hull_points.shape[0])
            all_hull_verts.append(hull_verts)
            hull_verts+=hull_count
            hull_count+=hull_verts.shape[0]

            all_count+=group_num_points[i]

        all_hull_points = np.concatenate(all_hull_points)


        f = open(filename,"w")
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("element vertex {}\n".format(all_hull_points.shape[0]))
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("element face {}\n".format(group_parameters.shape[0]))
        f.write("property list uchar int vertex_index\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")

        for p in all_hull_points:
            f.write("{:.6} {:.6} {:.6}\n".format(p[0],p[1],p[2]))

        for i,plane in enumerate(all_hull_verts):
            f.write(str(plane.shape[0]))
            f.write(" ")
            for id in plane:
                f.write(str(id)+" ")

            c = colors[i]
            f.write("{} ".format(c[0]))
            f.write("{} ".format(c[1]))
            f.write("{}\n".format(c[2]))

        f.close()



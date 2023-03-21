import os, sys
import numpy as np
from scipy.spatial import ConvexHull, Delaunay
import trimesh
import open3d as o3d

class PyPlane:


    def __init__(self,params,inliers=None):

        params = np.array(params,dtype=float)

        self.params = params

        self.a = params[0]; self.b = params[1]; self.c = params[2]; self.d = params[3]
        self.normal = params[:3]

        ## select max coordinate from plane, ie check towards which coordinate plane is oriented:
        self.max_coord = np.argmax(np.abs(self.normal))

        self.inliers = inliers



    def to_2d(self,points,return_max_coord=False):

        ### project inlier points to plane
        ## https://www.baeldung.com/cs/3d-point-2d-plane
        k = (self.a * points[:, 0] - self.b * points[:, 1] - self.c * points[:, 2] - self.d) / (self.a ** 2 + self.b ** 2 + self.c ** 2)
        pp = np.asarray([points[:, 0] + k * self.a, points[:, 1] + k * self.b, points[:, 2] + k * self.c])

        ## take the max coordinate coords away to make the 2d points
        if return_max_coord:
            return np.delete(pp.transpose(), self.max_coord, axis=1), points[:,self.max_coord]
        else:
            return np.delete(pp.transpose(),self.max_coord,axis=1)



    def to_3d(self,points):

        ## compute the missing max coordinate and insert it into the 2d points, to make them 3d again
        if self.max_coord == 0:
            x = (- self.b * points[:, 0] - self.c * points[:,1] - self.d) / self.a
            return np.hstack((x[:,np.newaxis], points))
        elif self.max_coord == 1:
            y = (- self.a * points[:, 0] - self.c * points[:,1] - self.d) / self.b
            return np.insert(points,1,y,axis=1)
        elif self.max_coord == 2:
            z = (- self.a * points[:, 0] - self.b * points[:, 1] - self.d) / self.c
            return np.hstack((points, z[:, np.newaxis]))
        else:
            raise RuntimeError


    def get_hull_points_of_projected_points(self,points,return_index=False):

        p2d = self.to_2d(points)

        ch = ConvexHull(p2d)

        if return_index:
            return p2d[ch.vertices], ch.vertices
        else:
            return p2d[ch.vertices]


    def get_trimesh_of_projected_points(self,points,type="convex_hull"):

        if type == "convex_hull":
            p2d = self.to_2d(points)
            ch = ConvexHull(p2d)
            chp = p2d[ch.vertices]
            tri = Delaunay(chp)
            return trimesh.Trimesh(vertices=points[ch.vertices],faces=tri.simplices)

        elif type == "all":
            p2d = self.to_2d(points)
            tri = Delaunay(p2d)
            return trimesh.Trimesh(vertices=points,faces=tri.simplices)

        else:
            raise NotImplementedError


    def export(self,points):

        pass




import numpy as np
from scipy.spatial import ConvexHull, Delaunay
import trimesh
from fractions import Fraction
from sage.all import Matrix, vector, QQ

class SagePlane:

    def __init__(self,params):

        self.a = QQ(params[0]); self.b = QQ(params[1]); self.c = QQ(params[2]); self.d = QQ(params[3])
        self.normal = vector((self.a,self.b,self.c))
        self.vector_rep = vector((self.a,self.b,self.c,self.d))

        self.max_coord=np.argmax(np.abs((self.vector_rep.numpy())))




    def to_2d(self,points,return_max_coord=False):

        ### project inlier points to plane
        ## https://www.baeldung.com/cs/3d-point-2d-plane
        pp = self.project_points_to_plane(points)

        ## take the max coordinate coords away to make the 2d points
        if return_max_coord:
            return np.delete(pp, self.max_coord, axis=1), points[:,self.max_coord]
        else:
            return np.delete(pp,self.max_coord,axis=1)





    def project_points_to_plane(self,points):

        ones = [QQ(1.0)]*points.shape[0]
        ones = np.array(ones,dtype=object).reshape((points.shape[0],1))
        pts = Matrix(QQ,np.hstack((points,ones)))

        k = (self.vector_rep*pts.transpose())/self.normal.norm()

        return Matrix(QQ,points)+(Matrix(self.normal).transpose()*Matrix(k)).transpose()

        # outpoints = []
        # for i,tk in enumerate(k):
        #     outpoints.append(vector(QQ,points[i,:])+self.normal*tk)
        #
        #
        # return outpoints

        ### project inlier points to plane
        ## https://www.baeldung.com/cs/3d-point-2d-plane
        # k = (-self.a * points[:, 0] - self.b * points[:, 1] - self.c * points[:, 2] - self.d) / (
        #             self.a ** 2 + self.b ** 2 + self.c ** 2)
        # # return np.asarray([points[:, 0] + k * self.a, points[:, 1] + k * self.b, points[:, 2] + k * self.c]).transpose()
        # return np.asarray([points[:, 0] + k * self.a, points[:, 1] + k * self.b, points[:, 2] + k * self.c]).transpose()



class ProjectedConvexHull:

    def __init__(self,plane_params,points):

        self.plane_params = plane_params

        self.pyplane = PyPlane(plane_params)


        pp = self.pyplane.project_points_to_plane(points)
        p2d = np.delete(pp, self.pyplane.max_coord, axis=1)

        self.hull = ConvexHull(p2d)

        self.all_points = pp # actually all projected points
        self.hull_points = self.all_points[self.hull.vertices]









class PyPlane:


    def __init__(self,params,inliers=None):

        params = np.array(params,dtype=float)

        self.params = params

        self.a = params[0]; self.b = params[1]; self.c = params[2]; self.d = params[3]
        self.normal = params[:3]

        ## select max coordinate from plane, ie check towards which coordinate plane is oriented:
        self.max_coord = np.argmax(np.abs(self.normal))

        self.inliers = inliers



    def project_points_to_plane_coordinate_system(self,points):

        ### project inlier points to plane
        ## https://www.baeldung.com/cs/3d-point-2d-plane
        # pp = self.project_points_to_plane(points).transpose()
        pp = self.project_points_to_plane(points)

        ## make e1 and e2 (see bottom of page linked above)
        ## take a starting vector (e0) and take a component of this vector which is nonzero (see here: https://stackoverflow.com/a/33758795)
        z = self.max_coord
        y = (z+1)%3
        x = (y+1)%3
        e0 = np.array(self.normal)
        e0 = e0/np.linalg.norm(e0)
        e1 = np.zeros(3)
        ## reverse the non-zero component and put it on a different axis
        e1[x], e1[y], e1[z] = e0[x], -e0[z], e0[y]
        ## take the cross product of e0 and e1 to make e2
        e2 = np.cross(e0,e1)
        e12 = np.array([e1,e2])
        return (e12@pp).transpose()

    def project_points_to_plane(self,points):



        ### project inlier points to plane
        ## https://www.baeldung.com/cs/3d-point-2d-plane
        k = (-self.a * points[:, 0] - self.b * points[:, 1] - self.c * points[:, 2] - self.d) / (
                    self.a ** 2 + self.b ** 2 + self.c ** 2)
        # return np.asarray([points[:, 0] + k * self.a, points[:, 1] + k * self.b, points[:, 2] + k * self.c]).transpose()
        return np.asarray([points[:, 0] + k * self.a, points[:, 1] + k * self.b, points[:, 2] + k * self.c]).transpose()



    def to_2d(self,points,return_max_coord=False):

        ### project inlier points to plane
        ## https://www.baeldung.com/cs/3d-point-2d-plane
        pp = self.project_points_to_plane(points)

        ## take the max coordinate coords away to make the 2d points
        if return_max_coord:
            return np.delete(pp, self.max_coord, axis=1), points[:,self.max_coord]
        else:
            return np.delete(pp,self.max_coord,axis=1)



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




    def get_convex_hull_points_of_projected_points(self,points,dim=2,return_index=False):

        pp = self.project_points_to_plane(points)
        p2d = np.delete(pp, self.max_coord, axis=1)

        ch = ConvexHull(p2d)

        if dim == 2:
            pts = p2d[ch.vertices]
        elif dim == 3:
            pts = pp[ch.vertices]
        else:
            NotImplementedError

        if return_index:
            return pts, ch.vertices
        else:
            return pts


    def get_trimesh_of_projected_points(self,points,type="convex_hull"):

        if type == "convex_hull":
            pp = self.project_points_to_plane(points)
            p2d = np.delete(pp, self.max_coord, axis=1)
            ch = ConvexHull(p2d)
            tri = Delaunay(p2d[ch.vertices,:])
            return trimesh.Trimesh(vertices=pp[ch.vertices],faces=tri.simplices)

        elif type == "all":
            pp = self.project_points_to_plane(points)
            p2d = np.delete(pp, self.max_coord, axis=1)
            tri = Delaunay(p2d)
            return trimesh.Trimesh(vertices=pp,faces=tri.simplices)

        else:
            raise NotImplementedError


    def export(self,points):

        pass




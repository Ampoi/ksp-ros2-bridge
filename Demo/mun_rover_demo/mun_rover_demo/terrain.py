"""Observed elevation cells and conservative ground traversability."""
import math
import numpy as np
from scipy.spatial import cKDTree, Delaunay, QhullError


class Terrain:
    def __init__(self, resolution=0.25, size=120.):
        self.resolution=resolution
        self.size=size
        self.n=int(round(size/resolution))
        self.cells={}
        self.grid=np.full((self.n,self.n),-1,np.int8)
        self.ground=np.empty((0,3)); self.obstacles=np.empty((0,3))
        self.bootstrapped=False

    def index(self, xy):
        return np.floor((np.asarray(xy)+self.size/2)/self.resolution).astype(int)

    def update(self, points):
        points=np.asarray(points,float).reshape(-1,3)
        points=points[np.isfinite(points).all(axis=1)]
        indices=self.index(points[:,:2])
        for p,(x,y) in zip(points,indices):
            if not (0<=x<self.n and 0<=y<self.n): continue
            key=(int(x),int(y))
            # Bounded per-cell samples retain both terrain and positive obstacles.
            values=self.cells.setdefault(key,[])
            values.append(float(p[2]))
            if len(values)>24: del values[0]
        if len(self.cells)<20: return
        keys=np.array(list(self.cells),int)
        centers=(keys+0.5)*self.resolution-self.size/2
        # Batch ragged quantiles; thousands of individual np.quantile calls
        # can starve a 0.5 s sensor watchdog as the explored area grows.
        samples=list(self.cells.values())
        lengths=np.fromiter((len(v) for v in samples),int)
        values=np.full((len(samples),24),np.inf)
        for i,v in enumerate(samples): values[i,:len(v)]=v
        values.sort(axis=1)
        rank=(lengths-1)*0.2; lower=rank.astype(int); fraction=rank-lower
        rows=np.arange(len(samples))
        low=values[rows,lower]*(1-fraction)+values[rows,np.minimum(lower+1,lengths-1)]*fraction
        high=values[rows,lengths-1]
        tree=cKDTree(centers)
        distances,neighbors=tree.query(centers,k=min(16,len(centers)),distance_upper_bound=2.0)
        valid=np.isfinite(distances)
        near=np.minimum(neighbors,len(keys)-1)
        xy=centers[near]-centers[:,None,:]
        A=np.concatenate([xy,np.ones((*xy.shape[:2],1))],axis=2)
        weights=valid.astype(float)
        normal=np.einsum('nki,nkj,nk->nij',A,A,weights)
        rhs=np.einsum('nki,nk,nk->ni',A,low[near],weights)
        coeff=np.linalg.solve(normal+np.eye(3)[None,:,:]*1e-9,rhs[...,None])[...,0]
        residual=low[near]-np.einsum('nki,ni->nk',A,coeff)
        slope=np.arctan(np.linalg.norm(coeff[:,:2],axis=1))
        rough=np.max(np.where(valid,np.abs(residual),0),axis=1)
        discontinuity=np.max(np.where(valid,residual,-np.inf),axis=1)-np.min(np.where(valid,residual,np.inf),axis=1)
        supported=(valid.sum(axis=1)>=6)&(np.linalg.det(normal)>1e-6)
        blocked=(slope>math.radians(12))|(rough>0.15)|(discontinuity>0.25)|(high-low>0.15)
        classes=np.where(supported,np.where(blocked,100,0),-1).astype(np.int8)
        self.grid.fill(-1)
        self.grid[keys[:,1],keys[:,0]]=classes
        # Rasterize short, observed surface triangles. This is bounded spatial
        # interpolation between returns, never ray-clearing no-return beams.
        # Holes wider than 2 m remain unknown; discontinuous/steep triangles
        # cannot become a driveable surface.
        try:
            mesh=Delaunay(centers)
            triangles=centers[mesh.simplices]
            edges=np.maximum.reduce([np.linalg.norm(triangles[:,i]-triangles[:,j],axis=1) for i,j in ((0,1),(1,2),(2,0))])
            good=(edges<=2.0)&np.all(classes[mesh.simplices]==0,axis=1)
            lo=keys.min(0); hi=keys.max(0)+1
            yy,xx=np.mgrid[lo[1]:hi[1],lo[0]:hi[0]]
            samples=np.column_stack([xx.ravel(),yy.ravel()])
            faces=mesh.find_simplex((samples+0.5)*self.resolution-self.size/2)
            supported=(faces>=0)&good[np.maximum(faces,0)]
            valid=samples[supported]
            self.grid[valid[:,1],valid[:,0]]=0
            # Restore explicit obstacle cells after interpolation.
            blocked=keys[classes==100];self.grid[blocked[:,1],blocked[:,0]]=100
        except QhullError:
            pass
        self.ground=np.column_stack([centers[classes==0],low[classes==0]])
        self.obstacles=np.column_stack([centers[classes==100],high[classes==100]])

    def seed_contact_patch(self, pose, geometry):
        """Only the currently occupied, grounded wheel support rectangle can be
        initialized without LiDAR returns (own-vessel occlusion). No future
        road is inferred; this is a one-time stationary contact observation.
        """
        if self.bootstrapped: return
        # Bootstrap exactly the current inflated body footprint. Its support is
        # inferred from stationary ground contacts; no additional road is seeded.
        xs=[p[0]+geometry.rear_x for p in geometry.footprint]; ys=[p[1] for p in geometry.footprint]
        local=np.array([[x,y,-geometry.height] for x in np.arange(min(xs),max(xs)+.1,self.resolution/2)
                         for y in np.arange(min(ys),max(ys)+.1,self.resolution/2)])
        world=local@pose[:3,:3].T+pose[:3,3]
        self.update(world)
        self.bootstrapped=True

    def corridor_safe(self, position, yaw, footprint, speed, curvature, distance=1.):
        """Check the entire swept polygon against obstacles AND unknown cells.
        Current self-occluded cells are exempt only within the contact patch;
        never treat no-return LiDAR rays as free ground.
        """
        poly=np.asarray(footprint,float)
        # Rectangle includes body collision bounds and the configured margin.
        local=np.array([[x,y] for x in np.arange(poly[:,0].min(),poly[:,0].max()+self.resolution/2,self.resolution/2)
                         for y in np.arange(poly[:,1].min(),poly[:,1].max()+self.resolution/2,self.resolution/2)])
        for travel in np.arange(0,distance+self.resolution/2,self.resolution/2):
            angle=yaw+curvature*travel
            if abs(curvature)<1e-5: center=np.asarray(position)+travel*np.array([math.cos(yaw),math.sin(yaw)])
            else: center=np.asarray(position)+np.array([math.sin(angle)-math.sin(yaw),-math.cos(angle)+math.cos(yaw)])/curvature
            c,s=math.cos(angle),math.sin(angle)
            xy=local@np.array([[c,s],[-s,c]])+center
            ij=self.index(xy)
            if np.any(ij<0) or np.any(ij>=self.n): return False
            if np.any(self.grid[ij[:,1],ij[:,0]]!=0): return False
        return True

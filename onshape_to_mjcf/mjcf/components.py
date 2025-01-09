from typing import Dict, List, Tuple, Any,Optional
from dataclasses import dataclass,field
from uuid import uuid4,UUID

from .tree import Part

from enum import Enum
import numpy as np


@dataclass
class Mesh:
    #TODO: maybe make it a path object
    file:str
    name:str
    rgba: List[float]

@dataclass
class Material:
    name:str
    rgba: List[float]

@dataclass
class Geom:
  mesh: str
  name: str = None
  material:Material = None
  rgba : List[float] = None
  g_type: str = "mesh"
  pos: List[float] = None
  euler: List[float] = None
  quat: List[float] = None
  g_class: str = None
  id:UUID = uuid4()

  def json(self):
    geom = {
      "name" : self.name,
      "pos" : self.pos,
      "quat" : self.quat,
      "euler" : self.euler,
      "mesh" : self.mesh,
      "rgba" : self.rgba,
      "material" : self.material.json() if self.material else None,
      "g_type" : self.g_type,
      "g_class" : self.g_class,
      "id" : str(self.id)
    }
    if self.name:
      geom["name"] = self.name
    return geom

@dataclass
class Inertia:
  pos : List[float]
  mass : float
  inertia : List[float]

  def fullinertia(self):
    inertia_mat = np.array(self.inertia).reshape((3,3))
    return [inertia_mat[0,0],inertia_mat[1,1],inertia_mat[2,2],
            inertia_mat[0,1],inertia_mat[0,2],inertia_mat[1,2]]

  def json(self):
    return {
      "pos" :self.pos,
      "mass": self.mass,
      "fullinertia" : self.fullinertia()
    }

@dataclass
class Joint:
  name : str
  j_type : str
  j_range : List[float]
  axis : List[float]
  id : UUID = uuid4()
  j_class : str = None

  def json(self):
    return{
      "name" : self.name,
      "j_type" : self.j_type,
      "j_range" : self.j_range,
      "axis" : self.axis,
      "j_class" : self.j_class,
      "id" : str(self.id)
    }

@dataclass
class Site:

  pos : List[float]
  euler: List[float]
  name : str

  def json(self):
    return{
      "name": self.name,
      "pos" : self.pos,
      "euler":self.euler
    }

@dataclass
class Body:
  inertia : Inertia
  geom : Geom
  name: str = None
  joint : Optional[Joint] = None
  site : Optional[Site] = None
  pos : List[float] = None
  euler : List[float] = None
  quat : List[float] = None
  parent: Optional['Body'] = None
  children: List["Body"] = field(default_factory = list)
  part: Optional[Part] = None

  def json(self):
    body =  {
      "pos" : self.pos,
      "quat" : self.quat,
      "euler" : self.euler,
      "inertia" : self.inertia.json(),
      "geom" : self.geom.json(),
      "joint" : self.joint.json() if self.joint else None ,
      "site" : self.site.json() if self.site else None ,
      "children": [child.json() for child in self.children]
    }
    if self.name:
      body["name"] = self.name
    return body

  def add_body(self,child):
    self.children.append(child)
    child.parent = self
  def add_site(self, site):
    self.site = site

@dataclass
class Connect:
  body1_instances_id:str
  body2_instances_id:str

  body1:str
  body2:str

  anchor:str

  def json(self):
    return {
      'body1': self.body1,
      'body2': self.body2,
      'anchor':self.anchor
    }


@dataclass
class ElementState:
    defaults:dict = None
    elements:Dict[UUID,object] = field(default_factory = dict)
    ids:List[UUID] = field(default_factory = list)
    attirbute_groups:List[dict] = field(default_factory = list)

    def add(self,e,obj):
        id = list(e.keys())[0]
        self.ids.append(id)
        self.attirbute_groups.append(e)
        self.elements[id]=obj

    def get_element(self, id: UUID):
        """
        in : id
        out: pointer to node given i
        """
        return self.elements[id]

@dataclass
class Assets:
    materials: List[Material] = field(default_factory=list)
    meshes:List[Mesh] = field(default_factory=list)

    def add_mesh(self,m):
        if  m not in self.meshes:
            self.meshes.append(m)

    def add_material(self,name,rgba):
        m = Material(name,rgba)
        if m not in self.materials:
            self.materials.append(m)

@dataclass
class MujocoGraphState:
    """
    stores state of the graph:
    This is going to be used for
    1.  default creation:
        If a thing happens more than once then it should be a default class
    2. assets
       - material should be created on first encounter and then reused
       - same is true for mesh

    """
    #all the parts in the tree
    part_list :List[Part] = field(default_factory=list)
    #used for making sure joints have unique names
    joint_names = {}
    # used for defaults management
    joint_state = ElementState()
    geom_state  = ElementState()
    # used for asset management
    assets = Assets()
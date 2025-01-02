from typing import Dict, List, Tuple, Any,Optional
from dataclasses import dataclass,field
from uuid import uuid4,UUID

from .tree import Part

from enum import Enum
import numpy as np

@dataclass
class MaterialD:
  name:str
  rgba: List[float]

  def json(self):
    return {
      "name" : self.name,
      "rgba" : " ".join(map(str, self.rgba)) if self.rgba else "",
    }

@dataclass
class GeomD:
  mesh: str
  name: str = None
  material:MaterialD = None
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
class InertiaD:
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
class JointD:
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
class SiteD:
  size : List[float]
  rgba : List[float]
  pos : List[float]
  name : str
  group : str

  def json(self):
    return{
      "name": self.name,
      "size" : " ".join(map(str, self.size)),
      "rgba" : " ".join(map(str, self.rgba)),
      "pos" : " ".join(map(str, self.pos)),
      "group": self.group
    }

@dataclass
class BodyD:
  inertia : InertiaD
  geom : GeomD
  name: str = None
  joint : Optional[JointD] = None
  site : Optional[SiteD] = None
  pos : List[float] = None
  euler : List[float] = None
  quat : List[float] = None
  parent: Optional['BodyD'] = None
  children: List["BodyD"] = field(default_factory = list)
  part: Optional['BodyD'] = None

  def json(self):
    print(f"BodyD::json::{self.pos}")
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


@dataclass
class ConnectD:
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

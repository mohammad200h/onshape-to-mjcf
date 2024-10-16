from dataclasses import dataclass,field
from typing import Dict, List, Tuple, Any,Optional
from enum import Enum
from uuid import uuid4,UUID

class JointType(Enum):
    REVOLUTE = "revolute"
    FASTENED = "fastened"
    SLIDER = "slider"
    CYLINDRICAL = "cylindrical"
    BALL = "ball"

@dataclass
class JointData:
    assemblyInfo:dict
    feature:dict
    z_axis: List[float]
    name: Optional[str] = None
    j_type: Optional[JointType] = None


@dataclass
class Part:
    unique_id:UUID
    instance_id:str
    occurence:dict
    transform:List[float]
    link_name:str
    joint:Optional[JointData] = None
    parent:Optional['Part'] = None
    relative_pose:List[float] = None
    children:List['Part'] = field(default_factory=list)

    def add_child(self, child:'Part'):
        """Adds a child node to the current node."""
        child.parent = self
        self.children.append(child)
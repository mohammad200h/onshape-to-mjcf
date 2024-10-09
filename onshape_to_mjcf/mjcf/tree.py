from dataclasses import dataclass,field
from typing import Dict, List, Tuple, Any,Optional
from enum import Enum

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
    instance_id:str
    occurence:dict
    transform:List[float]
    joint:Optional[JointData] = None
    parent:Optional['Part'] = None
    children:List['Part'] = field(default_factory=list)

    def add_child(self, child:'Part'):
        """Adds a child node to the current node."""
        child.parent = self
        self.children.append(child)
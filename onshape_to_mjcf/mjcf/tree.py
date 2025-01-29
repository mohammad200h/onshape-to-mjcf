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
class PartBase:
    unique_id: UUID
    instance_id: List[str]
    instance_id_str: str
    link_name: str

    def __post_init__(self):
        # Ensures that children list is initialized properly
        self.children:List[PartBase] = list()

    def add_child(self, child: 'PartBase'):
        """Adds a child node to the current node."""
        child.parent = self
        self.children.append(child)

@dataclass
class Part(PartBase):
    occurence: dict
    transform: List[float]

    # Optional fields should come after required fields
    joint: Optional['JointData'] = None
    parent: Optional['Part'] = None
    relative_pose: Optional[List[float]] = field(default_factory=list)

    def __post_init__(self):
        super().__post_init__()

@dataclass
class Group(PartBase):
    link_name: str
    # this is a part that is part of group but is referenced in relations
    reference_part:Part = Optional[Part]
    parts:List[Part] = field(default_factory = list)

    def __post_init__(self):
        super().__post_init__()

    def add_relation(self, relation: dict):
        self.relations.append(relation)

    def add_part(self,part:Part):
        self.parts.append(part)

    def set_reference_part(self,ref):
        self.reference_part = ref






from .tree import JointData,Part
from .components import (MujocoGraphState,
                        Default,
                        Tree,
                        refactor_joint,
                        refactor_geom,
                        Connect,
                        )
from .util import (addPart,
                   findInstance
)

from .util import(
  find_occurence,
  find_occurrence,
  get_part_relations,
  transform_to_pos_and_euler,
  processPartName,
  getMeshName,
  get_color,
  get_color_name,
  get_inetia_prop,
  compute_inertia,
  get_worldAxisFrame2,
  get_joint_name,
  get_joint_limit2,
  translate_joint_type_to_mjcf
)
from uuid import uuid4,UUID

from .components import(
  Geom,
  Body,
  BodyElements,
  Material,
  Inertia,
  Joint

)

import numpy as np

#Pretty Print
import xml.dom.minidom
from PrettyPrint import PrettyPrintTree


def create_model(client,assembly:dict):
    occurences_in_root = get_part_transforms_and_fetuses(assembly)
    part_instance = occurences_in_root['robot_base']
    occ = find_occurence(assembly,assembly["rootAssembly"]['occurrences'],
                        part_instance
            )
    # print(f"create_model::part_instance::{part_instance}")
    # print(f"create_model::assembly['rootAssembly']['occurrences']::{assembly['rootAssembly']['occurrences']}")
    # print(f"create_model::occ::{occ}")
    mj_state = MujocoGraphState()
    base_part = Part(
        unique_id =uuid4(),
        instance_id = part_instance,
        instance_id_str= part_instance,
        transform = occ['transform'],
        occurence = occ,
        link_name = "base"
    )
    create_parts_tree(client,base_part,part_instance,None,occurences_in_root,assembly,mj_state)

    # tree before looking for closed loop kinematic
    pt = PrettyPrintTree(lambda x: x.children, lambda x: x.part.link_name +" "+x.part.instance_id_str)


    matrix = np.matrix(np.identity(4))
    # base pose
    body_pos = [0]*6
    root_node =  part_trees_to_node(client,base_part,matrix,body_pos,mj_state)

    print(f"root_node::type::{type(root_node)}")
    pt(root_node)

    j_attribiutes_common_in_all_elements,j_classes = refactor_joint(root_node,mj_state)
    g_attribiutes_common_in_all_elements,g_classes = refactor_geom(root_node,mj_state)

    # create super default
    super_joint_default = Default(
        name=None,
        element_type="joint",
        attrbutes = j_attribiutes_common_in_all_elements[1],
        elements = [
            mj_state.joint_state.get_element(id) \
            for id in j_attribiutes_common_in_all_elements[0]
        ]
    )
    super_geom_default = Default(
        name=None,
        element_type="geom",
        attrbutes = g_attribiutes_common_in_all_elements[1],
        elements = [
            mj_state.geom_state.get_element(id) \
            for id in g_attribiutes_common_in_all_elements[0]
        ]
    )
    # creating named defaults
    named_defaults = []
    for j_class, ids_attributes_tuple in  j_classes.items():
        ids,attributes = ids_attributes_tuple
        named_defaults.append(
            Default(
                name=j_class,
                element_type="joint",
                attrbutes = attributes,
                elements = [
                    mj_state.joint_state.get_element(id) \
                    for id in ids
                ]
            )
        )

    for g_class, ids_attributes_tuple in  g_classes.items():
        ids,attributes = ids_attributes_tuple
        named_defaults.append(
            Default(
                name=g_class,
                element_type="geom",
                attrbutes = attributes,
                elements = [
                    mj_state.geom_state.get_element(id) \
                    for id in ids
                ]
            )
        )

    # look_for_closed_kinematic_in_tree logic:
    # if there is a closed kinematic
    # remove repeated link instance caused
    # by closed kinematic.
    # add an equality constraint between remaind repreated instance link
    # and parent of removed instance link
    parts_to_delete,connections = look_for_closed_kinematic_in_tree(base_part,mj_state)



    # print("\n")
    # print(f"parts_to_delete::{parts_to_delete}")
    # print(f"connections::{connections}")
    # print("\n")
    for part_to_delete in parts_to_delete:
      remove_duplicate_connections(root_node,connections,part_to_delete)
    for part_to_delete in parts_to_delete:
      remove_duplicate_from_body_tree(root_node,connections,part_to_delete)

    # cross reference connections with relations
    connections = cross_reference_connections_with_relations(occurences_in_root['relations'],connections)

    pt(root_node)

    # creating tree
    tree = Tree(
        root = root_node,
        equalities = connections,
        super_defaults = [super_joint_default,super_geom_default],
        named_defaults = named_defaults,
        state = mj_state
    )

    # assining classes and cleaning up tree
    tree.refactor()


    xml_str = tree.xml()

    # Parse the XML string
    dom = xml.dom.minidom.parseString(xml_str)

    # Pretty print the XML string
    pretty_xml_as_string = dom.toprettyxml()

    # Remove the XML declaration
    pretty_xml_as_string = '\n'.join(pretty_xml_as_string.split('\n')[1:])
    # print(pretty_xml_as_string)


    file_path = './model.xml'

    with open(file_path, 'w') as file:
        file.write(pretty_xml_as_string)

def get_part_transforms_and_fetuses(assembly:dict):
    # It is possible to get transform of all the parts from root assembly
    # However it is not possible to get the features (mate features from) root assembly
    # Here we will get all the part information form root assembly
    # we will get all the avilable features from root assembly.
    # Then we will dig in the sub-assemblies to get the missing features.

    root = assembly["rootAssembly"]

    # print(f"get_part_transforms_and_fetuses::root::keys::{root.keys()}")
    # print(f"get_part_transforms_and_fetuses::root[occurrences]::{root['occurrences']}")
    # for occ in root['occurrences']:
      # print(f"occ::path::{occ['path']}")

    assembly_info = {
        'fullConfiguration':root['fullConfiguration'],
        'documentId':root['documentId'],
        'assemblyId':root['elementId']
    }

    if len(assembly["subAssemblies"])>0:
        subassemblies = assembly["subAssemblies"]

    occurences_in_root = {
        "robot_base":None,
        "sub-assemblies":{},
        "parts":[],
        # parts in root assenbly that belong to a sub assembly
        # this happens when a mate between subassembly and root assembly is made
        "sub_assembly_parts":[],
        "relations":None
    }

    for idx,occurrence in enumerate(root["occurrences"]):
        # print(f"occurrence::{occurrence}")
        occurrence["instance"] = findInstance(assembly,occurrence["path"])
        typee = occurrence["instance"]['type']
        occurrence["linkName"] = None
        root["occurrences"][idx] = occurrence
        # assume that fixed link is the base
        is_fixed = occurrence['fixed']

        if is_fixed:
          occurences_in_root["robot_base"] = occurrence['instance']['id']

        if typee == "Assembly":
          data = {
            "documentId":occurrence['instance']['documentId'],
            "elementId":occurrence['instance']['elementId']
          }
          occurences_in_root['sub-assemblies'][occurrence['path'][0]] = data
        elif typee == 'Part':
          occurences_in_root['parts'].append(occurrence['path'])

    # recording parts in root assembly that belong to sub assembly
    for subassembly in occurences_in_root['sub-assemblies']:
      for part in occurences_in_root['parts']:
        if subassembly in part :
          part_id = part[:]
          part_id.remove(subassembly)
          data = {
            "assembly":subassembly,
            "part_path":part,
            "part":part_id
          }
          occurences_in_root['sub_assembly_parts'].append(data)

    relations = []
    relations_that_belong_to_assembly = []

    features = root["features"]

    # for feature in features:
      # print("\n")
      # print(f"feature::{feature}")

    ##### getting relations in root assembly #####
    for idx,feature in enumerate(features):
        child = feature['featureData']['matedEntities'][0]['matedOccurrence']
        parent = feature['featureData']['matedEntities'][1]['matedOccurrence']
        # print(f"root::parent::{parent}")
        # print(f"root::child::{child}")
        assemblyInstanceId = None
        if len(child)>1:
            assemblyInstanceId = child[0]
        relation = {
          'child':child,
          'parent':parent,
          'feature':feature,
          'assemblyInfo':assembly_info,
          'assemblyInstanceId':assemblyInstanceId
        }
        # print(f"root_assembly::assemblyInstanceId::{assemblyInstanceId}")

        relations.append(relation)
        # when two ids are in a list one belong to sub assembly
        # the first one represent the assembly
        # the second one represent the part
        child_is_part_of_subassembly = len(child)>1
        if child_is_part_of_subassembly:
            # print(f"assembly_id::child::{child}")
            assembly_id = child[0]
            # print(f"assembly_id::{assembly_id}")
            if assembly_id in occurences_in_root['sub-assemblies'].keys():
              root_part = child[:]
              # root_part.remove(id)
              data = {
                'assemblyInstanceId':assembly_id,
                'assembly':occurences_in_root['sub-assemblies'][assembly_id],
                'relation':relation,
                'assembly_root_part':root_part,
                #This will be filled when going through subassembly features
                'replacement':None

              }

              relations_that_belong_to_assembly.append(data)
    ##### get rest of the relations from sub-assemblies ######
    if len(relations_that_belong_to_assembly)>0:
      for idx,rbs in enumerate(relations_that_belong_to_assembly):
        subassembly_relations = []
        expected_element_id = rbs['assembly']['elementId']
        subassembly_root_part = rbs['assembly_root_part']
        expected_instance_id = rbs['assemblyInstanceId']
        subassembly = None
        for asm in assembly["subAssemblies"]:
          if expected_element_id == asm['elementId']:
            for feature in asm['features']:
              # print("\n")
              # print(f"sub-feature::{feature}")
              # print("\n")
              if feature['featureType'] != 'mateConnector':
                child = feature['featureData']['matedEntities'][0]['matedOccurrence']
                if len(child)>1:
                  assemblyInstanceId = child[0]
                else:
                  parent = feature['featureData']['matedEntities'][1]['matedOccurrence']
                  subassembly_info = assembly_info.copy()
                  subassembly_info['assemblyId']= asm['elementId']
                  relation = {
                    'child':[expected_instance_id] + child,
                    'parent':[expected_instance_id] + parent,
                    'feature':feature,
                    'assemblyInfo':subassembly_info,
                    'assemblyInstanceId':expected_instance_id
                  }
                  # print(f"three::assemblyInstanceId::{expected_instance_id}")
                  subassembly_relations.append(relation)
        relations_that_belong_to_assembly[idx]["replacement"] = subassembly_relations

    # replace relations in root with equivalent sub assemblies
    for rbs in relations_that_belong_to_assembly:
      original_relation = rbs['relation']
      replacement_relations = rbs ['replacement']

      # print("\n")
      # print(f"original_relation::parent::{original_relation['parent']}")
      # print(f"original_relation::child::{original_relation['child']}")
      # print("------")
      # print(f"replacement_relations::{replacement_relations}")
      # print(f"replacement_relations::parent::{replacement_relations['parent']}")
      # print(f"replacement_relations::child::{replacement_relations['child']}")
      # print("\n")

      insert_position = None
      for idx,r in enumerate(relations):
        if (r['child'] == original_relation['child'] and \
            r['parent'] == original_relation['parent']):
            #  record index of previous relation to be removed
            insert_position = idx
            break
      # insert new relations
      relations[insert_position+1:insert_position+1] = replacement_relations
      # correcting relation between assemblies
      # by removing assembly name form relation
      relations[insert_position]['child'] = relations[insert_position]['child']

    # print("get_part_transforms_and_fetuses")
    # print("\n\n")
    # for r in relations:
      # print(f"r:parent::{r['parent']}")
      # print(f"r:child::{r['child']}")
    # print("\n\n")

    occurences_in_root["relations"] = relations
    return occurences_in_root

def part_trees_to_node(client,part,matrix,body_pose,graph_state:MujocoGraphState):
    # print(f"part_trees_to_node::part.transform::type::{type(part.transform)}")
    pose = np.array(part.transform).reshape(4,4)
    pose = np.linalg.inv(matrix)*pose
    xyz,rpy,quat = transform_to_pos_and_euler(pose)

    #adding relative pose to part
    part.relative_pose = body_pose

    # print(f"part_trees_to_node::part.occurence::keys::{part.occurence.keys()}")
    instance = part.occurence["instance"]
    # print(f"part_trees_to_node::instance::keys::{instance.keys()}")
    link_name = part.link_name
    # print(f"link_name::{link_name}")
    # print(f"matrix::\n{matrix}")
    # print(f"pose::\n{pose}")

    justPart, prefix,part_ = getMeshName(part.occurence)

    graph_state.assets.add_mesh(justPart+".stl")

    rgba = get_color(client,part_)

    c_name = get_color_name(rgba)
    graph_state.assets.add_material(c_name,rgba)

    geom = Geom(
        id = uuid4(),
        name = justPart,
        pos = tuple(xyz),
        euler = tuple(rpy),
        mesh = justPart,
        material = Material(
            name = c_name,
            rgba = rgba)
    )


    # getting inertia
    mass,intertia_props,com = get_inetia_prop(client,prefix,part_)
    # print(f"part_trees_to_node::intertia_props::\n{intertia_props}")
    i_prop_dic = compute_inertia(pose,mass,com,intertia_props)
    inertia = Inertia(
        pos= i_prop_dic["com"],
        mass = mass,
        fullinertia=i_prop_dic["inertia"]
    )

    joint= None
    if part.joint and part.joint.j_type.lower() != "fastened":
        joint_name = get_joint_name(part.joint.name,graph_state)
        limits = get_joint_limit2(client,part.joint)
        print(f"limits::{limits}")
        if limits ==None:
          limits = (-3.14,3.14)
        # print(f"part_trees_to_node::part.joint.z_axis::{part.joint.z_axis}")
        # TODO need to apply axis frame to joint
        # seems like mujoco exporter does some kind of math
        # xml_urdf.cc ->  mjXURDF::Joint
        joint = Joint(
                name = joint_name,
                j_type=translate_joint_type_to_mjcf(part.joint.j_type.lower()),
                j_range=limits,
                axis=tuple(part.joint.z_axis),
                id = uuid4()
            )
        graph_state.joint_state.add(joint.to_dict(),joint)

    body_elem = BodyElements(inertia,geom,joint)
    node = Body(prop=body_elem,part =part,name=link_name,position=tuple(body_pose[:3]),euler=tuple(body_pose[3:]))


    for child in part.children:
        worldAxisFrame = get_worldAxisFrame2(child)
        # print(f"worldAxisFrame::\n{worldAxisFrame}")
        axisFrame = np.linalg.inv(matrix)*worldAxisFrame
        childMatrix = worldAxisFrame


        xyz,rpy,quat = transform_to_pos_and_euler(axisFrame)

        child_node = part_trees_to_node(client,child,childMatrix, list(xyz)+list(rpy),graph_state )
        node.add_child(child_node)

    return node

def create_parts_tree(client,root_part:Part, part_instance:str,
                      assemblyInstance:str,
                      occurences_in_root:dict,
                      assembly:dict,
                      graph_state:MujocoGraphState,
                      feature=None
                      ):

    # print("pathes:::")
    # print("\n")
    # for occ in  assembly["rootAssembly"]['occurrences']:
    #   print(f"occ::path::{occ['path']}")
    # print("\n")

    #add mesh file
    addPart(client,root_part)
    # add instance of part in tree to graph_state
    # for record keeping
    graph_state.part_list.append(root_part)
    print(f"part_instance::{part_instance}")
    if isinstance(part_instance,str):
      part_instance = [part_instance]
    relations = get_part_relations(occurences_in_root['relations'],
                part_instance,assemblyInstance
                )
    # for r in relations:
    #   occurences_in_root['relations'].remove(r)

    there_is_a_relation = len(relations)>0
    if there_is_a_relation:
        for relation in relations:
            print(f"create_parts_tree::relation::parent::{relation['parent']}")
            print(f"create_parts_tree::relation::child::{relation['child']}")

            feature = relation['feature']
            assemblyInfo = relation['assemblyInfo']
            assemblyInstanceId = relation['assemblyInstanceId']


            child =  relation['child']
            path = child[0]
            if len(relation['child'])>1:
              path = [assemblyInstanceId]+child[1:]



            # print(f"create_parts_tree::relation['child']::{relation['child']}")
            # print(f"create_parts_tree::assemblyInstanceId::{assemblyInstanceId}")
            # print(f"create_parts_tree::child::{child}")
            print(f"create_parts_tree::path::{path}")


            occ = find_occurrence(assembly["rootAssembly"]['occurrences'],path)
            # when looking onshape-to-robot -> load_robot.py
            # it seems z_axis is hard coded "zAxis": np.array([0, 0, 1])
            # so matter zAxis it will be set to the constant

            print(f"feature::{feature}")
            j = JointData(
                name = feature['featureData']['name'],
                j_type = feature['featureData']['mateType'],
                z_axis = np.array([0, 0, 1]),
                feature = feature,
                assemblyInfo = assemblyInfo
            )

            # print(f"create_parts_tree::occ::keys::{occ.keys()}")
            instance = occ["instance"]
            link_name = processPartName(
                            instance['name'], instance['configuration'],
                            occ['linkName']
            )
            instance_id_str = " ,".join(child) if len(child)>1 else child[0]
            part = Part(
                unique_id =uuid4(),
                instance_id = child,
                instance_id_str = instance_id_str,
                occurence = occ,
                transform = occ['transform'],
                link_name = link_name,
                joint = j,

            )

            create_parts_tree(client,part,child,assemblyInstanceId,
                              occurences_in_root,assembly,graph_state,
                              relation['feature'])
            root_part.add_child(part)
    return


def look_for_closed_kinematic_in_tree(base_part:Part,mj_state:MujocoGraphState):
  """
  get the position of removed duplicate so it can be used for equality constraint
  remained duplicate will be body2
  parent of deleted duplicate will be body1
  pos of deleted duplicate will be anchor value
  <connect anchor="pos of deleted duplicate" body1="link name of parent of deleted duplicated"
  body2="link name of remained duplicate" />
  """
  parts_instance_id = np.array([part.instance_id_str for part in mj_state.part_list])
  parts = [(part.instance_id_str,part.unique_id,part) for part in mj_state.part_list]
  duplicates = []
  visited_instance = []
  for part_instance_id in parts_instance_id:
    if part_instance_id in visited_instance:
      continue

    idxs =  np.where(parts_instance_id == part_instance_id)[0]
    visited_instance.append(part_instance_id)

    if idxs.shape[0]>1:
      # print(f"idxs.shape[0]::{idxs.shape[0]}")
      duplicates.append({
        "instance_id":part_instance_id,
        "instances":  [parts[i] for i in idxs.tolist()]
      })

  parts_to_delete = []
  connections = []
  for duplicate in duplicates:
    # I am deleting all the links excpept
    # body1 is parent of deleted link
    duplicated_instances_uid = [ t[2] for t in duplicate['instances']]
    parts_to_delete += duplicated_instances_uid[1:]

    part_to_keep =  duplicated_instances_uid[0]
    body2 = part_to_keep.link_name
    print(f"part_to_keep::{part_to_keep.instance_id}")


    for pd in parts_to_delete:
      anchor = pd.relative_pose
      body1 = pd.parent.link_name

      if body1 == body2:
        continue

      # add equality information to MjState
      # TODO: Sometime equality contraint for removed links are created
      # Need to deal with this
      connect = Connect(
        body1_instances_id = pd.parent.instance_id,
        body2_instances_id = part_to_keep.instance_id,

        body1  = body1,
        body2  = body2,
        anchor = anchor
      )
      connections.append(connect)
  return parts_to_delete,connections

def remove_duplicate_from_body_tree(root_node:Body,connections,duplicate_part):
  if root_node.part.unique_id == duplicate_part.unique_id:

    # remove node from tree
    parent = root_node.parent
    idx_of_child_to_remove = None
    for idx,child in enumerate(parent.children):
      if child.part.unique_id == duplicate_part.unique_id:
        idx_of_child_to_remove = idx
        break
    del parent.children[idx_of_child_to_remove]

  for child in root_node.children:
    remove_duplicate_from_body_tree(child,connections,duplicate_part)

def remove_duplicate_connections(root_node:Body,connections,duplicate_part):
  if root_node.part.unique_id == duplicate_part.unique_id:
    link_name = root_node.part.link_name
    occ = root_node.part.occurence


    print(f"remove_duplicate_from_body_tree::link_name::{link_name}")
    # print(f"remove_duplicate_from_body_tree::occ::{occ}")

    # remove connection
    connection_to_remove = []
    for c in connections:
      if c.body1 == link_name or c.body2 == link_name:
        connection_to_remove.append(c)
    for c in connection_to_remove:
      connections.remove(c)
  for child in root_node.children:
    remove_duplicate_connections(child,connections,duplicate_part)

def cross_reference_connections_with_relations(relations,connections):
  valid_connections = []
  for c in connections:


    search_term = [c.body1_instances_id[0],c.body2_instances_id[0]]

    print(f"search_term::{search_term}")
    for r in relations:
      current = [r['parent'][0],r['child'][0]]
      print(f"current::{current}")

      if search_term == current:
        valid_connections.append(c)


  print(f"valid_connections::len::{len(valid_connections)}")

  return valid_connections








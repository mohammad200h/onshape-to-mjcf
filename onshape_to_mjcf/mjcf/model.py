from .tree import JointData, Part, Group
from .util import (addPart,
                   findInstance
)

from .mjspc_generator import (model_file,
                              setup_file,
                              utility_file
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
from uuid import uuid4, UUID

from ..onshape_api.config import config

from .components import (
  Geom,
  Inertia,
  Joint,
  Body,
  Connect,
  Material,
  Site,
  MujocoGraphState
)

import numpy as np
import mujoco as mj
import json

import os

#Pretty Print
import xml.dom.minidom
from PrettyPrint import PrettyPrintTree


def create_model(client,assembly:dict):
    # folder structure
    python_pkg_path = config['packageName']
    asset_path = config['packageName'] + "/assets"
    for path in [python_pkg_path, asset_path]:
      if not os.path.exists(path):
        os.makedirs(path)



    occurences_in_root, groups = get_part_transforms_and_fetuses(assembly)


    mj_state = MujocoGraphState()



    base_part = create_parts_tree(client, None, None, None ,
                      occurences_in_root, assembly,mj_state,
                      groups
                      )


    print(f"create_model::base_part::{base_part}")

    # tree before looking for closed loop kinematic
    pt_tree = PrettyPrintTree(lambda x: x.children, lambda x: x.part.link_name +" "+x.part.instance_id_str)
    pt_part = PrettyPrintTree(lambda x: x.children, lambda x: x.link_name +" "+x.instance_id_str)


    pt_part(base_part)

    matrix = np.matrix(np.identity(4))
    # base pose
    body_pos = [0]*6

    ###### MJCF Data Graph #######

    root = part_tree_to_graph(client, base_part,
                       matrix, body_pos, mj_state)
    ###################

    # pt_tree(root)
    ###### Removing duplicated links #######
    parts_to_delete ,connections = look_for_closed_kinematic_in_tree(base_part,mj_state)

    for part_to_delete in parts_to_delete:
      remove_duplicate_connections(root,connections,part_to_delete)

    for part_to_delete in parts_to_delete:
      remove_duplicate_from_body_tree(root,connections,part_to_delete)

    # print("\n\n")
    # pt_tree(root)
    ####### Connections #######
    connections = cross_reference_connections_with_relations(occurences_in_root['relations'],connections)

    ######## Storing Tree Data ##########
    data = {
      "tree":root.json(),
      "equality":[c.json() for c in connections]
    }
    with open(python_pkg_path + '/tree.json','w') as f:
      json.dump(data ,f,indent=2)

    ####### Writing model.py ##########
    with open(python_pkg_path + "/model.py", "w") as f:
      f.write(model_file)

    ####### Writing utility.py ##########
    with open(python_pkg_path + "/utility.py", "w") as f:
      f.write(utility_file)

    ####### Writing __init__.py ##########
    with open(python_pkg_path + "/__init__.py", "w") as f:
      f.write("")

    ####### Writing setup.py ##########
    with open("setup.py", "w") as f:
      f.write(setup_file)

def part_tree_to_graph(client, part,
                      matrix, body_pose, graph_state:MujocoGraphState,
                      body = None):

  if isinstance(part,Group):
    pose = np.array(part.reference_part.transform).reshape(4, 4)
  else:
    pose = np.array(part.transform).reshape(4, 4)
  pose = np.linalg.inv(matrix) * pose
  xyz, rpy, quat = transform_to_pos_and_euler(pose)

  #adding relative pose to part
  if isinstance(part,Group):
    part.reference_part.relative_pose = body_pose
  else:
    part.relative_pose = body_pose


  # instance = part.occurence["instance"]
  link_name = part.link_name

  # geom
  geoms = []
  inertia = None
  if isinstance(part,Group):
    for p in part.parts:
      justPart, prefix, part_ = getMeshName(p.occurence)

      graph_state.assets.add_mesh(justPart + ".stl")

      rgba = get_color(client,part_)

      c_name = get_color_name(rgba)
      graph_state.assets.add_material(c_name, rgba)

      # inertia data
      # TODO Need to rethink inertial as it has to consider multiple geoms
      mass, inertia_props, com = get_inetia_prop(client, prefix, part_)
      i_prop_dic = compute_inertia(pose, mass, com, inertia_props)
      inertia = Inertia(pos = i_prop_dic["com"].tolist(),
                        mass = mass,
                        inertia = i_prop_dic["inertia"])

      # add part as geom
      pose = np.array(p.transform).reshape(4, 4)
      pose = np.linalg.inv(matrix) * pose
      xyz, rpy, quat = transform_to_pos_and_euler(pose)

      geom  = Geom( pos = tuple(xyz), euler = tuple(rpy),
                mesh = justPart, rgba = rgba)
      geoms.append(geom)
  else:
    justPart, prefix, part_ = getMeshName(part.occurence)

    graph_state.assets.add_mesh(justPart + ".stl")

    rgba = get_color(client,part_)

    c_name = get_color_name(rgba)

    # inertia data
    mass, inertia_props, com = get_inetia_prop(client, prefix, part_)
    i_prop_dic = compute_inertia(pose, mass, com, inertia_props)
    inertia = Inertia(pos = i_prop_dic["com"].tolist(),
                        mass = mass,
                        inertia = i_prop_dic["inertia"])

    graph_state.assets.add_material(c_name, rgba)
    geom  = Geom( pos = tuple(xyz), euler = tuple(rpy),
                mesh = justPart, rgba = rgba)
    geoms.append(geom)

  # joint if any
  joint_data = None
  joint = None
  site = None

  if isinstance(part,Group):
    joint_data = part.reference_part.joint
    print(f"part.reference_part::link_name{part.reference_part.link_name}")
    print(f"part.reference_part::joint{part.reference_part.joint}")
  else:
    print(f"part::link_name{part.link_name}")
    joint_data = part.joint

  if joint_data and joint_data.j_type.lower() != "fastened":
    joint_name = get_joint_name(joint_data.name,graph_state)
    limits = get_joint_limit2(client,joint_data)
    j_type = translate_joint_type_to_mjcf(joint_data.j_type.lower())

    if limits == None:
      limits = (-3.14,3.14)
      if j_type == 'ball':
        limits = (0,3.14)

    joint = Joint(name = joint_name, j_range = limits,
                   j_type = j_type,
                   axis = joint_data.z_axis.tolist() )
  elif joint_data and joint_data.j_type.lower() == "fastened":
    joint_name = get_joint_name(joint_data.name,graph_state)
    if "site" in joint_name:
      site = Site(name = joint_name,
                  pos = body_pose[:3],
                  euler = body_pose[3:])
  elif joint_data and joint_data.j_type :
    raise ValueError( joint_data.j_type + " is not supported \n")

  current_body = Body(name = link_name,
              pos = body_pose[:3],
              euler = body_pose[3:],
              geoms = geoms, # TODO need to implement geoms logic
              joint = joint,
              inertia = inertia,
              part = part
              )
  root = None
  if body == None:
    root = current_body
  else:
    if site:
      body.add_site(site)
    else:
      body.add_body(current_body)

  for child_part in part.children:
    worldAxisFrame = get_worldAxisFrame2(child_part)
    axisFrame = np.linalg.inv(matrix) * worldAxisFrame
    childMatrix = worldAxisFrame
    xyz, rpy, quat = transform_to_pos_and_euler(axisFrame)
    part_tree_to_graph(client, child_part,
                    childMatrix, list(xyz) + list(rpy),
                    graph_state, current_body)

  return root

def get_part_transforms_and_fetuses(assembly:dict):
    # It is possible to get transform of all the parts from root assembly
    # However it is not possible to get the features (mate features from) root assembly
    # Here we will get all the part information form root assembly
    # we will get all the avilable features from root assembly.
    # Then we will dig in the sub-assemblies to get the missing features.

    root = assembly["rootAssembly"]


    # for f in root['features']:
    #   print(f"\nfeature::{f}\n")


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
    groups = []
    relations_that_belong_to_assembly = []

    features = root["features"]

    ##### getting relations in root assembly #####
    for idx,feature in enumerate(features):
        if feature['featureType'] =='mateGroup':
          group = {"parts":[e['occurrence'][0] for e in feature['featureData']['occurrences']],
                   "feature":feature
                  }
          print(f"\ngot a group:{feature}\n")
          groups.append(group)
        if not 'matedEntities' in feature['featureData'].keys():
          continue

        child  = feature['featureData']['matedEntities'][0]['matedOccurrence']
        parent = feature['featureData']['matedEntities'][1]['matedOccurrence']
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

    print(f"get_part_transforms_and_fetuses::relations::{relations}")
    occurences_in_root["relations"] = relations
    return occurences_in_root,groups



def create_parts_tree(client, root_part:Part, part_instance:str,
                      assemblyInstance:str,
                      occurences_in_root:dict,
                      assembly:dict,
                      graph_state:MujocoGraphState,
                      groups = [],
                      feature = None
                      ):

    # Initialization
    if root_part == None :
      part_instance = occurences_in_root['robot_base']
      for g in groups:
        if part_instance in g['parts']:
          root_part = Group(
            unique_id = uuid4(),
            link_name = "base",
            instance_id = [part_instance],
            instance_id_str = part_instance,
          )
          for part_id in g["parts"]:
            # need to figure out occ
            path = part_id
            occ = find_occurrence(assembly["rootAssembly"]['occurrences'],path)
            print(f"create_parts_tree::Initialization::occ::{occ}")
            p = Part(
              unique_id = uuid4(),
              instance_id = [part_id],
              instance_id_str = part_id,
              transform = occ['transform'],
              occurence = occ,
              link_name = "base_" + part_id
            )
            root_part.add_part(p)
            if part_id == part_instance:
              root_part.set_reference_part(p)

      # part was not part of any group
      # so it was not set
      if root_part == None:
        path = part_instance
        # need to figure out occ
        occ = find_occurrence(assembly["rootAssembly"]['occurrences'],path)
        root_part = Part(
            unique_id = uuid4(),
            instance_id = [part_instance],
            instance_id_str = part_instance,
            transform = occ['transform'],
            occurence = occ,
            link_name = "base"
        )


    # add mesh file
    if isinstance(root_part,Group):
      for p in root_part.parts:
        addPart(client,p)
    else:
      addPart(client,root_part)

    # add instance of part in tree to graph_state
    # for record keeping
    graph_state.part_list.append(root_part)

    if isinstance(part_instance,str):
      part_instance = [part_instance]
    relations = get_part_relations(occurences_in_root['relations'],
                groups, part_instance, assemblyInstance
                )

    print(f"create_parts_tree::occurences_in_root[relations]::{occurences_in_root['relations']}")
    print(f"create_parts_tree::relations::{relations}")

    there_is_a_relation = len(relations) > 0
    if there_is_a_relation:
        print("there_is_a_relation!")
        for relation in relations:
            feature = relation['feature']
            assemblyInfo = relation['assemblyInfo']
            assemblyInstanceId = relation['assemblyInstanceId']

            child =  relation['child']

            print(f"create_parts_tree::child::{child}")
            path = child[0]
            # this part belongs to sub assembly
            if len(relation['child'])>1:
              path = [assemblyInstanceId]+child[1:]

            if relation['child_is_part_of_group']:
              print("relation[child_is_part_of_group]")
              group = Group()
              for part_id in relation["child_group"]:
                # TODO: create part for each instance id in group

                print(f"group::part_id::{part_id}")

                occ = find_occurrence(assembly["rootAssembly"]['occurrences'],path)
                instance = occ["instance"]
                link_name = processPartName(
                              instance['name'], instance['configuration'],
                              occ['linkName'])

                # link_name = TODO
                # occ = TODO
                # part = Part(
                  # unique_id =uuid4(),
                  # instance_id = TODO ,
                  # instance_id_str = TODO ,
                  # occurence = occ,
                  # transform = occ['transform'],
                  # link_name = link_name,
                  # joint = None,
                # )
                # TODO: add parts to group
                group.add_part(part)
                # add group to root part
              root_part.add_child(group)

            else:
              print("child is a simple part and not group")

              occ = find_occurrence(assembly["rootAssembly"]['occurrences'],path)
              # when looking onshape-to-robot -> load_robot.py
              # it seems z_axis is hard coded "zAxis": np.array([0, 0, 1])
              # so no matter, zAxis will be set to the constant

              j = JointData(
                  name = feature['featureData']['name'],
                  j_type = feature['featureData']['mateType'],
                  z_axis = np.array([0, 0, 1]),
                  feature = feature,
                  assemblyInfo = assemblyInfo
              )

              instance = occ["instance"]
              link_name = processPartName(
                              instance['name'], instance['configuration'],
                              occ['linkName']
              )
              instance_id_str = " ,".join(child) if len(child) > 1 else child[0]
              part = Part(
                  unique_id =uuid4(),
                  instance_id = child,
                  instance_id_str = instance_id_str,
                  occurence = occ,
                  transform = occ['transform'],
                  link_name = link_name,
                  joint = j
              )

              create_parts_tree(client,part,child,assemblyInstanceId,
                                occurences_in_root,assembly,graph_state,groups,
                                relation['feature'])
              root_part.add_child(part)
    return root_part

def look_for_closed_kinematic_in_tree(base_part:Part, mj_state:MujocoGraphState):
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
      duplicates.append({
        "instance_id":part_instance_id,
        "instances":  [parts[i] for i in idxs.tolist()]
      })

  parts_to_delete = []
  connections = []
  for duplicate in duplicates:
    # I am deleting all the links except
    # body1 is parent of deleted link
    duplicated_instances_uid = [ t[2] for t in duplicate['instances']]
    parts_to_delete += duplicated_instances_uid[1:]

    part_to_keep =  duplicated_instances_uid[0]
    body2 = part_to_keep.link_name

    for pd in parts_to_delete:
      anchor = pd.relative_pose[:3]
      body1 = pd.parent.link_name

      if body1 == body2:
        continue

      # add equality information to MjState
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

    for r in relations:
      current = [r['parent'][0],r['child'][0]]

      if search_term == current:
        valid_connections.append(c)

  if len(valid_connections)>0:
    return valid_connections
  return connections








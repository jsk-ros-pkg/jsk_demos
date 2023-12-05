#!/usr/bin/env python
#
# Copyright (c) 2011, Lars Kunze
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright 
#    notice, this list of conditions and the following disclaimer in the 
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of the Intelligent Autonomous Systems Group/
#    Technische Universitaet Muenchen nor the names of its contributors 
#    may be used to endorse or promote products derived from this software 
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import string
import time
import sys
import getopt
import yaml
import math

# global varibles
now = 0.0
vert = False
scale = 1.0
room_height = 4.5
spot_dim = 0.2
id = 0

### UTILS ###

def get_id():
    global id
    id = id + 1
    return id

def quaternion_to_rotation_mat3d(q, t):
    global scale
    m = [ [1, 0, 0, 0],
          [0, 1, 0, 0],
          [0, 0, 1, 0],
          [0, 0, 0, 1]]

    x = q['x'] 
    y = q['y'] 
    z = q['z'] 
    w = q['w'] 

    #rotation matrix
    #first row
    m[0][0] = 1 - 2 * y * y - 2 * z * z
    m[0][1] = 2 * x * y - 2 * w * z
    m[0][2] = 2 * x * z + 2 * w * y
  
    # second row 
    m[1][0] = 2 * x * y +  2 * w * z
    m[1][1] = 1 - 2 * x * x - 2 * z * z
    m[1][2] = 2 * y * z - 2 * w * x
  
    # third row
    m[2][0] = 2 * x * z - 2 * w * y
    m[2][1] = 2 * y * z + 2 * w * x
    m[2][2] = 1 - 2 * x * x - 2 * y * y

    # transposed matrix!
    # rotation matrix
    # first row
    # m[0][0] = 1 - 2 * y * y - 2 * z * z
    # m[0][1] = 2 * x * y + 2 * w * z
    # m[0][2] = 2 * x * z - 2 * w * y
  
    # # second row 
    # m[1][0] = 2 * x * y - 2 * w * z
    # m[1][1] = 1 - 2 * x * x - 2 * z * z
    # m[1][2] = 2 * y * z + 2 * w * x
  
    # # third row
    # m[2][0] = 2 * x * z + 2 * w * y
    # m[2][1] = 2 * y * z - 2 * w * x
    # m[2][2] = 1 - 2 * x * x - 2 * y * y

    # translation
    m[0][3] = get_x(t)
    m[1][3] = get_y(t)
    m[2][3] = get_z(t)
    
    return m


### YAML ACCESSOR FUNCTIONS ###

def get_name(name):
    n = name.replace('/','-')
    if n[0] is '-':
        return n[1:]
    return n
    
def get_room_number(name):
    rn = name[name.rfind('-') + 1:]
    return rn

def get_floor_number(name):
    n = name[name.rfind('-') + 1:]
    fn = n.rstrip(string.ascii_letters)
    return fn

def get_parent(frame):
   return frame['frame_id']

def get_rotation(frame):
    return frame['rotation']

def get_translation(frame):
    return frame['translation']

def get_x(coord):
    global scale
    return coord['x'] * scale

def get_y(coord):
    global scale
    return coord['y'] * scale

def get_z(coord):
    global scale 
    return coord['z'] * scale

def get_types(frame):
    return frame['type']

def get_obj_properties(frame):
    if frame.has_key('object-properties'):
        return frame['object-properties']
    return None

def get_data_properties(frame):
    if frame.has_key('data-properties'):
        return frame['data-properties']
    return None

# get actual depth, width, and height
def get_width(r):
    global scale;
    return r['width'] * scale
    
def get_height(r):
    global scale
    return r['height'] * scale
    
def get_depth(r):
    global scale
    return r['depth'] * scale


### OWL stuff ###    

def create_header():
    print('''<?xml version="1.0"?>

<!DOCTYPE rdf:RDF [
    <!ENTITY local_path "file://@OWL_PATH_PREFIX@/owl/">
    <!ENTITY owl "http://www.w3.org/2002/07/owl#" >
    <!ENTITY owl2 "http://www.w3.org/2006/12/owl2#" >
    <!ENTITY xsd "http://www.w3.org/2001/XMLSchema#" >
    <!ENTITY owl2xml "http://www.w3.org/2006/12/owl2-xml#" >
    <!ENTITY rdfs "http://www.w3.org/2000/01/rdf-schema#" >
    <!ENTITY rdf "http://www.w3.org/1999/02/22-rdf-syntax-ns#" >
    <!ENTITY protege "http://protege.stanford.edu/plugins/owl/protege#" >
    <!ENTITY knowrob "http://ias.cs.tum.edu/kb/knowrob.owl#" >
    <!ENTITY jsk_map "http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#" >
]>

<rdf:RDF xmlns="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#"
     xml:base="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl"
     xmlns:rdfs="http://www.w3.org/2000/01/rdf-schema#"
     xmlns:protege="http://protege.stanford.edu/plugins/owl/protege#"
     xmlns:owl2xml="http://www.w3.org/2006/12/owl2-xml#"
     xmlns:owl="http://www.w3.org/2002/07/owl#"
     xmlns:xsd="http://www.w3.org/2001/XMLSchema#"
     xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
     xmlns:owl2="http://www.w3.org/2006/12/owl2#"
     xmlns:knowrob="http://ias.cs.tum.edu/kb/knowrob.owl#">
    <owl:Ontology rdf:about="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#">
      <owl:imports rdf:resource="&local_path;knowrob.owl"/>
    </owl:Ontology>
    
 ''')
    
def create_footer():
    print('''
</rdf:RDF>

<!-- Generated by yaml2owl.py -->
''')

def create_timepoint(t):
    print('')
    print('<owl:NamedIndividual rdf:about="&jsk_map;timepoint_{0}">'.format(t))
    print('  <rdf:type rdf:resource="&knowrob;TimePoint"/>')
    print('</owl:Na1medIndividual>')
    print('')

def create_instance(inst, type):
    print('')
    print('<owl:NamedIndividual rdf:about="&jsk_map;{0}">'.format(inst))
    print('  <rdf:type rdf:resource="http://ias.cs.tum.edu/kb/knowrob.owl#{0}"/>'.format(type))
    print('</owl:NamedIndividual>')
    print('')

def create_spot(inst,types,width,height,depth, x, y, z, props=None):
    global map_inst
    print('')
    print('<owl:NamedIndividual rdf:about="&jsk_map;{0}">'.format(inst))
    for t in types:
        create_type(t)
    print('  <knowrob:widthOfObject rdf:datatype="&xsd;float">{0}</knowrob:widthOfObject>'.format(width))
    print('  <knowrob:depthOfObject rdf:datatype="&xsd;float">{0}</knowrob:depthOfObject>'.format(depth))
    print('  <knowrob:heightOfObject rdf:datatype="&xsd;float">{0}</knowrob:heightOfObject>'.format(height))
    print('  <knowrob:describedInMap rdf:resource="&jsk_map;SemanticEnvironmentMap-{0}"/>'.format(map_inst))
        
    if props is not None:
        for p in props:
            create_prop(p[0], get_name(p[1]))
    print('</owl:NamedIndividual>')
    print('')
    
def create_object(inst,types,width,height,depth, prop=None, objs=None, x=0.0, y=0.0, z=0.0, data_props=None):
    global map_inst
    print('')
    print('<owl:NamedIndividual rdf:about="&jsk_map;{0}">'.format(inst))
    for t in types:
        create_type(t)
    print('  <knowrob:widthOfObject rdf:datatype="&xsd;float">{0}</knowrob:widthOfObject>'.format(width))
    print('  <knowrob:depthOfObject rdf:datatype="&xsd;float">{0}</knowrob:depthOfObject>'.format(depth))
    print('  <knowrob:heightOfObject rdf:datatype="&xsd;float">{0}</knowrob:heightOfObject>'.format(height))
    print('  <knowrob:describedInMap rdf:resource="&jsk_map;SemanticEnvironmentMap-{0}"/>'.format(map_inst))

    if objs is not None:
        if objs.has_key(inst):
            create_props(prop, objs[inst])

    if data_props is not None:
        for p in data_props:
            create_data_prop(p[0], p[1], p[2])
            
    print('</owl:NamedIndividual>')
    print('')
    
def create_room(inst,types,width,height,depth, prop=None, objs=None, x=0.0, y=0.0, z=0.0):
    global map_inst
    print('')
    print('<owl:NamedIndividual rdf:about="&jsk_map;{0}">'.format(inst))
    for t in types:
        create_type(t)
    print('  <knowrob:widthOfObject rdf:datatype="&xsd;float">{0}</knowrob:widthOfObject>'.format(width))
    print('  <knowrob:depthOfObject rdf:datatype="&xsd;float">{0}</knowrob:depthOfObject>'.format(depth))
    print('  <knowrob:heightOfObject rdf:datatype="&xsd;float">{0}</knowrob:heightOfObject>'.format(height))
    print('  <knowrob:describedInMap rdf:resource="&jsk_map;SemanticEnvironmentMap-{0}"/>'.format(map_inst))

    print('  <knowrob:roomNumber rdf:datatype="&xsd;string">{0}</knowrob:roomNumber>'.format(get_room_number(inst)))
        
    if objs is not None:
        if objs.has_key(inst):
            create_props(prop, objs[inst])
    print('</owl:NamedIndividual>')
    print('')

def create_level(inst,types,width,height,depth, prop=None, objs=None, x=0.0, y=0.0, z=0.0):
    global map_inst
    print('')
    print('<owl:NamedIndividual rdf:about="&jsk_map;{0}">'.format(inst))
    for t in types:
        create_type(t)
    print('  <knowrob:widthOfObject rdf:datatype="&xsd;float">{0}</knowrob:widthOfObject>'.format(width))
    print('  <knowrob:depthOfObject rdf:datatype="&xsd;float">{0}</knowrob:depthOfObject>'.format(depth))
    print('  <knowrob:heightOfObject rdf:datatype="&xsd;float">{0}</knowrob:heightOfObject>'.format(height))
    print('  <knowrob:describedInMap rdf:resource="&jsk_map;SemanticEnvironmentMap-{0}"/>'.format(map_inst))

    print('  <knowrob:floorNumber rdf:datatype="&xsd;string">{0}</knowrob:floorNumber>'.format(get_floor_number(inst)))
        
    if objs is not None:
        if objs.has_key(inst):
            create_props(prop, objs[inst])
    print('</owl:NamedIndividual>')
    print('')

    
def create_type(type):
    print('  <rdf:type rdf:resource="&knowrob;{0}"/>'.format(type))

def create_bldg(inst,types,width,height,depth, prop=None, objs=None, x=0.0, y=0.0, z=0.0):
    global map_inst
    print('<owl:NamedIndividual rdf:about="&jsk_map;{0}">'.format(inst))
    for t in types:
        create_type(t)
    print('  <knowrob:widthOfObject rdf:datatype="&xsd;float">{0}</knowrob:widthOfObject>'.format(width))
    print('  <knowrob:depthOfObject rdf:datatype="&xsd;float">{0}</knowrob:depthOfObject>'.format(depth))
    print('  <knowrob:heightOfObject rdf:datatype="&xsd;float">{0}</knowrob:heightOfObject>'.format(height))
    print('  <knowrob:describedInMap rdf:resource="&jsk_map;SemanticEnvironmentMap-{0}"/>'.format(map_inst))
    if objs is not None:
        if objs.has_key(inst):
            create_props(prop, objs[inst])
    print('</owl:NamedIndividual>')    
    
def create_props(prop, objs):
    for o in objs:
        create_prop(prop,o)
    
def create_prop(prop, obj):
    print('  <knowrob:{0} rdf:resource="&jsk_map;{1}"/>'.format(prop,obj))

def create_data_prop(prop, type, val):
    print('  <knowrob:{0} rdf:datatype="&xsd;{1}">{2}</knowrob:{0}>'.format(prop,type,val))
    
def create_perception_event(obj,mat3d_name,time=0):
    print('')
    print('<owl:NamedIndividual rdf:about="&jsk_map;SemanticMapPerception-{0}">'.format(get_id()))
    print('  <rdf:type rdf:resource="&knowrob;SemanticMapPerception"/>')
    print('  <knowrob:objectActedOn rdf:resource="&jsk_map;{0}"/>'.format(obj))
    print('  <knowrob:eventOccursAt rdf:resource="&jsk_map;{0}"/>'.format(mat3d_name))
    print('  <knowrob:startTime rdf:resource="&jsk_map;timepoint_{0}"/>'.format(time))
    print('</owl:NamedIndividual>')
    print('')
    

def create_rot_mat3d(m):
    name = 'RotationMatrix3D-{0}'.format(get_id())
    print('')
    print('<owl:NamedIndividual rdf:about="&jsk_map;{0}">'.format(name))
    print('  <rdf:type rdf:resource="&knowrob;RotationMatrix3D"/>')
    print('  <knowrob:m00 rdf:datatype="&xsd;float">{0}</knowrob:m00>'.format(m[0][0]))
    print('  <knowrob:m01 rdf:datatype="&xsd;float">{0}</knowrob:m01>'.format(m[0][1]))
    print('  <knowrob:m02 rdf:datatype="&xsd;float">{0}</knowrob:m02>'.format(m[0][2]))
    print('  <knowrob:m03 rdf:datatype="&xsd;float">{0}</knowrob:m03>'.format(m[0][3]))
    print('  <knowrob:m10 rdf:datatype="&xsd;float">{0}</knowrob:m10>'.format(m[1][0]))
    print('  <knowrob:m11 rdf:datatype="&xsd;float">{0}</knowrob:m11>'.format(m[1][1]))
    print('  <knowrob:m12 rdf:datatype="&xsd;float">{0}</knowrob:m12>'.format(m[1][2]))
    print('  <knowrob:m13 rdf:datatype="&xsd;float">{0}</knowrob:m13>'.format(m[1][3]))
    print('  <knowrob:m20 rdf:datatype="&xsd;float">{0}</knowrob:m20>'.format(m[2][0]))
    print('  <knowrob:m21 rdf:datatype="&xsd;float">{0}</knowrob:m21>'.format(m[2][1]))
    print('  <knowrob:m22 rdf:datatype="&xsd;float">{0}</knowrob:m22>'.format(m[2][2]))
    print('  <knowrob:m23 rdf:datatype="&xsd;float">{0}</knowrob:m23>'.format(m[2][3]))
    print('  <knowrob:m30 rdf:datatype="&xsd;float">{0}</knowrob:m30>'.format(m[3][0]))
    print('  <knowrob:m31 rdf:datatype="&xsd;float">{0}</knowrob:m31>'.format(m[3][1]))
    print('  <knowrob:m32 rdf:datatype="&xsd;float">{0}</knowrob:m32>'.format(m[3][2]))
    print('  <knowrob:m33 rdf:datatype="&xsd;float">{0}</knowrob:m33>'.format(m[3][3]))
    print('</owl:NamedIndividual>')
    print('')
    return name

### CREATING OWL FROM YAML ###

def create_levels(map, has_rooms, now):
    global room_height
    
    # read floors
    # create floors with rooms 
    has_floors = dict()
    floors = map.get('floor')
    if floors is not None:
        for f in floors:

            name = get_name(f)
            parent = get_name(get_parent(floors[f]))
            
            # remember the parent (building) of the floor, in order to add the hasLevels property to the building
            if not has_floors.has_key(parent):
                has_floors[parent] = [name] 
            else:
                has_floors[parent] = has_floors[parent] + [name]
                

            q = get_rotation(floors[f])
            trans = get_translation(floors[f])
            trans_cpy = trans.copy()

            if vert:
                trans_cpy['x'] = get_width(floors[f])/2 #trans['z']
                trans_cpy['z'] = (float(get_floor_number(name))-1.0) * room_height  + trans['z']             
                
            mat3d = quaternion_to_rotation_mat3d(q,trans_cpy)

            create_level(name,
                         ['LevelOfAConstruction'],
                         get_depth(floors[f]), get_height(floors[f]), get_width(floors[f]),
                         'hasRooms', has_rooms,
                         get_x(trans), get_y(trans), get_z(trans))
            
            mat_inst = create_rot_mat3d(mat3d)
                
            create_perception_event(name, mat_inst, now)
            
    return has_floors


def create_rooms(map, now):
    global room_height
    # read rooms
    has_rooms = dict()
    rooms = map.get('room')
    if rooms is not None:
        for r in rooms:

            name = get_name(r)
            parent = get_name(get_parent(rooms[r]))
            
                # remember the parent (floor) of the room, in order to add the hasRooms property to the floor
            if not has_rooms.has_key(parent):
                has_rooms[parent] = [name] 
            else:
                has_rooms[parent] = has_rooms[parent] + [name]
                
            q = get_rotation(rooms[r])
            trans = get_translation(rooms[r])
            trans_cpy = trans.copy()
                
            if vert:
                trans_cpy['x'] = trans['x'] - get_translation((map.get('floor'))[get_parent(rooms[r])])['x'] +  get_width( (map.get('floor'))[get_parent(rooms[r])])/2
                trans_cpy['z'] = (float(get_floor_number(get_name(get_parent(rooms[r])))) - 1.0) * room_height + trans['z'] #['x']

            mat3d = quaternion_to_rotation_mat3d(q,trans_cpy)

            # switched width and height !!!!
            create_room(name,
                        get_types(rooms[r]),  #['RoomInAConstruction'],
                        get_depth(rooms[r]), get_height(rooms[r]), get_width(rooms[r]),
                        None, None,  get_x(trans), get_y(trans), get_z(trans))

            mat_inst = create_rot_mat3d(mat3d)

            create_perception_event(name, mat_inst, now)

    return has_rooms


def create_objs(map, now):
    has_objs = dict()
    objs = map.get('object')
    if objs is not None:
        for o in objs:
            
            name =   get_name(o)
            parent = get_name(get_parent(objs[o]))

                        
            # remember the parent (room) of the obj, in order to add the hasObjs property to the room
            if not has_objs.has_key(parent):
                has_objs[parent] = [name] 
            else:
                has_objs[parent] = has_objs[parent] + [name]
                
            q = get_rotation(objs[o])
            trans = get_translation(objs[o])
            trans_cpy = trans.copy()

            if vert:
                if map.get('room').has_key(get_parent(objs[o])):
                    trans_cpy['x'] = trans['x'] -  get_translation( (map.get('floor'))[ get_parent((map.get('room'))[get_parent(objs[o])]) ])['x'] + get_width((map.get('floor'))[ get_parent((map.get('room'))[get_parent(objs[o])])])/2
                    trans_cpy['z'] = (float(get_floor_number(get_name(get_parent( (map.get('room'))[get_parent(objs[o])]))))-1.0) * room_height + trans['z'] # ['x']
                elif map.get('floor').has_key(get_parent(objs[o])):
                    trans_cpy['x'] = trans['x'] -  get_translation( (map.get('floor'))[ get_parent(objs[o])])['x'] +  get_width( (map.get('floor'))[get_parent(objs[o])])/2
                    trans_cpy['z'] = (float(get_floor_number(get_name(get_parent((objs[o]))))) - 1.0) * room_height + trans['z'] # ['x']
                else: # all objects need parent!!!
                    break
                    
            mat3d = quaternion_to_rotation_mat3d(q,trans_cpy)

            data_props = get_data_properties(objs[o])

            create_object(name,get_types(objs[o]),get_depth(objs[o]),get_height(objs[o]),get_width(objs[o]), None, None,  get_x(trans), get_y(trans), get_z(trans), data_props)

            mat_inst = create_rot_mat3d(mat3d)
            
            create_perception_event(name, mat_inst, now)

def create_spots(map, now):
    global spot_dim
    spots = map.get('spots')
    if spots is not None:
        for s in spots:
            
            name =   get_name(s)
                        
            q = get_rotation(spots[s])
            trans = get_translation(spots[s])
            trans_cpy = trans.copy()

            if vert:
                if map.get('room').has_key(get_parent(spots[s])):
                    trans_cpy['x'] = trans['x'] -  get_translation( (map.get('floor'))[ get_parent((map.get('room'))[get_parent(spots[s])]) ])['x'] + get_width((map.get('floor'))[ get_parent((map.get('room'))[get_parent(spots[s])])])/2
                    trans_cpy['z'] = (float(get_floor_number(get_name(get_parent( (map.get('room'))[get_parent(spots[s])]))))-1.0) * room_height + trans['z'] # ['x']
                elif map.get('floor').has_key(get_parent(spots[s])):
                    trans_cpy['x'] = trans['x'] -  get_translation( (map.get('floor'))[ get_parent(spots[s])])['x'] +  get_width( (map.get('floor'))[get_parent(spots[s])])/2
                    trans_cpy['z'] = (float(get_floor_number(get_name(get_parent((spots[s]))))) - 1.0) * room_height + trans['z'] # ['x']
                else: # all objects need parent!!!
                    break
                
            mat3d = quaternion_to_rotation_mat3d(q,trans_cpy)

            props = get_obj_properties(spots[s])
            
            create_spot(name,['Place'],spot_dim,spot_dim/2,spot_dim, get_x(trans), get_y(trans), get_z(trans), props)

            mat_inst = create_rot_mat3d(mat3d)
            
            create_perception_event(name, mat_inst, now)
            

def create_map(map, now):
    global map_inst
    # read building, because building name become map name
    buildings = map.get('building')
    if buildings is not None:
        b = buildings.keys()
        map_inst = get_name(b[0])
        create_instance('SemanticEnvironmentMap-' + map_inst,
                        'SemanticEnvironmentMap')

def create_building(map, now, has_floors):    
    # read building
    buildings = map.get('building')
    if buildings is not None:
        b = buildings.keys()
        name  = get_name(b[0])
        q = get_rotation(buildings[b[0]])
        trans = get_translation(buildings[b[0]])
        trans_cpy = trans.copy()
        
        if vert:
            trans_cpy['x'] = get_width(buildings[b[0]])/2 #trans['z'] +
            trans_cpy['z'] = trans['x'] # maximoiun level numnber * room_height   
                
        mat3d = quaternion_to_rotation_mat3d(q,trans_cpy)
            
        create_bldg(name,
                    get_types(buildings[b[0]]), #['Building'],
                    get_depth(buildings[b[0]]),
                    get_height(buildings[b[0]]),
                    get_width(buildings[b[0]]),
                    'hasLevels', has_floors,
                    get_x(trans), get_y(trans), get_z(trans))

        mat_inst = create_rot_mat3d(mat3d)
            
        create_perception_event(name, mat_inst, now)

        
        
    
#______________________________________________________________________________
# main  

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def help_msg():
    return """
  Usage: yaml2owl.py [-h] <map>

    map               name of the yaml file 

    -h, --help  for seeing this msg
    -v, --vert  for creating the map vertically
    -s, --scale for scaling the map

"""

def main(argv=None):
    global map_inst, vert, scale, now
    if argv is None:
        argv = sys.argv
    try:
        try:
            opts, args = getopt.getopt(argv[1:], "svh", ["scale", "vert", "help"])
        except getopt.error, msg:
            raise Usage(msg)

        if ('-h','') in opts or ('--help', '') in opts: #len(args) != 2 or
            raise Usage(help_msg())

        if ('-v','') in opts or ('--vert', '') in opts:
            vert = True

        if ('-s','') in opts or ('--scale', '') in opts:
            scale = 0.1

        # read yaml file
        stream = file(args[0],'r')
        map = yaml.load(stream)

        now = int(time.time())

        create_header()

        create_map(map,now)

        has_rooms = create_rooms(map, now)

        has_floors = create_levels(map, has_rooms, now)

        create_objs(map, now)

        create_building(map, now, has_floors)

        create_spots(map,now)
    
        create_footer()

    except Usage, err:
        print(err.msg, file=sys.stderr)
        print("for help use --help", file=sys.stderr)
        return 2
    
if __name__ == "__main__":
    sys.exit(main())

# deprecated 
#
# def quaternion_to_rotation_mat3d(q, t, r):
#     global scale
#     m = [ [1, 0, 0, 0],
#           [0, 1, 0, 0],
#           [0, 0, 1, 0],
#           [0, 0, 0, 1]]

#     x = q['x'] 
#     y = q['y'] 
#     z = q['z'] 
#     w = q['w'] 

#     # rotation matrix
#     # first row
#     m[0][0] = 1 - 2 * y * y - 2 * z * z
#     m[0][1] = 2 * x * y + 2 * w * z
#     m[0][2] = 2 * x * z - 2 * w * y
  
#     # # # second row 
#     m[1][0] = 2 * x * y - 2 * w * z
#     m[1][1] = 1 - 2 * x * x - 2 * z * z
#     m[1][2] = 2 * y * z + 2 * w * x
  
#     # # # third row
#     m[2][0] = 2 * x * z + 2 * w * y
#     m[2][1] = 2 * y * z - 2 * w * x
#     m[2][2] = 1 - 2 * x * x - 2 * y * y

#     # # fourth row 
#     # m[3][0] = 0
#     # m[3][1] = 0
#     # m[3][2] = 0

#     # region should always be None now!
#     if r is None:
#         m[0][3] = get_x(t)
#         m[1][3] = get_y(t)
#         m[2][3] = get_z(t)
#         m[3][3] = 1
#         return m

#     # calculate the center of room (2D only)
#     xs = [xval[0] for xval in r]
#     ys = [yval[1] for yval in r]
    
#     cent_x = (math.fsum(xs) / float(len(xs))) * scale 
#     cent_y = (math.fsum(ys) / float(len(ys))) * scale
#     cent_z = 0

#     # rotate the center point
#     #rot_x = m[0][0] * cent_x + m[0][1] * cent_y + m[0][2] * cent_z 
#     #rot_y = m[1][0] * cent_x + m[1][1] * cent_y + m[1][2] * cent_z
#     #rot_z = m[2][0] * cent_x + m[2][1] * cent_y + m[2][2] * cent_z

#     # now set translation of obj 
#     m[0][3] = get_x(t) + cent_x#+ rot_x 
#     m[1][3] = get_y(t) + cent_y#+ rot_y 
#     m[2][3] = get_z(t) + cent_z#+ rot_z
#     m[3][3] = 1

#     # print "q ", q
#     # print "x ", x
#     # print "y ", y
#     # print "z ", z
#     # print "w ", w
#     # print "t ", t
#     # print "r ", r
#     # # print "trans x/y ", get_x(t), " ", get_y(t) 
#     # # print "cent x/y ", cent_x, " ", cent_y  
#     # # print "rot x/y/z ", rot_x, " ", rot_y, " ",  rot_z
#     # print "m  x/y ", m[0][3] , " ", m[1][3]
#     # print m
    
#     return m

 

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

# global varibles
scale = 0.1
height = 0.5
id = 0

def get_rotation(frame):
    return frame['rotation']

def get_translation(frame):
    return frame['translation']

def get_w(coord):
    global scale
    return coord['w'] * scale 

def get_x(coord):
    global scale
    return coord['x'] * scale

def get_y(coord):
    global scale
    return coord['y'] * scale

def get_z(coord):
    global scale 
    return coord['z'] * scale

def get_region(frame):
    return frame['region']

def get_width(r):
    global scale
    max_x = max( max(r[0][0],r[1][0]), max(r[2][0],r[3][0]))
    min_x = min( min(r[0][0],r[1][0]), min(r[2][0],r[3][0]))
    return (max_x - min_x) * scale 


def get_height(r):
    global height, scale
    return height * scale * scale

def get_depth(r):
    global scale 
    max_y = max( max(r[0][1],r[1][1]), max(r[2][1],r[3][1]))
    min_y = min( min(r[0][1],r[1][1]), min(r[2][1],r[3][1]))
    return (max_y - min_y)  * scale 

    
def quaternion_to_rotation_mat3d(q, t, r):
    global scale
    m = [ [1, 0, 0, 0],
          [0, 1, 0, 0],
          [0, 0, 1, 0],
          [0, 0, 0, 1]]

    x = get_x(q)
    y = get_y(q)
    z = get_z(q)
    w = get_w(q)

    # transposed matrix 
    # first column
    # m[0][0] = 1 - 2 * y * y - 2 * z * z
    # m[1][0] = 2 * x * y + 2 * w * z
    # m[2][0] = 2 * x * z - 2 * w * y
    
    # # second column
    # m[0][1] = 2 * x * y - 2 * w * z
    # m[1][1] = 1 - 2 * x * x - 2 * z * z
    # m[2][1] = 2 * y * z + 2 * w * x
    
    # # third column
    # m[0][2] = 2 * x * z + 2 * w * y
    # m[1][2] = 2 * y * z - 2 * w * x
    # m[2][2] = 1 - 2 * x * x - 2 * y * y
    
    # first row
    m[0][0] = 1 - 2 * y * y - 2 * z * z
    m[0][1] = 2 * x * y + 2 * w * z
    m[0][2] = 2 * x * z - 2 * w * y
    m[0][3] = get_x(t) + (get_depth(r) / 2) 

    # second ros 
    m[1][0] = 2 * x * y - 2 * w * z
    m[1][1] = 1 - 2 * x * x - 2 * z * z
    m[1][2] = 2 * y * z + 2 * w * x
    m[1][3] = get_y(t) + (get_width(r) / 2)

    # third row
    m[2][0] = 2 * x * z + 2 * w * y
    m[2][1] = 2 * y * z - 2 * w * x
    m[2][2] = 1 - 2 * x * x - 2 * y * y
    m[2][3] = get_z(t) 

    # fourth row
    m[3][0] = 0
    m[3][1] = 0
    m[3][2] = 0
    m[3][3] = 1

    return m

def create_header():
    print '''<?xml version="1.0"?>

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
    
<owl:ObjectProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#describedInMap"/>
<owl:ObjectProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#eventOccursAt"/>
<owl:ObjectProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#objectActedOn"/>
<owl:ObjectProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#startTime"/>

<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#widthOfObject"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#heightOfObject"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#depthOfObject"/>

<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m00"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m01"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m02"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m03"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m10"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m11"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m12"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m13"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m20"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m21"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m22"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m23"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m30"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m31"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m32"/>
<owl:DatatypeProperty rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#m33"/>

<owl:Class rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#RotationMatrix3D"/>
<owl:Class rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#SemanticEnvironmentMap"/>
<owl:Class rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#SemanticMapPerception"/>
<owl:Class rdf:about="http://ias.cs.tum.edu/kb/knowrob.owl#TimePoint"/>

 '''
    
def create_footer():
    print '''
</rdf:RDF>

<!-- Generated by yaml2owl.py -->
'''

def create_timepoint(t):
    print ''
    print '<owl:NamedIndividual rdf:about="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#timepoint_{0}">'.format(t)
    print '  <rdf:type rdf:resource="&knowrob;TimePoint"/>'
    print '</owl:Na1medIndividual>'
    print ''

def create_instance(inst, type):
    print ''
    print '<owl:NamedIndividual rdf:about="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#{0}">'.format(inst)
    print '  <rdf:type rdf:resource="http://ias.cs.tum.edu/kb/knowrob.owl#{0}"/>'.format(type)
    print '</owl:NamedIndividual>'
    print ''
    
def create_instance_with_dim(inst,type,width,height,depth, prop=None, objs=None):
    global building_inst
    print ''
    print '<owl:NamedIndividual rdf:about="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#{0}">'.format(inst)
    print '  <rdf:type rdf:resource="&knowrob;{0}"/>'.format(type)
    print '  <knowrob:widthOfObject rdf:datatype="&xsd;float">{0}</knowrob:widthOfObject>'.format(width)
    print '  <knowrob:depthOfObject rdf:datatype="&xsd;float">{0}</knowrob:depthOfObject>'.format(depth)
    print '  <knowrob:heightOfObject rdf:datatype="&xsd;float">{0}</knowrob:heightOfObject>'.format(height)
    print '  <knowrob:describedInMap rdf:resource="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#SemanticEnvironmentMap-{0}"/>'.format(building_inst)
    if type is 'LevelOfAConstruction':
        print '  <knowrob:floorNumber rdf:datatype="&xsd;int">{0}</knowrob:floorNumber>'.format(get_floor_number(inst))
    if objs is not None:
        if objs.has_key(inst):
            create_props(prop, objs[inst])
    print '</owl:NamedIndividual>'
    print ''

def create_type(type):
    print '  <rdf:type rdf:resource="&knowrob;{0}"/>'.format(type)

def create_building(building_inst, has_floors):
    print '<owl:NamedIndividual rdf:about="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#{0}">'.format(building_inst)
    create_type('Building')
    create_props('hasLevels', has_floors[building_inst])
    print '  <knowrob:describedInMap rdf:resource="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#SemanticEnvironmentMap-{0}"/>'.format(building_inst)
    print '</owl:NamedIndividual>'    
    
def create_props(prop, objs):
    for o in objs:
        create_prop(prop,o)
    
def create_prop(prop, obj):
    print '  <knowrob:{0} rdf:resource="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#{1}"/>'.format(prop,obj)

def create_perception_event(obj,mat3d_name,time=0):
    print   ''
    print   '<owl:NamedIndividual rdf:about="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#SemanticMapPerception-{0}">'.format(get_id())
    print   '  <rdf:type rdf:resource="&knowrob;SemanticMapPerception"/>'
    print   '  <knowrob:objectActedOn rdf:resource="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#{0}"/>'.format(obj)
    print   '  <knowrob:eventOccursAt rdf:resource="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#{0}"/>'.format(mat3d_name)
    print   '  <knowrob:startTime rdf:resource="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#timepoint_{0}"/>'.format(time)
    print   '</owl:NamedIndividual>'
    print   ''

def create_rot_mat3d(m):
    name = 'RotationMatrix3D-{0}'.format(get_id())
    print ''
    print '<owl:NamedIndividual rdf:about="http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#{0}">'.format(name)
    print '  <rdf:type rdf:resource="&knowrob;RotationMatrix3D"/>'
    print '  <knowrob:m00 rdf:datatype="&xsd;float">{0}</knowrob:m00>'.format(m[0][0])
    print '  <knowrob:m01 rdf:datatype="&xsd;float">{0}</knowrob:m01>'.format(m[0][1])
    print '  <knowrob:m02 rdf:datatype="&xsd;float">{0}</knowrob:m02>'.format(m[0][2])
    print '  <knowrob:m03 rdf:datatype="&xsd;float">{0}</knowrob:m03>'.format(m[0][3])
    print '  <knowrob:m10 rdf:datatype="&xsd;float">{0}</knowrob:m10>'.format(m[1][0])
    print '  <knowrob:m11 rdf:datatype="&xsd;float">{0}</knowrob:m11>'.format(m[1][1])
    print '  <knowrob:m12 rdf:datatype="&xsd;float">{0}</knowrob:m12>'.format(m[1][2])
    print '  <knowrob:m13 rdf:datatype="&xsd;float">{0}</knowrob:m13>'.format(m[1][3])
    print '  <knowrob:m20 rdf:datatype="&xsd;float">{0}</knowrob:m20>'.format(m[2][0])
    print '  <knowrob:m21 rdf:datatype="&xsd;float">{0}</knowrob:m21>'.format(m[2][1])
    print '  <knowrob:m22 rdf:datatype="&xsd;float">{0}</knowrob:m22>'.format(m[2][2])
    print '  <knowrob:m23 rdf:datatype="&xsd;float">{0}</knowrob:m23>'.format(m[2][3])
    print '  <knowrob:m30 rdf:datatype="&xsd;float">{0}</knowrob:m30>'.format(m[3][0])
    print '  <knowrob:m31 rdf:datatype="&xsd;float">{0}</knowrob:m31>'.format(m[3][1])
    print '  <knowrob:m32 rdf:datatype="&xsd;float">{0}</knowrob:m32>'.format(m[3][2])
    print '  <knowrob:m33 rdf:datatype="&xsd;float">{0}</knowrob:m33>'.format(m[3][3])
    print '</owl:NamedIndividual>'
    print ''
    return name

def get_name(name):
    return name.replace('/','-')

def get_floor_number(name):
    n = name[name.rfind('-') + 1:]
    fn = int(n.rstrip(string.ascii_letters))
    return fn

def get_parent(frame):
   return frame['frame_id']

def get_id():
    global id
    id = id + 1
    return id



#______________________________________________________________________________
# main  

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def help_msg():
    return """
  Usage: yaml2owl.py [-h] <map>

    map               name of the yaml file 

    -h, --help for seeing this msg
"""

def main(argv=None):
    global building_inst
    if argv is None:
        argv = sys.argv
    try:
        try:
            opts, args = getopt.getopt(argv[1:], "h", ["help"])
        except getopt.error, msg:
            raise Usage(msg)

        if ('-h','') in opts or ('--help', '') in opts: #len(args) != 2 or
            raise Usage(help_msg())

        #print >>sys.stderr, args

        # read yaml file
        stream = file(args[0],'r')
        map = yaml.load(stream)

        now = time.time()

        create_header()

        # read building
        building = map.get('map_frame')
        building_inst = 'Building' + get_name(building)
        create_instance('SemanticEnvironmentMap-' + building_inst, 'SemanticEnvironmentMap')

        
        # read rooms
        has_rooms = dict()
        rooms = map.get('room')
        if rooms is not None:
            for r in rooms:

                name = 'RoomInAConstruction' + get_name(r)
                parent = 'LevelOfAConstruction' + get_name(get_parent(rooms[r]))
                
                # remember the parent (floor) of the room, in order to add the hasRooms property to the floor
                if not has_rooms.has_key(parent):
                    has_rooms[parent] = [name] 
                else:
                    has_rooms[parent] = has_rooms[parent] + [name]
                    
                #print parent, " ", has_rooms[parent]

                reg = get_region(rooms[r])   
                q = get_rotation(rooms[r])
                trans = get_translation(rooms[r])
                mat3d = quaternion_to_rotation_mat3d(q,trans,reg)


                # switched depth and width !!!!
                create_instance_with_dim(name,'RoomInAConstruction',get_depth(reg), get_height(reg), get_width(reg)) 
                mat_inst = create_rot_mat3d(mat3d)
                create_perception_event(name, mat_inst, now)


        
        # read floors
        # create floors with rooms 
        has_floors = dict()
        floors = map.get('floor')
        if floors is not None:
            for f in floors:

                #print get_floor_number(get_name(f))

                name = 'LevelOfAConstruction' + get_name(f)
                parent = 'Building' + get_name(get_parent(floors[f]))
                
                # remember the parent (building) of the floor, in order to add the hasLevels property to the building
                if not has_floors.has_key(parent):
                    has_floors[parent] = [name] 
                else:
                    has_floors[parent] = has_floors[parent] + [name]

                #print parent, " ", has_floors[parent]
                reg = get_region(floors[f])                    
                q = get_rotation(floors[f])
                trans = get_translation(floors[f])
                mat3d = quaternion_to_rotation_mat3d(q,trans,reg)


                create_instance_with_dim(name,'LevelOfAConstruction',get_width(reg), get_height(reg), get_depth(reg), 'hasRooms', has_rooms) 
                mat_inst = create_rot_mat3d(mat3d)
                create_perception_event(name, mat_inst, now)
                

        # create buidling with floors
        create_building(building_inst, has_floors)
                
        create_footer()

    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2

if __name__ == "__main__":
    sys.exit(main())

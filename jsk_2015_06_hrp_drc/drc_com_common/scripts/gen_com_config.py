#!/usr/bin/env python

import commands
import yaml

def write_launch_and_config_from_yaml(_yaml, _from_config_f, _to_config_f, _from_launch_f, _to_launch_f):
    if _yaml.has_key('buffered_topic'):
        for _buffer_node in (_yaml['buffered_topic'] or []):
            _node = _buffer_node['name']
            _topics = _buffer_node['topics']
            # fc config file
            for _topic in _topics:
                _update_topic = _topic + '_update'
                ## update topic
                _from_config_f.write('  - name: %s\n' % _update_topic)
                _from_config_f.write('    node: None\n')
                _from_config_f.write('    type: publisher\n')
            ## list service
            _from_config_f.write('  - name: /%s/list\n' % _node)
            _from_config_f.write('    node: None\n')
            _from_config_f.write('    type: service\n')
            ## update service
            _from_config_f.write('  - name: /%s/update\n' % _node)
            _from_config_f.write('    node: None\n')
            _from_config_f.write('    type: service\n')
            # ocs launch file
            _to_launch_f.write('<node pkg=\"jsk_topic_tools\" type=\"topic_buffer_client\" name=\"%s_client\" clear_params=\"true\" respawn=\"true\">\n' % _node)
            _to_launch_f.write('  <param name="fixed_rate" value="0.05" />\n')
            _to_launch_f.write('  <param name="update_rate" value="-1" />\n')
            _to_launch_f.write('  <param name="latched" value="true" />\n')
            _to_launch_f.write('  <remap from="/list" to="/%s/list" />\n' % _node)
            _to_launch_f.write('  <remap from="/update" to="/%s/update" />\n' % _node)
            for _topic in _topics:
                _to_launch_f.write('  <remap from="%s_buffered" to="%s" />\n' % (_topic, _topic))
            _to_launch_f.write('</node>\n')
            # fc launch file
            _from_launch_f.write("<node pkg=\"jsk_topic_tools\" type=\"topic_buffer_server\" name=\"%s\" args=\"%s\"  respawn=\"true\">\n" % (_node, ' '.join(_topics)))
            _from_launch_f.write('  <param name="periodic_rate" value="1" />\n')
            _from_launch_f.write('</node>\n')

    if _yaml.has_key('relaid_topic'):
        for _topic in (_yaml['relaid_topic'] or []):
            _node = _topic + '_relay'
            _relaid_topic = _topic + '_relaid'
            # fc config file
            _from_config_f.write('  - name: %s\n' % _relaid_topic)
            _from_config_f.write('    node: None\n')
            _from_config_f.write('    type: publisher\n')
            # ocs launch file
            _to_launch_f.write('<node pkg=\"topic_tools\" type=\"relay\" name=\"%s\" args=\"%s %s _unreliable:=true\"/>\n' % (_node[1:].replace('/', '_'), _relaid_topic, _topic))
            # fc launch file
            _from_launch_f.write('<node pkg=\"topic_tools\" type=\"relay\" name=\"%s\" args=\"%s %s _unreliable:=true\"/>\n' % (_node[1:].replace('/', '_'), _topic, _relaid_topic))


if __name__ == '__main__':
    # open files
    drc_com_common_path = commands.getoutput('rospack find drc_com_common')
    fc2ocs_f = open(drc_com_common_path+'/config/fc2ocs.yaml','r') # open original config file
    ocs2fc_f = open(drc_com_common_path+'/config/ocs2fc.yaml','r') # open original config file
    fc_config_f = open(drc_com_common_path+'/config/fc_config.yaml', 'w') # open config file of FC (field computer)
    ocs_config_f = open(drc_com_common_path+'/config/ocs_config.yaml', 'w') # open config file of OCS (operator control station)
    ocs_launch_f = open(drc_com_common_path+'/launch/ocs_relay.launch', 'w') # open launch file to relay topic OCS
    fc_launch_f = open(drc_com_common_path+'/launch/fc_relay.launch', 'w') # open launch file to relay topic FC

    # start writing files
    ocs_config_f.write('# This launch file is automatically generated.\n')
    fc_config_f.write('# This launch file is automatically generated.\n')
    ocs_launch_f.write('<!-- This launch file is automatically generated. -->\n')
    ocs_launch_f.write('<launch>\n')
    fc_launch_f.write('<!-- This launch file is automatically generated. -->\n')
    fc_launch_f.write('<launch>\n')

    # fc2ocs
    fc_config_f.write('default_advertisements:\n')
    ocs_config_f.write('default_pulls:\n')
    ocs_config_f.write('  - gateway: jsk_field_computer.*\n')
    fc2ocs_yaml = yaml.load(fc2ocs_f.read())
    write_launch_and_config_from_yaml(fc2ocs_yaml, fc_config_f, ocs_config_f, fc_launch_f, ocs_launch_f)

    # ocs2fc
    fc_config_f.write('default_pulls:\n')
    fc_config_f.write('  - gateway: jsk_operator_station.*\n')
    ocs_config_f.write('default_advertisements:\n')
    ocs2fc_yaml = yaml.load(ocs2fc_f.read())
    write_launch_and_config_from_yaml(ocs2fc_yaml, ocs_config_f, fc_config_f, ocs_launch_f, fc_launch_f)

    # finish writing files
    ocs_launch_f.write('</launch>')
    fc_launch_f.write('</launch>')

    # close files
    fc2ocs_f.close()
    ocs2fc_f.close()
    fc_config_f.close()
    ocs_config_f.close()
    ocs_launch_f.close()
    fc_launch_f.close()



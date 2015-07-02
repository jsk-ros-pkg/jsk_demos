#!/usr/bin/env python

import sys

def usage():
    print "gen_hosts.py TEAM_NO(13) MY_HOSTNAME(fc20)"

def gen_hosts(team_no, my_hostname):
    hosts_str ="""
#
127.0.0.1 localhost
127.0.1.1 %s
# robot and field computers
%s
# ocs computers
%s
    """
    fc_hostnames = "\n".join(["10.%s.3.%s fc%s" % (team_no, i, i)
                              for i in range(20, 255)
                              if not "fc%s" % i == my_hostname])
    ocs_hostnames = "\n".join(["10.%s.2.%s ocs%s" % (team_no, i, i)
                               for i in range(10, 255)
                               if not "ocs%s" % i == my_hostname])
    print hosts_str % (my_hostname, fc_hostnames, ocs_hostnames)
        
    
if __name__ == "__main__":
    if len(sys.argv) != 3:
        usage()
        sys.exit(-1)
    TEAM_NO = sys.argv[1]
    MY_HOSTNAME = sys.argv[2]
    gen_hosts(TEAM_NO, MY_HOSTNAME)
    

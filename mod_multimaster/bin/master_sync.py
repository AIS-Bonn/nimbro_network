#!/usr/bin/env python

import roslib; roslib.load_manifest('mod_multimaster')

import threading
import time
import urlparse

import rosgraph
import rosgraph.names
import rosgraph.network

import rospy

from rospy.core import global_name, is_topic
from rospy.impl.validators import non_empty, ParameterInvalid

from rospy.impl.masterslave import apivalidate

from rosgraph.xmlrpc import XmlRpcNode, XmlRpcHandler

import roslib.network

import sys

class MasterSync(object):
    def __init__(self):
        local_uri   = rosgraph.get_master_uri()
        foreign_uri = rospy.get_param('~foreign_master', '')
        if not foreign_uri:
            raise Exception('foreign_master URI not specified')

        local_pubs   = rospy.get_param('~local_pubs',   [])
        foreign_pubs = rospy.get_param('~foreign_pubs', [])

        self._local_services   = rospy.get_param('~local_services',   [])
        self._foreign_services = rospy.get_param('~foreign_services', [])

        foreign_master = rosgraph.Master(rospy.get_name(), master_uri=foreign_uri)
        r = rospy.Rate(1)
        while not _is_master_up(foreign_master) and not rospy.is_shutdown():
            rospy.logdebug('Waiting for foreign master to come up...')
            r.sleep()

        self._local_manager   = None
        self._foreign_manager = None
        if not rospy.is_shutdown():
            self._local_manager   = _RemoteManager(local_uri,   self._local_publisher_update)
            self._foreign_manager = _RemoteManager(foreign_uri, self._foreign_publisher_update)
            for t in local_pubs:
                self._local_manager.subscribe(t)
            for t in foreign_pubs:
                self._foreign_manager.subscribe(t)

    def _local_publisher_update(self, topic, publishers):
        topic_type = self._local_manager.get_topic_type(topic)
        
        self._foreign_manager.publishers_updated(topic, topic_type, publishers)

    def _foreign_publisher_update(self, topic, publishers):
        topic_type = self._foreign_manager.get_topic_type(topic)
        
        self._local_manager.publishers_updated(topic, topic_type, publishers)

    def spin(self):
        # @todo: is this excessively hitting the master?

        r = rospy.Rate(1.0)

        while not rospy.is_shutdown():
            for s in self._local_services:
                srv_uri = self._local_manager.lookup_service(s)
                if srv_uri:
                    self._foreign_manager.advertise_service(s, srv_uri)
                else:
                    self._foreign_manager.unadvertise_service(s)

            for s in self._foreign_services:
                srv_uri = self._foreign_manager.lookup_service(s)
                if srv_uri:
                    self._local_manager.advertise_service(s, srv_uri)
                else:
                    self._local_manager.unadvertise_service(s)

            r.sleep()

        if self._local_manager:
            self._local_manager.unsubscribe_all()
        if self._foreign_manager:
            self._foreign_manager.unsubscribe_all()

def _is_master_up(m):
    try:
        m.getUri()
        return True
    except Exception:
        return False

class _RemoteManager(object):
    def __init__(self, master_uri, new_topics_callback):
        self.master_uri          = master_uri
        self.new_topics_callback = new_topics_callback

        name = rosgraph.names.ns_join(rosgraph.names.get_ros_namespace(), rosgraph.names.anonymous_name('master_sync'))
        self.master = rosgraph.Master(name, master_uri=self.master_uri)

        self._lock = threading.RLock()

        self._type_cache = {}

        self._subs = {}
        self._pubs = {}
        self._srvs = {}

        self._external_node = XmlRpcNode(rpc_handler=_TopicPubListenerHandler(self._new_topics))
        self._external_node.start()

        timeout_t = time.time() + 5.0
        while time.time() < timeout_t and self._external_node.uri is None:
            time.sleep(0.01)

    def get_topic_type(self, query_topic):
        query_topic = self.resolve(query_topic)

        query_topic_type = self._type_cache.get(query_topic)
        if query_topic_type:
            return query_topic_type

        for topic, topic_type in self.master.getTopicTypes():
            self._type_cache[topic] = topic_type

        return self._type_cache.get(query_topic, '*')

    def subscribe(self, topic):
        topic = self.resolve(topic)
        publishers = self.master.registerSubscriber(topic, '*', self._external_node.uri)        
        self._subs[(topic, self._external_node.uri)] = self.master
        self._new_topics(topic, publishers)

    def publishers_updated(self, topic, topic_type, uris):
        resolved = self.resolve(topic)

        with self._lock:
            # Unregister any publishers that no longer exist
            uris_set = set(uris)
            for t, uri in self._pubs.keys():
                if t == resolved and uri not in uris_set:
                    self.unadvertise(t, uri)
    
            # Register new publishers
            for uri in uris:
                if (resolved, uri) not in self._pubs:
                    # Registrations need to be anonymous so master doesn't kill us if there's a duplicate name
                    rospy.loginfo('Registering (%s, %s) on master %s' % (resolved, uri, self.master_uri))
                    anon_name = rosgraph.names.anonymous_name('master_sync')
                    master = rosgraph.masterapi.Master(anon_name, master_uri=self.master_uri)
                    master.registerPublisher(resolved, topic_type, uri)
    
                    self._pubs[(resolved, uri)] = master

    def unadvertise(self, topic, uri):
        with self._lock:
            if (topic, uri) in self._pubs:
                master = self._pubs[(topic, uri)]
    
                rospy.loginfo('Unregistering (%s, %s) from master %s' % (topic, uri, master.master_uri))
                master.unregisterPublisher(topic, uri)
                del self._pubs[(topic, uri)]

    def lookup_service(self, service_name):
        service_name = self.resolve(service_name)
        try:
            return self.master.lookupService(service_name)
        except:
            return None

    def advertise_service(self, service_name, uri):
        # These registrations need to be anonymous so the master doesn't kill us if there is a duplicate name
        anon_name = rosgraph.names.anonymous_name('master_sync')
        master = rosgraph.masterapi.Master(anon_name, master_uri=self.master_uri)

        if service_name in self._srvs:
            if self._srvs[service_name][0] == uri:
                return
            self.unadvertise_service(service_name)

        rospy.loginfo('Registering service (%s, %s) on master %s' % (service_name, uri, master.master_uri))

        fake_api = 'http://%s:0' % roslib.network.get_host_name()
        master.registerService(service_name, uri, fake_api)

        self._srvs[service_name] = (uri, master)

    def unadvertise_service(self, service_name):
        if service_name in self._srvs:
            uri, master = self._srvs[service_name]
            
            rospy.loginfo('Unregistering service (%s, %s) from master %s' % (service_name, uri, master.master_uri))
            master.unregisterService(service_name, uri)

            del self._srvs[service_name]

    def resolve(self, topic):
        return rosgraph.names.ns_join(rosgraph.names.namespace(self.master.caller_id), topic)

    def unsubscribe_all(self):
        for (topic, uri), master in self._subs.iteritems():
            master.unregisterSubscriber(topic, uri)
        for topic, uri in self._pubs.keys():
            self.unadvertise(topic, uri)
        for service in self._srvs.keys():
            self.unadvertise_service(service)

    def _new_topics(self, topic, publishers):
        self.new_topics_callback(topic, [p for p in publishers if (topic, p) not in self._pubs])

def is_publishers_list(paramName):
    return ('is_publishers_list', paramName)

class _TopicPubListenerHandler(XmlRpcHandler):
    def __init__(self, publisher_update_callback):
        super(_TopicPubListenerHandler, self).__init__()
        
        self.uri                       = None
        self.publisher_update_callback = publisher_update_callback

    def _ready(self, uri):
        self.uri = uri

    def _custom_validate(self, validation, param_name, param_value, caller_id):
        if validation == 'is_publishers_list':
            if not type(param_value) == list:
                raise ParameterInvalid("ERROR: param [%s] must be a list"%param_name)
            for v in param_value:
                if not isinstance(v, basestring):
                    raise ParameterInvalid("ERROR: param [%s] must be a list of strings"%param_name)
                parsed = urlparse.urlparse(v)
                if not parsed[0] or not parsed[1]: #protocol and host
                    raise ParameterInvalid("ERROR: param [%s] does not contain valid URLs [%s]"%(param_name, v))
            return param_value
        else:
            raise ParameterInvalid("ERROR: param [%s] has an unknown validation type [%s]"%(param_name, validation))

    @apivalidate([])
    def getBusStats(self, caller_id):
        # not supported
        return 1, '', [[], [], []]

    @apivalidate([])
    def getBusInfo(self, caller_id):
        # not supported
        return 1, '', [[], [], []]
    
    @apivalidate('')
    def getMasterUri(self, caller_id):
        # not supported
        return 1, '', ''
        
    @apivalidate(0, (None, ))
    def shutdown(self, caller_id, msg=''):
        return -1, 'not authorized', 0

    @apivalidate(-1)
    def getPid(self, caller_id):
        return -1, 'not authorized', 0

    # pub/sub

    @apivalidate([])
    def getSubscriptions(self, caller_id):
        return 1, 'subscriptions', [[], []]

    @apivalidate([])
    def getPublications(self, caller_id):
        return 1, 'publications', [[], []]
    
    @apivalidate(-1, (global_name('parameter_key'), None))
    def paramUpdate(self, caller_id, parameter_key, parameter_value):
        # not supported
        return -1, 'not authorized', 0

    @apivalidate(-1, (is_topic('topic'), is_publishers_list('publishers')))
    def publisherUpdate(self, caller_id, topic, publishers):
        self.publisher_update_callback(topic, publishers)
    
    @apivalidate([], (is_topic('topic'), non_empty('protocols')))
    def requestTopic(self, caller_id, topic, protocols):
        return 0, 'no supported protocol implementations', []

if __name__ == '__main__':
    try:
        rospy.init_node('master_sync')
        
        sync = MasterSync()
        sync.spin()
    except KeyboardInterrupt:
        pass
    except Exception as ex:
        import traceback
        rospy.logfatal(traceback.format_exc()) 
        rospy.logfatal(str(ex))
        time.sleep(1.0)
        sys.exit(1)

#!/usr/bin/env python
import requests
import json
from requests.auth import HTTPBasicAuth

class MKHubBridge(object):
    def __init__(self, server, namespace, challenge):
        self.teamkey = 'bb3d3002-4d51-4f01-9e79-b209870a4f04'
        self.url = server + namespace + '/' + challenge
    
    # put to the MK data hub, returns a boolean for operation status
    def put(parameter, payload):
        try:
            print 'MKHub: posting to {}'.format(parameter)
            url = self.url + '/' + parameter
            response = requests.request("PUT", url, data=payload, auth=HTTPBasicAuth(self.teamkey, ''))
            print 'MKHub: {}'.format(response)

        except requests.exceptions.ConnectionError as e:
            print ('Error: could not connect to MK hub!')
            return 0

        except ValueError as e:
            print 'Error: {}. Is the payload correct?'.format(e)
            return 0

        return 1

    # post to the MK data hub, returns a boolean for operation status
    def post(parameter, payload):
        try:
            print 'MKHub: posting to {}'.format(parameter)
            url = self.url + '/' + parameter
            response = requests.request("POST", url, data=payload, auth=HTTPBasicAuth(self.teamkey, ''))
            print 'MKHub: {}'.format(response)

        except requests.exceptions.ConnectionError as e:
            print ('Error: could not connect to MK hub!')
            return 0

        except ValueError as e:
            print 'Error: {}. Is the payload correct?'.format(e)
            return 0

        return 1

    # get operation from the MK data hub, returns an empty dictionary if nothing is found
    def get(parameter=''):
        try:
            if parameter != '':
                url = self.url + '/' + parameter
            else:
                url = self.url
            response = requests.request("GET", url, auth=HTTPBasicAuth(teamkey, ''))
            data = response.json()

        except requests.exceptions.ConnectionError as e:
            print ('Error: could not connect to MK hub!')
            return {}

        except ValueError as e:
            print('ValueError: no JSON to decode. Is the parameter correct?')
            return {}

        return data
    
    # GET TABLE
    def getTable(id):
        data = get(id)
        print '{}:'.format(data[0]['@id'])
        print '\tType: {}'.format(data[0]['@type'])
        print '\tCustomers: {}'.format(data[0]['customers'])
        print '\tStatus: {}'.format(data[0]['status'])
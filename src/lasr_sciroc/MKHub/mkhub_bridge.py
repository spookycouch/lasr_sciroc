#!/usr/bin/env python
import requests
import json
import datetime
from requests.auth import HTTPBasicAuth

class MKHubBridge(object):
    def __init__(self, server, namespace, challenge):
        self.teamkey = 'bb3d3002-4d51-4f01-9e79-b209870a4f04'
        self.url = server + '/' + namespace + '/' + challenge
    
    # PUT request to the MK data hub, returns a boolean for operation status
    def put(self, parameter, payload):
        try:
            print 'MKHub: putting to {}'.format(parameter)
            url = self.url + '/' + parameter
            print('URL is: {}'.format(url))
            response = requests.request("PUT", url, data=payload, auth=HTTPBasicAuth(self.teamkey, ''))
            print 'MKHub: {}'.format(response)

        except requests.exceptions.ConnectionError as e:
            print ('Error: could not connect to MK hub!')
            return response

        except ValueError as e:
            print 'Error: {}. Is the payload correct?'.format(e)
            return response

        return response

    # post to the MK data hub, returns a boolean for operation status
    def post(self, parameter, payload):
        try:
            print 'MKHub: posting to {}'.format(parameter)
            url = self.url + '/' + parameter
            print('URL is: {}'.format(url))
            response = requests.request("POST", url, data=payload, auth=HTTPBasicAuth(self.teamkey, ''))
            print 'MKHub: {}'.format(response)

        except requests.exceptions.ConnectionError as e:
            print ('Error: could not connect to MK hub!')
            return response

        except ValueError as e:
            print 'Error: {}. Is the payload correct?'.format(e)
            return response

        return response

    # get operation from the MK data hub, returns an empty dictionary if nothing is found
    def get(self, parameter=''):
        try:
            if parameter != '':
                url = self.url + '/' + parameter
            else:
                url = self.url
            print('URL is: {}'.format(url))
            response = requests.request("GET", url, auth=HTTPBasicAuth(self.teamkey, ''))
            data = response.json()

        except requests.exceptions.ConnectionError as e:
            print ('Error: could not connect to MK hub!')
            return {}, response

        except ValueError as e:
            print('ValueError: no JSON to decode. Is the parameter correct?')
            return {}, response

        return data, response

    def constructRobotStatusPayload(self, status_message, episode, x, y, z):
        data = {}
        data['@id'] = 'Tiago'
        data['@type'] = 'RobotStatus'
        data['message'] = status_message
        data['episode'] = episode
        data['team'] = 'leedsasr'
        data['timestamp'] = datetime.datetime.now().isoformat()
        data['x'] = x
        data['y'] = y
        data['z'] = z
        payload = json.dumps(data)
        return payload

    def constructRobotLocationPayload(self, episode, x, y, z):
        data = {}
        data['@id'] = 'Tiago'
        data['@type'] = 'RobotLocation'
        data['episode'] = episode
        data['team'] = 'leedsasr'
        data['timestamp'] = datetime.datetime.now().isoformat()
        data['x'] = x
        data['y'] = y
        data['z'] = z
        payload = json.dumps(data)
        return payload
    
    def constructTablePayload(self, id, people_count, status):
        data = {}
        data['@id'] = id
        data['@type'] = 'Table'
        data['customers'] = people_count
        data['status'] = status
        payload = json.dumps(data)
        return payload

    def constructOrderPayload(self, id, products, status):
        data = {}
        data['@id'] = id
        data['@type'] = 'Order'
        data['table'] = id
        data['timestamp'] = datetime.datetime.now().isoformat()
        data['products'] = products
        data['status'] = status
        payload = json.dumps(data)
        return payload